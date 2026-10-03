#!/usr/bin/env python3
"""Export weights/drive_net.pt to lightweight inference formats for the Pi.

Always:
  drive_net.torchscript        TorchScript (runs wherever torch does)
  drive_net.onnx               ONNX FP32, static batch 1

Optional:
  --fp16    drive_net_fp16.onnx    weights in FP16, float32 I/O (half the size)
  --int8    drive_net_int8.onnx    static INT8 (QDQ, per-channel), calibrated
                                   on real frames -- the fastest CPU path on
                                   the Pi 5's Cortex-A76 via onnxruntime
  --tflite  drive_net_fp16.tflite / drive_net_int8.tflite (+ _fp32) through
            onnx2tf (needs `pip install tensorflow onnx2tf onnx_graphsurgeon
            sng4onnx`; heavy, so run it on Colab/PC, not on the Pi)

INT8 calibration needs representative frames: --calib points at one or more
tubs (or a folder of images). Every frame goes through the exact production
preprocessing (vision/drive_net.py:preprocess), so the quantization ranges are
the ones the car will actually see. Without --calib the export falls back to
synthetic noise and warns -- the model will run, but its ranges are guesses.

Every export is checked against the PyTorch model on the calibration frames
(steering RMSE in px), and --bench times each file on this CPU so the INT8
speed-up is measured, not assumed. Run --bench ON THE PI for numbers that
mean anything for the car.

NCNN (the format the sign detector uses) can still be produced from the
TorchScript with pnnx:  pnnx drive_net.torchscript inputshape=[1,3,66,200]

Usage:
    python TMR2026/tools/export_drive.py
    python TMR2026/tools/export_drive.py --fp16 --int8 --calib datasets/real_a --bench
    python TMR2026/tools/export_drive.py --tflite --int8 --calib datasets/real_a
"""
from __future__ import annotations

import argparse
import glob
import os
import shutil
import sys
import time
from pathlib import Path

import cv2
import numpy as np

try:
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")
except Exception:
    pass

HERE = Path(__file__).resolve().parent
TMR_ROOT = HERE.parent
sys.path.insert(0, str(TMR_ROOT))

from vision.drive_net import (IMG_H, IMG_W, ROI_BOTTOM, ROI_FRAC, load_checkpoint,
                              make_runner, preprocess)

DEFAULT_WEIGHTS = TMR_ROOT / "weights" / "drive_net.pt"


# ── calibration data ────────────────────────────────────────────────────────

def calib_tensors(spec: str, meta: dict, n: int, seed: int = 0) -> np.ndarray:
    """Up to n preprocessed frames, shape (N,3,H,W) float32."""
    img_w = int(meta.get("img_w", IMG_W))
    img_h = int(meta.get("img_h", IMG_H))
    roi_frac = float(meta.get("roi_frac", ROI_FRAC))
    roi_bottom = float(meta.get("roi_bottom", ROI_BOTTOM))

    paths = []
    for raw in (spec or "").split(","):
        raw = raw.strip()
        if not raw:
            continue
        p = Path(raw)
        if not p.is_absolute() and not p.exists() and (TMR_ROOT / p).exists():
            p = TMR_ROOT / p
        if p.is_dir():
            sub = p / "frames" if (p / "frames").is_dir() else p
            for ext in ("*.jpg", "*.jpeg", "*.png"):
                paths += sorted(glob.glob(str(sub / ext)))
        else:
            paths += sorted(glob.glob(str(p)))

    if not paths:
        print("[EXPORT] WARNING: no --calib frames; INT8 ranges come from noise. "
              "Pass a recorded tub for a usable INT8 model.")
        rng = np.random.default_rng(seed)
        return rng.uniform(0, 1, (min(n, 32), 3, img_h, img_w)).astype(np.float32)

    rng = np.random.default_rng(seed)
    if len(paths) > n:
        paths = [paths[i] for i in sorted(rng.choice(len(paths), n, replace=False))]
    xs = []
    for p in paths:
        img = cv2.imread(p)
        if img is not None:
            xs.append(preprocess(img, img_w, img_h, roi_frac, roi_bottom))
    print(f"[EXPORT] calibration: {len(xs)} frames from {spec}")
    return np.stack(xs).astype(np.float32)


# ── exporters ───────────────────────────────────────────────────────────────

def export_onnx(model, dummy, path: Path, opset: int) -> None:
    import torch
    kw = dict(input_names=["frame"], output_names=["out"], opset_version=opset)
    try:
        torch.onnx.export(model, dummy, str(path), dynamo=False, **kw)
    except TypeError:                     # torch < 2.5 has no `dynamo` kwarg
        torch.onnx.export(model, dummy, str(path), **kw)


def export_fp16(src: Path, dst: Path) -> None:
    import onnx
    try:
        from onnxconverter_common import float16
    except ImportError:
        from onnxruntime.transformers import float16
    m = float16.convert_float_to_float16(onnx.load(str(src)), keep_io_types=True)
    onnx.save(m, str(dst))


def export_int8(src: Path, dst: Path, calib: np.ndarray) -> None:
    from onnxruntime.quantization import (CalibrationDataReader, QuantFormat,
                                          QuantType, quantize_static)
    try:
        from onnxruntime.quantization.shape_inference import quant_pre_process
        pre = dst.with_name(dst.stem + "_pre.onnx")
        quant_pre_process(str(src), str(pre))
        src_q = pre
    except Exception:
        src_q = src

    class Reader(CalibrationDataReader):
        def __init__(self, data):
            self.it = iter([{"frame": x[None]} for x in data])

        def get_next(self):
            return next(self.it, None)

    quantize_static(str(src_q), str(dst), Reader(calib),
                    quant_format=QuantFormat.QDQ, per_channel=True,
                    activation_type=QuantType.QInt8, weight_type=QuantType.QInt8)
    if src_q != src:
        Path(src_q).unlink(missing_ok=True)


def export_tflite(onnx_path: Path, calib: np.ndarray, want_int8: bool) -> list:
    """ONNX -> TFLite via onnx2tf. Returns the produced files."""
    try:
        import onnx2tf
    except ImportError:
        print("[EXPORT] TFLite skipped: pip install tensorflow onnx2tf "
              "onnx_graphsurgeon sng4onnx  (Colab: see notebooks/train_colab.ipynb)")
        return []

    work = onnx_path.with_name("drive_net_tflite").resolve()
    shutil.rmtree(work, ignore_errors=True)
    work.mkdir(parents=True)
    nhwc = np.ascontiguousarray(np.transpose(calib, (0, 2, 3, 1)))  # onnx2tf: NHWC
    kw = {}
    if want_int8:
        npy = work / "calib.npy"
        np.save(npy, nhwc)
        kw = dict(output_integer_quantized_tflite=True,
                  custom_input_op_name_np_data_path=[["frame", str(npy), 0.0, 1.0]],
                  quant_type="per-channel")
    # onnx2tf validates any 3-channel image model against a sample file it
    # downloads from its GitHub releases -- and that asset returns 404, which
    # aborts the conversion. It looks in the current directory first, so put
    # our own (real, correctly-shaped) frames there under the expected name.
    sample = "calibration_image_sample_data_20x128x128x3_float32.npy"
    np.save(work / sample, nhwc[:20])
    cwd = Path.cwd()
    try:
        os.chdir(work)
        onnx2tf.convert(input_onnx_file_path=str(onnx_path.resolve()),
                        output_folder_path=str(work), non_verbose=True, **kw)
    finally:
        os.chdir(cwd)

    produced = []
    mapping = {"float32": "drive_net_fp32.tflite",
               "float16": "drive_net_fp16.tflite",
               "full_integer_quant": "drive_net_int8.tflite"}
    for tag, name in mapping.items():
        hits = sorted(work.glob(f"*_{tag}.tflite"))
        if hits:
            dst = onnx_path.with_name(name)
            shutil.copy(hits[0], dst)
            produced.append(dst)
    shutil.rmtree(work, ignore_errors=True)
    return produced


# ── checks ──────────────────────────────────────────────────────────────────

def parity(path: Path, ref: np.ndarray, xs: np.ndarray, norm_px: float) -> float:
    """Steering RMSE (px) of an export against the PyTorch outputs."""
    run = make_runner(path, "cpu")
    out = np.stack([run(x[None]) for x in xs])
    return float(np.sqrt(np.mean(((out[:, 0] - ref[:, 0]) * norm_px) ** 2)))


def bench(path: Path, x: np.ndarray, iters: int, threads: int) -> float:
    run = make_runner(path, "cpu", threads)
    for _ in range(10):
        run(x)
    t0 = time.perf_counter()
    for _ in range(iters):
        run(x)
    return (time.perf_counter() - t0) / iters * 1000.0


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", default=str(DEFAULT_WEIGHTS))
    ap.add_argument("--fp16", action="store_true", help="ONNX FP16")
    ap.add_argument("--int8", action="store_true", help="ONNX (and TFLite) INT8")
    ap.add_argument("--tflite", action="store_true", help="TFLite via onnx2tf")
    ap.add_argument("--calib", default="", help="tubs / image dirs for calibration")
    ap.add_argument("--calib-n", type=int, default=300)
    ap.add_argument("--opset", type=int, default=17)
    ap.add_argument("--bench", action="store_true", help="time every export here")
    ap.add_argument("--iters", type=int, default=300)
    ap.add_argument("--threads", type=int, default=0)
    ap.add_argument("--onnx", action="store_true", help=argparse.SUPPRESS)  # legacy
    args = ap.parse_args()

    import torch

    wp = Path(args.weights)
    if not wp.exists():
        sys.exit(f"ERROR: {wp} not found. Train first with tools/train_drive.py")

    model, meta = load_checkpoint(wp, "cpu")
    img_w = int(meta.get("img_w", IMG_W))
    img_h = int(meta.get("img_h", IMG_H))
    norm_px = float(meta.get("norm_px", 150.0))
    dummy = torch.zeros(1, 3, img_h, img_w)
    # The INT8/TFLite exports read the base sidecar for their preprocessing.
    if not wp.with_suffix(".json").exists():
        import json
        wp.with_suffix(".json").write_text(json.dumps(meta, indent=2))

    calib = calib_tensors(args.calib, meta, args.calib_n)
    with torch.no_grad():
        ref = model(torch.from_numpy(calib)).numpy()

    outputs = []

    ts = torch.jit.trace(model, dummy)
    ts_path = wp.with_suffix(".torchscript")
    ts.save(str(ts_path))
    with torch.no_grad():
        d = float((torch.jit.load(str(ts_path))(torch.from_numpy(calib)).numpy() - ref).__abs__().max())
    print(f"[EXPORT] TorchScript -> {ts_path.name}   max_abs_diff={d:.2e}")

    onnx_path = wp.with_suffix(".onnx")
    try:
        export_onnx(model, dummy, onnx_path, args.opset)
        outputs.append(onnx_path)
    except Exception as e:
        sys.exit(f"[EXPORT] ONNX export failed ({e}). pip install onnx onnxscript")

    if args.fp16:
        p = wp.with_name(wp.stem + "_fp16.onnx")
        try:
            export_fp16(onnx_path, p)
            outputs.append(p)
        except Exception as e:
            print(f"[EXPORT] FP16 failed ({e}). pip install onnxconverter-common")

    if args.int8:
        p = wp.with_name(wp.stem + "_int8.onnx")
        try:
            export_int8(onnx_path, p, calib)
            outputs.append(p)
        except Exception as e:
            print(f"[EXPORT] INT8 failed ({e}). pip install onnxruntime")

    if args.tflite:
        try:
            outputs += export_tflite(onnx_path, calib, args.int8)
        except Exception as e:
            print(f"[EXPORT] TFLite failed ({e})")

    print(f"\n[EXPORT] input [1,3,{img_h},{img_w}] YUV/255   norm_px={norm_px}   "
          f"arch={meta.get('arch', 'pilotnet')}")
    print(f"{'file':28s} {'size':>9s} {'RMSE vs torch':>14s}"
          + (f" {'ms/frame':>9s} {'FPS':>7s}" if args.bench else ""))
    x1 = calib[:1]
    for p in outputs:
        size = p.stat().st_size / 1024
        try:
            r = parity(p, ref, calib[:200], norm_px)
            rs = f"{r:10.2f} px"
        except Exception as e:
            rs = f"  ({type(e).__name__})"
        line = f"{p.name:28s} {size:7.0f}KB {rs:>14s}"
        if args.bench:
            try:
                ms = bench(p, x1, args.iters, args.threads)
                line += f" {ms:9.2f} {1000.0 / ms:7.0f}"
            except Exception as e:
                line += f"  bench failed: {e}"
        print(line)
    print("\nDeploy: copy the chosen file to the Pi's TMR2026/weights/ and set "
          "config.py:DRIVE_NET_WEIGHTS to it (with USE_DRIVE_NET = True).")


if __name__ == "__main__":
    main()
