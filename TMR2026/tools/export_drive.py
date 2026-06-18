#!/usr/bin/env python3
"""Export weights/drive_net.pt for deployment on the Pi.

Produces:
  weights/drive_net.torchscript   always (no extra deps; run via torch on the Pi)
  weights/drive_net.onnx          best-effort (for onnxruntime; needs torch.onnx)

NCNN (fastest on the Pi 5 ARM CPU, same path the sign model uses) is generated
from the TorchScript with pnnx, which is a separate Linux/Windows tool:
    pip install pnnx
    pnnx weights/drive_net.torchscript inputshape=[1,3,66,200]
-> drive_net.ncnn.param / drive_net.ncnn.bin

A parity check confirms the exported graph matches the eager model.

Usage:
    python TMR2026/tools/export_drive.py
    python TMR2026/tools/export_drive.py --onnx --opset 17
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

try:
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")
except Exception:
    pass

HERE = Path(__file__).resolve().parent
TMR_ROOT = HERE.parent
sys.path.insert(0, str(TMR_ROOT))

from vision.drive_net import load_checkpoint, IMG_W, IMG_H

DEFAULT_WEIGHTS = TMR_ROOT / "weights" / "drive_net.pt"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", default=str(DEFAULT_WEIGHTS))
    ap.add_argument("--onnx", action="store_true", help="also try ONNX export")
    ap.add_argument("--opset", type=int, default=17)
    args = ap.parse_args()

    import torch

    wp = Path(args.weights)
    if not wp.exists():
        print(f"ERROR: {wp} not found. Train first with tools/train_drive.py")
        sys.exit(1)

    model, meta = load_checkpoint(wp, "cpu")
    img_w = int(meta.get("img_w", IMG_W))
    img_h = int(meta.get("img_h", IMG_H))
    dummy = torch.randn(1, 3, img_h, img_w)

    with torch.no_grad():
        ref = model(dummy)

    ts = torch.jit.trace(model, dummy)
    ts_path = wp.with_suffix(".torchscript")
    ts.save(str(ts_path))
    with torch.no_grad():
        ts_out = torch.jit.load(str(ts_path))(dummy)
    err = float((ts_out - ref).abs().max())
    print(f"[EXPORT] TorchScript -> {ts_path}   max_abs_diff={err:.2e}")
    if err > 1e-4:
        print("[EXPORT] WARNING: TorchScript parity drift > 1e-4")

    if args.onnx:
        onnx_path = wp.with_suffix(".onnx")
        try:
            torch.onnx.export(
                model, dummy, str(onnx_path),
                input_names=["frame"], output_names=["out"],
                opset_version=args.opset,
                dynamic_axes={"frame": {0: "batch"}, "out": {0: "batch"}},
            )
            print(f"[EXPORT] ONNX -> {onnx_path}")
            try:
                import onnxruntime as ort
                sess = ort.InferenceSession(str(onnx_path),
                                            providers=["CPUExecutionProvider"])
                o = sess.run(None, {"frame": dummy.numpy()})[0]
                print(f"[EXPORT] onnxruntime parity max_abs_diff="
                      f"{np.abs(o - ref.numpy()).max():.2e}")
            except Exception as e:
                print(f"[EXPORT] (onnxruntime check skipped: {e})")
        except Exception as e:
            print(f"[EXPORT] ONNX export failed ({e}).\n"
                  f"          pip install onnx  (and onnxscript) then retry, or "
                  f"deploy the TorchScript instead.")

    print(f"[EXPORT] input shape [1,3,{img_h},{img_w}]  norm_px={meta.get('norm_px')}")
    print("[EXPORT] For NCNN on the Pi:  pip install pnnx && "
          f"pnnx {ts_path.name} inputshape=[1,3,{img_h},{img_w}]")


if __name__ == "__main__":
    main()
