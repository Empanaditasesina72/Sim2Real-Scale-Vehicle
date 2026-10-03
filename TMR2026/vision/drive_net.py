"""End-to-end steering network (behavioral cloning) -- drop-in for LanePipeline.

`DriveNet.process(frame)` returns a `LaneResult(error_px, confidence)`, the exact
same contract as `vision/lane_pipeline.py:LanePipeline.process`. That means the
learned driver can replace the classic CV lane follower WITHOUT touching the FSM,
the PID, the sign gating or the vehicle lights: `main.py` keeps feeding
`fsm.lane_error = result.error_px` just like today.

What the network predicts (PilotNet-style CNN, two outputs):
  - error_px : lane error in pixels, same sign convention as LanePipeline
               (lane_centre - frame_centre; positive = lane is to the right).
  - confidence : in [0, 1], so the FSM can treat a low-confidence prediction the
               same way it treats a low-confidence classic detection.

Data + training live in:
  - tools/gen_synth_driving.py  -> synthetic domain-randomized tub (no sim/track)
  - tools/record_driving.py     -> real tub from the Unity sim or the Pi camera
  - tools/train_drive.py        -> trains weights/drive_net.pt (+ drive_net.json)
  - tools/test_drive_net.py     -> offline evaluation / preview
  - tools/export_drive.py       -> TorchScript / ONNX for the Pi
See TMR2026/docs/DRIVE_NET.md for the full workflow.

Architectures (`build_model(arch=...)`, recorded in the checkpoint meta):
  - "pilotnet"  : NVIDIA PilotNet (5 conv + 4 FC), ~0.25 M params. Default.
  - "mobilenet" : MobileNetV2-style inverted residuals with depthwise convs,
                  ~0.17 M params; ReLU6 + BatchNorm so it quantizes to INT8.
An optional third output predicts a normalized throttle in [0, 1] when the
training tubs carry a `throttle` column (`--throttle` in train_drive.py). The
FSM still owns the speed; DriveNet only exposes it as `last_throttle`.

Inference backends, picked by the weights file extension:
  - .pt      PyTorch (needs torch)
  - .onnx    onnxruntime (FP32 / FP16 / INT8 from tools/export_drive.py) --
             no torch on the Pi, and the fastest CPU path for the INT8 model
  - .tflite  tflite-runtime / ai-edge-litert (FP32 / FP16 / INT8)

`torch` is only imported when the model is actually built/loaded, so importing
this module is free on a Pi that runs the classic pipeline
(config.py:USE_DRIVE_NET defaults to False).
"""
from __future__ import annotations

import json
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

try:
    from vision.lane_pipeline import LaneResult
except Exception:  # pragma: no cover - keeps DriveNet importable standalone
    @dataclass
    class LaneResult:
        error_px:   float
        confidence: float
        heading:    float = 0.0          # main.py reads it every tick
        left_x:     Optional[int] = None
        right_x:    Optional[int] = None
        bev_frame:  Optional[np.ndarray] = None
        mask_frame: Optional[np.ndarray] = None


IMG_W = 200
IMG_H = 66
ROI_FRAC = 0.5
ROI_BOTTOM = 1.0
DEFAULT_NORM_PX = 150.0
DEFAULT_ARCH = "pilotnet"
ARCHS = ("pilotnet", "mobilenet")


def preprocess(frame_bgr: np.ndarray, img_w: int = IMG_W, img_h: int = IMG_H,
               roi_frac: float = ROI_FRAC,
               roi_bottom: float = ROI_BOTTOM) -> np.ndarray:
    """BGR frame -> normalized CHW float32 YUV tensor (numpy).

    Region of interest: rows [roi_frac*h, roi_bottom*h). The top of the frame is
    walls, furniture and the horizon -- exactly what fooled the classic pipeline
    on the city track -- so the network never sees it. `roi_bottom` < 1 also cuts
    the chassis/bumper if the camera ever sees it.

    Resizes to the network input and converts to YUV like the original PilotNet.
    This is the single source of truth for preprocessing: training, evaluation,
    export calibration and live inference all call it, so they cannot drift.
    """
    h = frame_bgr.shape[0]
    y0 = int(h * roi_frac)
    y1 = max(y0 + 1, int(h * roi_bottom))
    roi = frame_bgr[y0:y1, :]
    img = cv2.resize(roi, (img_w, img_h), interpolation=cv2.INTER_AREA)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2YUV).astype(np.float32) / 255.0
    return np.transpose(img, (2, 0, 1)).copy()


def build_model(arch: str = DEFAULT_ARCH, n_out: int = 2, width: float = 1.0,
                img_w: int = IMG_W, img_h: int = IMG_H):
    """Construct the steering network (lazy torch import).

    n_out = 2 -> [error_norm, confidence_logit]
    n_out = 3 -> [error_norm, confidence_logit, throttle_logit]
    `width` scales the MobileNet channel counts (ignored by PilotNet).
    `img_w/img_h` fix the MobileNet pooling so the graph is static (see there).
    """
    if arch == "pilotnet":
        return _build_pilotnet(n_out)
    if arch == "mobilenet":
        return _build_mobilenet(n_out, width, img_w, img_h)
    raise ValueError(f"unknown arch {arch!r}; choose one of {ARCHS}")


def count_params(model) -> int:
    return sum(p.numel() for p in model.parameters())


def _build_pilotnet(n_out: int):
    import torch.nn as nn

    class PilotNet(nn.Module):
        """NVIDIA PilotNet regressor, size-robust via adaptive pooling.

        Outputs n_out logits: [error_norm, confidence_logit(, throttle_logit)].
        The adaptive pool to (1, 18) keeps horizontal resolution (lateral lane
        position is exactly what encodes steering) while decoupling the head
        from the input size.
        """

        def __init__(self) -> None:
            super().__init__()
            self.features = nn.Sequential(
                nn.Conv2d(3, 24, 5, stride=2), nn.BatchNorm2d(24), nn.ELU(),
                nn.Conv2d(24, 36, 5, stride=2), nn.BatchNorm2d(36), nn.ELU(),
                nn.Conv2d(36, 48, 5, stride=2), nn.BatchNorm2d(48), nn.ELU(),
                nn.Conv2d(48, 64, 3), nn.ELU(),
                nn.Conv2d(64, 64, 3), nn.ELU(),
            )
            self.pool = nn.AdaptiveAvgPool2d((1, 18))
            self.head = nn.Sequential(
                nn.Flatten(),
                nn.Dropout(0.3),
                nn.Linear(64 * 18, 100), nn.ELU(),
                nn.Linear(100, 50), nn.ELU(),
                nn.Linear(50, 10), nn.ELU(),
                nn.Linear(10, n_out),
            )

        def forward(self, x):
            return self.head(self.pool(self.features(x)))

    return PilotNet()


def _build_mobilenet(n_out: int, width: float = 1.0, img_w: int = IMG_W,
                     img_h: int = IMG_H):
    import torch
    import torch.nn as nn

    def ch(c: int) -> int:
        return max(8, int(round(c * width / 8)) * 8)

    class InvertedResidual(nn.Module):
        """MobileNetV2 block: 1x1 expand -> 3x3 depthwise -> 1x1 linear project."""

        def __init__(self, cin: int, cout: int, stride: int, expand: int) -> None:
            super().__init__()
            hid = cin * expand
            self.use_res = stride == 1 and cin == cout
            layers = []
            if expand != 1:
                layers += [nn.Conv2d(cin, hid, 1, bias=False),
                           nn.BatchNorm2d(hid), nn.ReLU6(inplace=True)]
            layers += [nn.Conv2d(hid, hid, 3, stride, 1, groups=hid, bias=False),
                       nn.BatchNorm2d(hid), nn.ReLU6(inplace=True),
                       nn.Conv2d(hid, cout, 1, bias=False), nn.BatchNorm2d(cout)]
            self.block = nn.Sequential(*layers)

        def forward(self, x):
            return x + self.block(x) if self.use_res else self.block(x)

    class DriveMobileNet(nn.Module):
        """Lightweight MobileNetV2-style regressor for 66x200 YUV crops.

        ReLU6 + BatchNorm everywhere (folds into the convs at export) so the
        INT8 model loses very little accuracy. Like PilotNet, the pool keeps a
        row of horizontal cells: the lateral position of the lane IS the answer,
        and a global average pool would throw exactly that away.

        The pool averages over height only, with a kernel sized from the real
        feature map at build time. An AdaptiveAvgPool2d to a width that does
        not divide the map (13 columns for 66x200) cannot be exported to ONNX,
        and a fixed kernel is also what TFLite/INT8 runtimes handle best.
        """

        def __init__(self) -> None:
            super().__init__()
            # (expand, out_channels, repeats, first_stride)
            cfg = [(1, 16, 1, 1), (4, 24, 2, 2), (4, 32, 2, 2),
                   (4, 48, 2, 2), (4, 64, 1, 1)]
            cin = ch(16)
            layers = [nn.Conv2d(3, cin, 3, 2, 1, bias=False),
                      nn.BatchNorm2d(cin), nn.ReLU6(inplace=True)]
            for t, c, n, s in cfg:
                cout = ch(c)
                for i in range(n):
                    layers.append(InvertedResidual(cin, cout, s if i == 0 else 1, t))
                    cin = cout
            last = ch(96)
            layers += [nn.Conv2d(cin, last, 1, bias=False),
                       nn.BatchNorm2d(last), nn.ReLU6(inplace=True)]
            self.features = nn.Sequential(*layers)
            with torch.no_grad():
                fh, fw = self.features.eval()(torch.zeros(1, 3, img_h, img_w)).shape[2:]
            self.features.train()
            self.pool = nn.AvgPool2d((int(fh), 1))
            self.head = nn.Sequential(
                nn.Flatten(),
                nn.Dropout(0.2),
                nn.Linear(last * int(fw), 64), nn.ReLU6(inplace=True),
                nn.Linear(64, n_out),
            )

        def forward(self, x):
            return self.head(self.pool(self.features(x)))

    return DriveMobileNet()


def load_checkpoint(weights_path, device):
    """Load a checkpoint saved by train_drive.py. Returns (model, meta dict)."""
    import torch

    p = Path(weights_path)
    try:
        ckpt = torch.load(p, map_location=device, weights_only=False)
    except TypeError:
        ckpt = torch.load(p, map_location=device)

    if isinstance(ckpt, dict) and "model" in ckpt:
        state = ckpt["model"]
        meta = dict(ckpt.get("meta", {}))
    else:
        state = ckpt
        meta = {}

    sidecar = p.with_suffix(".json")
    if sidecar.exists():
        try:
            meta = {**json.loads(sidecar.read_text()), **meta}
        except Exception:
            pass

    model = build_model(meta.get("arch", DEFAULT_ARCH),
                        int(meta.get("n_out", 2)),
                        float(meta.get("width", 1.0)),
                        int(meta.get("img_w", IMG_W)),
                        int(meta.get("img_h", IMG_H)))
    model.load_state_dict(state)
    model.to(device).eval()
    return model, meta


def _read_sidecar(weights_path) -> dict:
    """Meta for a non-PyTorch export: <name>.json, else the base drive_net.json.

    An export such as drive_net_int8.onnx shares the preprocessing of the
    checkpoint it came from, so the base model's sidecar is a valid fallback.
    """
    p = Path(weights_path)
    base = re.sub(r"_(fp32|fp16|int8|float32|float16|integer_quant|"
                  r"full_integer_quant)$", "", p.stem)
    for cand in (p.with_suffix(".json"), p.with_name(base + ".json"),
                 p.with_name("drive_net.json")):
        if cand.exists():
            try:
                return json.loads(cand.read_text())
            except Exception:
                pass
    return {}


class _TorchRunner:
    """PyTorch backend: x (1,3,H,W float32) -> raw output vector."""

    def __init__(self, path, device: Optional[str]) -> None:
        import torch
        self._torch = torch
        self.device = device or ("cuda" if torch.cuda.is_available() else "cpu")
        self.model, self.meta = load_checkpoint(path, self.device)

    def __call__(self, x: np.ndarray) -> np.ndarray:
        t = self._torch.from_numpy(x).to(self.device)
        with self._torch.no_grad():
            return self.model(t)[0].float().cpu().numpy()


class _OnnxRunner:
    """onnxruntime backend (FP32 / FP16 / INT8 graphs from export_drive.py)."""

    def __init__(self, path, threads: int = 0) -> None:
        import onnxruntime as ort
        so = ort.SessionOptions()
        if threads:
            so.intra_op_num_threads = int(threads)
        self.sess = ort.InferenceSession(str(path), so,
                                         providers=["CPUExecutionProvider"])
        inp = self.sess.get_inputs()[0]
        self.name = inp.name
        # An FP16 graph exported with keep_io_types=False wants float16 input.
        self.dtype = np.float16 if "float16" in inp.type else np.float32
        self.meta = _read_sidecar(path)

    def __call__(self, x: np.ndarray) -> np.ndarray:
        out = self.sess.run(None, {self.name: x.astype(self.dtype)})[0]
        return np.asarray(out[0], dtype=np.float32)


class _TFLiteRunner:
    """TFLite backend. Handles NHWC graphs (onnx2tf) and INT8 quantized I/O."""

    def __init__(self, path, threads: int = 0) -> None:
        Interpreter = None
        for mod in ("ai_edge_litert.interpreter", "tflite_runtime.interpreter",
                    "tensorflow.lite"):
            try:
                m = __import__(mod, fromlist=["Interpreter"])
                Interpreter = m.Interpreter
                break
            except Exception:
                continue
        if Interpreter is None:
            raise ImportError("no TFLite runtime: pip install ai-edge-litert "
                              "(or tflite-runtime)")
        kw = {"num_threads": int(threads)} if threads else {}
        self.itp = Interpreter(model_path=str(path), **kw)
        self.itp.allocate_tensors()
        self.inp = self.itp.get_input_details()[0]
        self.out = self.itp.get_output_details()[0]
        self.nhwc = int(self.inp["shape"][-1]) == 3
        self.meta = _read_sidecar(path)

    def __call__(self, x: np.ndarray) -> np.ndarray:
        if self.nhwc:
            x = np.transpose(x, (0, 2, 3, 1))
        dt = self.inp["dtype"]
        if dt in (np.int8, np.uint8):
            scale, zp = self.inp["quantization"]
            x = np.clip(np.round(x / scale + zp), np.iinfo(dt).min,
                        np.iinfo(dt).max)
        self.itp.set_tensor(self.inp["index"], x.astype(dt))
        self.itp.invoke()
        y = self.itp.get_tensor(self.out["index"])[0]
        if self.out["dtype"] in (np.int8, np.uint8):
            scale, zp = self.out["quantization"]
            y = (y.astype(np.float32) - zp) * scale
        return np.asarray(y, dtype=np.float32)


def make_runner(weights_path, device: Optional[str] = None, threads: int = 0):
    """Backend for a weights file, chosen by extension (.pt/.onnx/.tflite)."""
    ext = Path(weights_path).suffix.lower()
    if ext == ".onnx":
        return _OnnxRunner(weights_path, threads)
    if ext == ".tflite":
        return _TFLiteRunner(weights_path, threads)
    return _TorchRunner(weights_path, device)


class DriveNet:
    """Learned steering policy with the LanePipeline.process() contract.

    Parameters
    ----------
    weights_path : str | Path
        weights/drive_net.pt, or an export (.onnx / .tflite) from
        tools/export_drive.py. A .json sidecar with norm/img size/ROI is loaded
        automatically if present.
    device : str | None
        'cuda' / 'cpu' for the PyTorch backend. Auto-selected when None.
    threads : int
        CPU threads for onnxruntime / TFLite (0 = runtime default).
    conf_min : float
        Predictions below this confidence are reported with their raw value but
        flagged via `confidence`, so the FSM can fall back / hold like it does
        for a weak classic detection.
    """

    def __init__(self, weights_path, device: Optional[str] = None,
                 conf_min: float = 0.30, debug: bool = False,
                 threads: int = 0, **_ignored) -> None:
        self._runner = make_runner(weights_path, device, threads)
        meta = self._runner.meta
        self.backend = Path(weights_path).suffix.lower().lstrip(".") or "pt"
        self.img_w = int(meta.get("img_w", IMG_W))
        self.img_h = int(meta.get("img_h", IMG_H))
        self.roi_frac = float(meta.get("roi_frac", ROI_FRAC))
        self.roi_bottom = float(meta.get("roi_bottom", ROI_BOTTOM))
        self.norm_px = float(meta.get("norm_px", DEFAULT_NORM_PX))
        self._conf_min = float(conf_min)
        self._debug = bool(debug)
        self._last_error = 0.0
        self.last_throttle: Optional[float] = None

    def process(self, frame: np.ndarray) -> LaneResult:
        """Predict the lane error for a BGR frame (never blocks, never raises)."""
        try:
            x = preprocess(frame, self.img_w, self.img_h, self.roi_frac,
                           self.roi_bottom)[None]
            out = self._runner(x)
            error_px = float(out[0]) * self.norm_px
            confidence = _sigmoid(float(out[1]))
            if out.shape[0] > 2:
                self.last_throttle = _sigmoid(float(out[2]))
            self._last_error = error_px
        except Exception:
            error_px = self._last_error
            confidence = 0.0

        res = LaneResult(error_px=error_px, confidence=confidence)
        if self._debug:
            res.mask_frame = _overlay(frame, error_px, confidence)
        return res

    def reset(self) -> None:
        self._last_error = 0.0

    def draw_debug(self, frame: np.ndarray, result: LaneResult) -> np.ndarray:
        """Same overlay contract as LanePipeline.draw_debug (annotated copy)."""
        vis = frame.copy()
        h, w = vis.shape[:2]
        cv2.line(vis, (w // 2, h), (w // 2, h // 2), (0, 150, 150), 1)
        cx = max(0, min(w - 1, w // 2 + int(result.error_px)))
        col = (0, 255, 0) if result.confidence >= 0.5 else (0, 80, 255)
        cv2.line(vis, (cx, h), (cx, h // 2), col, 3)
        cv2.putText(vis,
            f"err:{result.error_px:+.0f}px  conf:{result.confidence:.0%} [NET]",
            (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2, cv2.LINE_AA)
        return vis


def _sigmoid(v: float) -> float:
    return float(1.0 / (1.0 + np.exp(-np.clip(v, -30.0, 30.0))))


def predicted_steering_x(frame_w: int, error_px: float) -> int:
    """Pixel x of the predicted lane centre in the original frame (for overlays)."""
    return int(np.clip(frame_w / 2.0 + error_px, 0, frame_w - 1))


def _overlay(frame_bgr: np.ndarray, error_px: float, conf: float) -> np.ndarray:
    vis = frame_bgr.copy()
    h, w = vis.shape[:2]
    cx = w // 2
    px = predicted_steering_x(w, error_px)
    cv2.line(vis, (cx, h), (cx, h - 60), (120, 120, 120), 1)
    cv2.line(vis, (px, h), (px, h - 80), (0, 255, 0), 3)
    cv2.putText(vis, f"err:{error_px:+6.1f}px conf:{conf:.0%}", (8, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    return vis
