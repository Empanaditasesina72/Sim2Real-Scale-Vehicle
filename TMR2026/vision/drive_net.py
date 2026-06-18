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

`torch` is only imported when the model is actually built/loaded, so importing
this module is free on a Pi that runs the classic pipeline
(config.py:USE_DRIVE_NET defaults to False).
"""
from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np

try:
    from vision.lane_pipeline import LaneResult
except Exception:  # pragma: no cover - keeps DriveNet importable standalone
    @dataclass
    class LaneResult:
        error_px:   float
        confidence: float
        left_x:     Optional[int] = None
        right_x:    Optional[int] = None
        bev_frame:  Optional[np.ndarray] = None
        mask_frame: Optional[np.ndarray] = None


IMG_W = 200
IMG_H = 66
ROI_FRAC = 0.5
DEFAULT_NORM_PX = 150.0


def preprocess(frame_bgr: np.ndarray, img_w: int = IMG_W, img_h: int = IMG_H,
               roi_frac: float = ROI_FRAC) -> np.ndarray:
    """BGR frame -> normalized CHW float32 YUV tensor (numpy).

    Mirrors the classic pipeline ROI (lower part of the frame), resizes to the
    network input and converts to YUV like the original PilotNet. This is the
    single source of truth for preprocessing: training, evaluation and live
    inference all call it, so they can never drift apart.
    """
    h = frame_bgr.shape[0]
    roi = frame_bgr[int(h * roi_frac):, :]
    img = cv2.resize(roi, (img_w, img_h), interpolation=cv2.INTER_AREA)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2YUV).astype(np.float32) / 255.0
    return np.transpose(img, (2, 0, 1)).copy()


def build_model():
    """Construct the PilotNet module (lazy torch import)."""
    import torch
    import torch.nn as nn

    class PilotNet(nn.Module):
        """NVIDIA PilotNet regressor, size-robust via adaptive pooling.

        Outputs 2 logits: [error_norm, confidence_logit]. The adaptive pool to
        (1, 18) keeps horizontal resolution (lateral lane position is exactly
        what encodes steering) while decoupling the head from the input size.
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
                nn.Linear(10, 2),
            )

        def forward(self, x):
            return self.head(self.pool(self.features(x)))

    return PilotNet()


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

    model = build_model()
    model.load_state_dict(state)
    model.to(device).eval()
    return model, meta


class DriveNet:
    """Learned steering policy with the LanePipeline.process() contract.

    Parameters
    ----------
    weights_path : str | Path
        Path to weights/drive_net.pt (a .json sidecar with norm/img size is
        loaded automatically if present).
    device : str | None
        'cuda' / 'cpu'. Auto-selected when None.
    conf_min : float
        Predictions below this confidence are reported with their raw value but
        flagged via `confidence`, so the FSM can fall back / hold like it does
        for a weak classic detection.
    """

    def __init__(self, weights_path, device: Optional[str] = None,
                 conf_min: float = 0.30, debug: bool = False, **_ignored) -> None:
        import torch
        self._torch = torch
        self.device = device or ("cuda" if torch.cuda.is_available() else "cpu")

        self.model, meta = load_checkpoint(weights_path, self.device)
        self.img_w = int(meta.get("img_w", IMG_W))
        self.img_h = int(meta.get("img_h", IMG_H))
        self.roi_frac = float(meta.get("roi_frac", ROI_FRAC))
        self.norm_px = float(meta.get("norm_px", DEFAULT_NORM_PX))
        self._conf_min = float(conf_min)
        self._debug = bool(debug)
        self._last_error = 0.0

    def process(self, frame: np.ndarray) -> LaneResult:
        """Predict the lane error for a BGR frame (never blocks, never raises)."""
        try:
            x = preprocess(frame, self.img_w, self.img_h, self.roi_frac)
            t = self._torch.from_numpy(x).unsqueeze(0).to(self.device)
            with self._torch.no_grad():
                out = self.model(t)[0]
            error_px = float(out[0].item()) * self.norm_px
            confidence = float(self._torch.sigmoid(out[1]).item())
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
