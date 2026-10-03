"""Data pipeline for DriveNet training: tubs, label conditioning, augmentation.

Shared by tools/train_drive.py and notebooks/train_colab.ipynb so the PC, the
Colab/Kaggle GPU and any future trainer apply exactly the same transforms.
Inference never imports this module -- the only preprocessing the car runs is
vision/drive_net.py:preprocess (ROI crop + resize + YUV), which the dataset
calls last, after augmentation.

Tub format (written by tools/gen_synth_driving.py and tools/record_driving.py):

    <tub>/frames/000000.jpg ...
    <tub>/labels.csv   frame,error_px,confidence,source[,throttle]

Label conditioning
------------------
* Normalization: error_px / norm_px, clipped to [-CLIP, CLIP], so the regression
  target lives in ~[-1, 1] whatever the camera resolution.
* Temporal smoothing (`smooth_window`): a centred moving average over consecutive
  frames of the SAME tub. The classic pipeline used as a teacher jitters a few
  px frame to frame and a human driver's stick even more; the network should
  imitate the intent, not the jitter. Rows whose source is "synthetic" are never
  smoothed -- those frames are independent draws, and averaging their labels
  would corrupt every one of them.
* Throttle (optional column, any scale): normalized to [0, 1] by `throttle_max`
  and smoothed the same way.
* Balancing (`balance_weights`): a lane follower spends most of its time near
  zero error, so an unweighted regressor learns "drive straight" and under-
  steers in curves. Inverse-frequency weights over |error| bins fix that.

Augmentation (all on the full BGR frame, BEFORE the ROI crop)
------------------------------------------------------------
Photometric, label-free -- for "any kind of light":
  brightness/contrast/gamma, HSV jitter (white balance), soft synthetic shadows,
  specular glare spots, Gaussian/motion blur, sensor noise, JPEG artefacts.
Geometric, label-aware:
  - horizontal flip            -> error *= -1
  - perspective lateral shift  -> error += dx * shift_gain
    The shift grows linearly from the horizon (0 px) to the bottom row (dx px),
    which is what a sideways move of the camera does to a flat road; a plain
    image translation would instead model a yaw. `shift_gain` converts bottom-
    row image pixels into label pixels: 1.0 for the synthetic tubs (labelled in
    image px), ~BEV-px-per-image-px for tubs labelled by LanePipeline.
"""
from __future__ import annotations

import csv
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Sequence

import cv2
import numpy as np

from vision.drive_net import IMG_H, IMG_W, ROI_BOTTOM, ROI_FRAC, preprocess

CLIP = 1.5


@dataclass
class Sample:
    path: str
    error_px: float
    confidence: float
    throttle: Optional[float] = None     # normalized [0, 1], None if unlabelled
    source: str = ""


# ── tubs ────────────────────────────────────────────────────────────────────

def _resolve(raw: str, roots: Sequence[Path]) -> Path:
    tub = Path(raw).expanduser()
    if tub.is_absolute():
        return tub
    for r in roots:
        if (r / tub).exists():
            return r / tub
    return Path.cwd() / tub


def smooth_series(values: np.ndarray, window: int) -> np.ndarray:
    """Centred moving average with edge padding (window forced odd)."""
    if window <= 1 or len(values) < 2:
        return values.astype(np.float64)
    window = int(window) | 1
    pad = window // 2
    v = np.pad(values.astype(np.float64), pad, mode="edge")
    k = np.ones(window) / window
    return np.convolve(v, k, mode="valid")


def read_tubs(spec, roots: Sequence[Path] = (), smooth_window: int = 0,
              throttle_max: float = 100.0) -> List[Sample]:
    """Read comma-separated tub dirs (or a list of them) into Samples.

    Frames within a tub are ordered by file name (the recorders number them
    sequentially) before smoothing.
    """
    if isinstance(spec, (str, Path)):
        parts = [p for p in str(spec).split(",")]
    else:
        parts = [str(p) for p in spec]

    samples: List[Sample] = []
    for raw in parts:
        raw = raw.strip()
        if not raw:
            continue
        tub = _resolve(raw, roots)
        csv_path = tub / "labels.csv"
        if not csv_path.exists():
            print(f"[DATA] WARN: no labels.csv in {tub}")
            continue
        with open(csv_path, newline="") as f:
            rows = sorted(csv.DictReader(f), key=lambda r: r["frame"])
        if not rows:
            continue

        err = np.array([float(r["error_px"]) for r in rows])
        has_thr = "throttle" in rows[0] and all(r.get("throttle") not in (None, "")
                                                 for r in rows)
        thr = (np.clip(np.array([float(r["throttle"]) for r in rows]) / throttle_max,
                       0.0, 1.0) if has_thr else None)
        synthetic = np.array([r.get("source", "").startswith("synth") for r in rows])

        if smooth_window > 1 and not synthetic.all():
            sm = smooth_series(err, smooth_window)
            err = np.where(synthetic, err, sm)
            if thr is not None:
                thr = np.where(synthetic, thr, smooth_series(thr, smooth_window))

        for i, r in enumerate(rows):
            samples.append(Sample(
                path=str(tub / r["frame"]),
                error_px=float(err[i]),
                confidence=float(r.get("confidence", 1.0) or 1.0),
                throttle=None if thr is None else float(thr[i]),
                source=r.get("source", ""),
            ))
    return samples


def split_samples(samples: List[Sample], val_frac: float, seed: int = 0,
                  block: int = 50):
    """Train/val split in contiguous blocks of `block` frames.

    A random per-frame split of a recorded run leaks: frame t is in train and
    the near-identical frame t+1 in val, so validation RMSE looks great and
    measures memorization. Splitting whole blocks keeps neighbours together.
    """
    n = len(samples)
    if n == 0 or val_frac <= 0:
        return samples, []
    rng = np.random.default_rng(seed)
    n_blocks = max(1, (n + block - 1) // block)
    order = rng.permutation(n_blocks)
    n_val_blocks = max(1, int(round(n_blocks * val_frac)))
    val_blocks = set(order[:n_val_blocks].tolist())
    tr, va = [], []
    for i, s in enumerate(samples):
        (va if (i // block) in val_blocks else tr).append(s)
    return tr, va


def balance_weights(samples: List[Sample], norm_px: float, bins: int = 15,
                    max_ratio: float = 10.0) -> np.ndarray:
    """Per-sample inverse-frequency weights over |error| bins, capped."""
    if not samples:
        return np.zeros(0)
    e = np.clip(np.abs([s.error_px / norm_px for s in samples]), 0, CLIP)
    hist, edges = np.histogram(e, bins=bins, range=(0, CLIP))
    idx = np.clip(np.digitize(e, edges[1:-1]), 0, bins - 1)
    freq = hist[idx].astype(np.float64)
    w = 1.0 / np.maximum(freq, 1.0)
    w = np.minimum(w, w.min() * max_ratio)
    return w / w.sum()


# ── augmentation ────────────────────────────────────────────────────────────

@dataclass
class AugConfig:
    """Probabilities and ranges. `AugConfig.off()` disables everything."""
    p_flip: float = 0.5
    p_shift: float = 0.6
    shift_frac: float = 0.12          # max |dx| as a fraction of frame width
    shift_gain: float = 1.0           # label px per bottom-row image px
    horizon_frac: float = 0.45        # where the shift falls to 0 (frame rows)
    p_bright: float = 0.85
    gain: tuple = (0.55, 1.5)
    bias: tuple = (-35.0, 35.0)
    p_gamma: float = 0.3
    gamma: tuple = (0.6, 1.6)
    p_hsv: float = 0.4
    p_shadow: float = 0.5
    shadow_alpha: tuple = (0.25, 0.65)
    p_glare: float = 0.2
    p_blur: float = 0.3
    p_motion_blur: float = 0.15
    p_noise: float = 0.4
    noise_sigma: tuple = (2.0, 10.0)
    p_jpeg: float = 0.2
    extra: dict = field(default_factory=dict)

    @classmethod
    def off(cls) -> "AugConfig":
        z = cls()
        for k, v in vars(z).items():
            if k.startswith("p_"):
                setattr(z, k, 0.0)
        return z


def _shift_lateral(img, dx: float, horizon_frac: float):
    """Shift rows by an amount growing linearly from the horizon to the bottom."""
    h, w = img.shape[:2]
    y_h = h * horizon_frac
    ys, xs = np.mgrid[0:h, 0:w].astype(np.float32)
    k = np.clip((ys - y_h) / max(1.0, (h - 1) - y_h), 0.0, 1.0)
    map_x = xs - dx * k
    return cv2.remap(img, map_x, ys, cv2.INTER_LINEAR,
                     borderMode=cv2.BORDER_REFLECT)


def _shadow(img, rng, alpha_rng):
    """Random soft-edged polygon darkening (trees, people, the car itself)."""
    h, w = img.shape[:2]
    mask = np.zeros((h, w), np.float32)
    for _ in range(int(rng.integers(1, 3))):
        n = int(rng.integers(3, 7))
        cx, cy = rng.uniform(0, w), rng.uniform(h * 0.3, h)
        r = rng.uniform(0.15, 0.6) * w
        ang = np.sort(rng.uniform(0, 2 * np.pi, n))
        pts = np.stack([cx + r * np.cos(ang) * rng.uniform(0.5, 1.5, n),
                        cy + r * np.sin(ang) * rng.uniform(0.3, 1.0, n)], 1)
        cv2.fillPoly(mask, [pts.astype(np.int32)], 1.0)
    k = int(rng.choice([1, 15, 31, 61]))
    if k > 1:
        mask = cv2.GaussianBlur(mask, (k, k), 0)
    a = float(rng.uniform(*alpha_rng))
    out = img.astype(np.float32) * (1.0 - a * mask[..., None])
    return np.clip(out, 0, 255).astype(np.uint8)


def _glare(img, rng):
    """Specular hot spot: overhead lamps reflecting off a glossy track."""
    h, w = img.shape[:2]
    ys, xs = np.mgrid[0:h, 0:w].astype(np.float32)
    cx, cy = rng.uniform(0, w), rng.uniform(0, h)
    sx, sy = rng.uniform(0.05, 0.25) * w, rng.uniform(0.03, 0.15) * h
    g = np.exp(-(((xs - cx) / sx) ** 2 + ((ys - cy) / sy) ** 2))
    out = img.astype(np.float32) + rng.uniform(80, 200) * g[..., None]
    return np.clip(out, 0, 255).astype(np.uint8)


def _motion_blur(img, rng):
    k = int(rng.choice([5, 7, 9]))
    kern = np.zeros((k, k), np.float32)
    if rng.random() < 0.5:
        kern[k // 2, :] = 1.0
    else:
        kern[:, k // 2] = 1.0
    return cv2.filter2D(img, -1, kern / k)


def augment(img: np.ndarray, error_px: float, rng: np.random.Generator,
            cfg: Optional[AugConfig] = None):
    """Return (augmented BGR frame, adjusted error_px). Never mutates `img`."""
    cfg = cfg or AugConfig()
    h, w = img.shape[:2]

    if rng.random() < cfg.p_flip:
        img = np.ascontiguousarray(img[:, ::-1])
        error_px = -error_px

    if rng.random() < cfg.p_shift:
        dx = float(rng.uniform(-cfg.shift_frac, cfg.shift_frac) * w)
        img = _shift_lateral(img, dx, cfg.horizon_frac)
        # Image content moved by +dx at the bottom, so the lane centre is now dx
        # further right of the frame centre.
        error_px += dx * cfg.shift_gain

    if rng.random() < cfg.p_bright:
        g = float(rng.uniform(*cfg.gain))
        b = float(rng.uniform(*cfg.bias))
        img = np.clip(img.astype(np.float32) * g + b, 0, 255).astype(np.uint8)

    if rng.random() < cfg.p_gamma:
        gm = float(rng.uniform(*cfg.gamma))
        lut = np.clip(((np.arange(256) / 255.0) ** gm) * 255.0, 0, 255).astype(np.uint8)
        img = cv2.LUT(img, lut)

    if rng.random() < cfg.p_hsv:
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV).astype(np.int16)
        hsv[..., 0] = (hsv[..., 0] + int(rng.uniform(-12, 12))) % 180
        hsv[..., 1] = np.clip(hsv[..., 1] * rng.uniform(0.6, 1.4), 0, 255)
        img = cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)

    if rng.random() < cfg.p_shadow:
        img = _shadow(img, rng, cfg.shadow_alpha)

    if rng.random() < cfg.p_glare:
        img = _glare(img, rng)

    if rng.random() < cfg.p_blur:
        k = int(rng.choice([3, 5]))
        img = cv2.GaussianBlur(img, (k, k), 0)
    elif rng.random() < cfg.p_motion_blur:
        img = _motion_blur(img, rng)

    if rng.random() < cfg.p_noise:
        sigma = float(rng.uniform(*cfg.noise_sigma))
        n = rng.normal(0.0, sigma, img.shape).astype(np.float32)
        img = np.clip(img.astype(np.float32) + n, 0, 255).astype(np.uint8)

    if rng.random() < cfg.p_jpeg:
        q = int(rng.integers(25, 80))
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, q])
        if ok:
            img = cv2.imdecode(buf, cv2.IMREAD_COLOR)

    return img, error_px


# ── torch dataset ───────────────────────────────────────────────────────────

def make_dataset(samples: List[Sample], norm_px: float, img_w: int = IMG_W,
                 img_h: int = IMG_H, roi_frac: float = ROI_FRAC,
                 roi_bottom: float = ROI_BOTTOM,
                 aug: Optional[AugConfig] = None, seed: int = 0):
    """TubDataset over `samples`. aug=None -> evaluation (no augmentation)."""
    return TubDataset(samples, norm_px, img_w, img_h, roi_frac, roi_bottom,
                      aug, seed)


try:
    import torch

    class TubDataset(torch.utils.data.Dataset):
        """Yields (x[3,H,W] float32, y[3] float32 = err_norm, conf, throttle).

        throttle is NaN where unlabelled; the loss masks it. Module-level (not
        nested) so DataLoader workers can pickle it under the spawn start method.
        """

        def __init__(self, samples, norm_px, img_w, img_h, roi_frac, roi_bottom,
                     aug, seed=0):
            self.samples = samples
            self.norm_px = float(norm_px)
            self.img_w, self.img_h = img_w, img_h
            self.roi_frac, self.roi_bottom = roi_frac, roi_bottom
            self.aug = aug
            self.rng = np.random.default_rng(seed)

        def __len__(self):
            return len(self.samples)

        def __getitem__(self, i):
            s = self.samples[i]
            img = cv2.imread(s.path)
            if img is None:
                raise FileNotFoundError(f"unreadable frame: {s.path}")
            err = s.error_px
            if self.aug is not None:
                img, err = augment(img, err, self.rng, self.aug)
            x = preprocess(img, self.img_w, self.img_h, self.roi_frac,
                           self.roi_bottom)
            thr = np.nan if s.throttle is None else s.throttle
            y = np.array([np.clip(err / self.norm_px, -CLIP, CLIP),
                          s.confidence, thr], dtype=np.float32)
            return torch.from_numpy(x), torch.from_numpy(y)

    def worker_init_fn(worker_id: int) -> None:
        """Give every DataLoader worker its own augmentation stream.

        Without this each worker inherits a copy of the same Generator and they
        all produce identical "random" augmentations, every epoch.
        """
        info = torch.utils.data.get_worker_info()
        if info is not None:
            info.dataset.rng = np.random.default_rng(torch.initial_seed() % 2**32)

except ImportError:          # pragma: no cover - augmentation still usable
    TubDataset = None        # type: ignore

    def worker_init_fn(worker_id: int) -> None:   # type: ignore
        pass
