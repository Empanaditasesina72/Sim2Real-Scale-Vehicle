#!/usr/bin/env python3
"""Procedurally generate a domain-randomized 'road-like' driving tub.

Why this exists: behavioral cloning needs (image -> steering) pairs, and you may
not have the Unity sim or a physical track on hand. This renders synthetic
road scenes with a KNOWN lane error, so the whole pipeline (train -> evaluate ->
export) can be built and validated today, and so the network sees lots of
lighting / shadow / curvature variety -> it generalizes to "any basic road-like
track" instead of memorizing one.

It writes the same tub format the real recorder (tools/record_driving.py) uses:

    <out>/frames/000000.jpg, 000001.jpg, ...
    <out>/labels.csv            header: frame,error_px,confidence,source
    <out>/tub.json              {"width":..,"height":..,"count":..}

The label `error_px` follows the LanePipeline convention: lane_centre - frame
centre, in pixels, positive = lane is to the right (steer right).

Usage:
    python TMR2026/tools/gen_synth_driving.py --n 3000
    python TMR2026/tools/gen_synth_driving.py --n 800 --out datasets/drive_synth_val --seed 7
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
TMR_ROOT = HERE.parent
DEFAULT_OUT = TMR_ROOT / "datasets" / "drive_synth"


def _rand_color(rng, lo, hi):
    return np.array([rng.integers(lo, hi) for _ in range(3)], dtype=np.uint8)


def render_scene(w: int, h: int, rng: np.random.Generator):
    """Render one road-like BGR frame and return (img, error_px, confidence)."""
    horizon = int(h * rng.uniform(0.35, 0.5))

    asphalt = int(rng.integers(35, 85))
    img = np.full((h, w, 3), asphalt, dtype=np.uint8)

    sky = _rand_color(rng, 60, 200)
    img[:horizon, :] = sky
    img[:horizon, :] = cv2.GaussianBlur(img[:horizon, :], (0, 0), 7)

    base_off = float(rng.uniform(-0.22, 0.22) * w)
    base_x = w / 2.0 + base_off
    curv = float(rng.uniform(-1.0, 1.0)) * w * 0.35
    lane_w_bottom = float(rng.uniform(0.32, 0.52) * w)

    line_color = (255, 255, 255) if rng.random() < 0.7 else (60, 220, 240)
    thick = int(rng.integers(2, 6))
    dashed = rng.random() < 0.45
    drop_left = rng.random() < 0.10
    drop_right = rng.random() < 0.10
    if drop_left and drop_right:
        drop_right = False

    ys = np.arange(horizon, h, 2)
    left_pts, right_pts = [], []
    for y in ys:
        t = (y - horizon) / max(1, (h - horizon))
        cx = base_x + curv * (1.0 - t) ** 2
        hw = lane_w_bottom * (0.18 + 0.82 * t)
        left_pts.append((int(cx - hw), int(y)))
        right_pts.append((int(cx + hw), int(y)))

    def draw_line(pts):
        for i in range(len(pts) - 1):
            if dashed and ((pts[i][1] // 18) % 2 == 0):
                continue
            cv2.line(img, pts[i], pts[i + 1], line_color, thick, cv2.LINE_AA)

    if not drop_left:
        draw_line(left_pts)
    if not drop_right:
        draw_line(right_pts)

    if rng.random() < 0.5:
        sx = int(rng.integers(0, w))
        poly = np.array([[sx, horizon], [sx + rng.integers(40, 240), horizon],
                         [sx + rng.integers(-80, 320), h], [sx - rng.integers(40, 200), h]])
        shadow = img.copy()
        cv2.fillPoly(shadow, [poly], (0, 0, 0))
        img = cv2.addWeighted(img, 1.0 - rng.uniform(0.15, 0.45), shadow,
                              rng.uniform(0.15, 0.45), 0)

    gain = float(rng.uniform(0.6, 1.35))
    bias = float(rng.uniform(-25, 25))
    img = np.clip(img.astype(np.float32) * gain + bias, 0, 255).astype(np.uint8)

    img = cv2.add(img, rng.integers(0, int(rng.integers(4, 22)) + 1,
                                    size=img.shape, dtype=np.uint8))
    if rng.random() < 0.4:
        k = int(rng.choice([3, 5]))
        img = cv2.GaussianBlur(img, (k, k), 0)

    error_px = base_x - w / 2.0
    confidence = 1.0 if not (drop_left or drop_right) else float(rng.uniform(0.55, 0.8))
    return img, float(error_px), confidence


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=3000, help="number of frames")
    ap.add_argument("--out", default=str(DEFAULT_OUT))
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    out = Path(args.out)
    frames_dir = out / "frames"
    frames_dir.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(args.seed)

    rows = []
    for i in range(args.n):
        img, err, conf = render_scene(args.width, args.height, rng)
        name = f"frames/{i:06d}.jpg"
        cv2.imwrite(str(out / name), img, [cv2.IMWRITE_JPEG_QUALITY, 90])
        rows.append((name, f"{err:.3f}", f"{conf:.3f}", "synthetic"))
        if (i + 1) % 500 == 0:
            print(f"  {i + 1}/{args.n}")

    with open(out / "labels.csv", "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["frame", "error_px", "confidence", "source"])
        wr.writerows(rows)

    (out / "tub.json").write_text(json.dumps(
        {"width": args.width, "height": args.height, "count": args.n,
         "label": "error_px", "convention": "lane_centre - frame_centre (px)"},
        indent=2))

    errs = np.array([float(r[1]) for r in rows])
    print(f"[GEN] {args.n} frames -> {out}")
    print(f"[GEN] error_px range [{errs.min():.0f}, {errs.max():.0f}]  "
          f"std {errs.std():.0f}")


if __name__ == "__main__":
    main()
