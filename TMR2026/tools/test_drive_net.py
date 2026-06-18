#!/usr/bin/env python3
"""Evaluate / preview the trained steering network. NEVER touches GPIO.

Safe to run on a dev PC and on the Pi while the systemd service is active (it
imports no hardware). Three input sources:

    --tub    datasets/drive_synth_val   evaluate vs labels (RMSE) + save montage
    --images path/to/dir_or_glob        run on loose images, save montage
    --camera                            live preview (Pi only; needs CameraStream)

Usage:
    python TMR2026/tools/test_drive_net.py --tub datasets/drive_synth_val
    python TMR2026/tools/test_drive_net.py --images datasets/drive_synth/frames --limit 16
    python TMR2026/tools/test_drive_net.py --camera
"""
from __future__ import annotations

import argparse
import csv
import glob
import sys
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
TMR_ROOT = HERE.parent
sys.path.insert(0, str(TMR_ROOT))

from vision.drive_net import DriveNet, predicted_steering_x

DEFAULT_WEIGHTS = TMR_ROOT / "weights" / "drive_net.pt"


def _annot(img, pred_err, conf, gt_err=None):
    vis = img.copy()
    h, w = vis.shape[:2]
    cx = w // 2
    cv2.line(vis, (cx, h), (cx, h - 60), (160, 160, 160), 1)
    if gt_err is not None:
        gx = predicted_steering_x(w, gt_err)
        cv2.line(vis, (gx, h), (gx, h - 70), (0, 220, 255), 2)
    px = predicted_steering_x(w, pred_err)
    cv2.line(vis, (px, h), (px, h - 90), (0, 255, 0), 3)
    txt = f"pred {pred_err:+.0f}px {conf:.0%}"
    if gt_err is not None:
        txt += f"  gt {gt_err:+.0f}"
    cv2.putText(vis, txt, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    return vis


def _montage(tiles, cols=4):
    if not tiles:
        return None
    tiles = [cv2.resize(t, (320, 240)) for t in tiles]
    while len(tiles) % cols:
        tiles.append(np.zeros_like(tiles[0]))
    rows = [np.hstack(tiles[i:i + cols]) for i in range(0, len(tiles), cols)]
    return np.vstack(rows)


def eval_tub(net, tub, limit, out_png):
    rows = list(csv.DictReader(open(Path(tub) / "labels.csv")))
    if limit:
        rows = rows[:limit] if limit > 0 else rows
    sq = 0.0
    n = 0
    tiles = []
    for r in rows:
        img = cv2.imread(str(Path(tub) / r["frame"]))
        if img is None:
            continue
        gt = float(r["error_px"])
        res = net.process(img)
        sq += (res.error_px - gt) ** 2
        n += 1
        if len(tiles) < 16:
            tiles.append(_annot(img, res.error_px, res.confidence, gt))
    rmse = (sq / max(1, n)) ** 0.5
    print(f"[EVAL] tub={tub}  n={n}  RMSE={rmse:.1f}px")
    mont = _montage(tiles)
    if mont is not None:
        cv2.imwrite(str(out_png), mont)
        print(f"[EVAL] montage -> {out_png}")
    return rmse


def eval_images(net, pattern, limit, out_png):
    p = Path(pattern)
    files = sorted(glob.glob(str(p / "*.jpg")) + glob.glob(str(p / "*.png"))) \
        if p.is_dir() else sorted(glob.glob(pattern))
    if limit > 0:
        files = files[:limit]
    tiles = []
    for fp in files:
        img = cv2.imread(fp)
        if img is None:
            continue
        res = net.process(img)
        tiles.append(_annot(img, res.error_px, res.confidence))
    print(f"[EVAL] images={len(tiles)}")
    mont = _montage(tiles)
    if mont is not None:
        cv2.imwrite(str(out_png), mont)
        print(f"[EVAL] montage -> {out_png}")


def live_camera(net):
    from vision.camera_stream import CameraStream
    cam = CameraStream()
    cam.start()
    try:
        while True:
            f = cam.get_frame()
            if f is None:
                continue
            res = net.process(f)
            cv2.imshow("DriveNet preview", _annot(f, res.error_px, res.confidence))
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break
    finally:
        cam.stop()
        cv2.destroyAllWindows()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", default=str(DEFAULT_WEIGHTS))
    ap.add_argument("--tub", default="")
    ap.add_argument("--images", default="")
    ap.add_argument("--camera", action="store_true")
    ap.add_argument("--limit", type=int, default=16)
    ap.add_argument("--device", default=None)
    ap.add_argument("--out", default=str(TMR_ROOT / "weights" / "drive_eval.png"))
    args = ap.parse_args()

    if not Path(args.weights).exists():
        print(f"ERROR: weights not found: {args.weights}\n"
              f"Train first: python TMR2026/tools/train_drive.py --data datasets/drive_synth")
        sys.exit(1)

    net = DriveNet(args.weights, device=args.device, debug=False)
    print(f"[EVAL] loaded {args.weights}  img={net.img_w}x{net.img_h}  norm_px={net.norm_px}")

    if args.tub:
        eval_tub(net, args.tub, args.limit, Path(args.out))
    elif args.images:
        eval_images(net, args.images, args.limit, Path(args.out))
    elif args.camera:
        live_camera(net)
    else:
        print("Nothing to do: pass --tub, --images or --camera")


if __name__ == "__main__":
    main()
