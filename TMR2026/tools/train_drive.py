#!/usr/bin/env python3
"""Train the end-to-end steering network (behavioral cloning).

Reads one or more tubs (frames/ + labels.csv) produced by
tools/gen_synth_driving.py or tools/record_driving.py, applies heavy
augmentation so the policy generalizes to "any basic road-like track", and
trains the PilotNet defined in vision/drive_net.py.

The label is `error_px` (lane_centre - frame_centre); it is normalized by
--norm-px so the regression target sits in ~[-1, 1]. The norm and input size
are written next to the weights (drive_net.json) so inference/export reproduce
the exact preprocessing.

Augmentation (label-aware where geometric):
  - horizontal flip            -> error_px *= -1
  - horizontal shift           -> error_px += dx
  - brightness / contrast / gamma, HSV jitter, shadows, blur, noise (label-free)

Outputs:
  weights/drive_net.pt    {"model": state_dict, "meta": {...}}
  weights/drive_net.json  {"img_w","img_h","roi_frac","norm_px",...}
  weights/drive_train.png loss + val-RMSE curves

Usage:
    python TMR2026/tools/train_drive.py --data datasets/drive_synth --epochs 30
    python TMR2026/tools/train_drive.py --data datasets/a,datasets/b \
        --val datasets/val --epochs 60 --batch 64
    python TMR2026/tools/train_drive.py --data datasets/drive_synth --device cuda
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
TMR_ROOT = HERE.parent
sys.path.insert(0, str(TMR_ROOT))

from vision.drive_net import build_model, preprocess, IMG_W, IMG_H, ROI_FRAC, DEFAULT_NORM_PX


def read_tubs(spec: str):
    """spec = comma-separated tub dirs. Returns list of (abs_image_path, err, conf)."""
    samples = []
    for raw in spec.split(","):
        raw = raw.strip()
        if not raw:
            continue
        tub = Path(raw)
        if not tub.is_absolute():
            tub = (TMR_ROOT / tub) if (TMR_ROOT / tub).exists() else (Path.cwd() / tub)
        csv_path = tub / "labels.csv"
        if not csv_path.exists():
            print(f"[WARN] no labels.csv in {tub}")
            continue
        with open(csv_path) as f:
            for r in csv.DictReader(f):
                samples.append((str(tub / r["frame"]),
                                float(r["error_px"]), float(r.get("confidence", 1.0))))
    return samples


def augment(img, error_px, norm_px, rng):
    """Photometric + label-aware geometric augmentation on a BGR frame."""
    h, w = img.shape[:2]

    if rng.random() < 0.5:
        img = img[:, ::-1].copy()
        error_px = -error_px

    if rng.random() < 0.6:
        dx = int(rng.uniform(-0.12, 0.12) * w)
        M = np.float32([[1, 0, dx], [0, 1, 0]])
        img = cv2.warpAffine(img, M, (w, h), borderMode=cv2.BORDER_REFLECT)
        error_px += dx

    if rng.random() < 0.85:
        gain = float(rng.uniform(0.55, 1.5))
        bias = float(rng.uniform(-35, 35))
        img = np.clip(img.astype(np.float32) * gain + bias, 0, 255).astype(np.uint8)

    if rng.random() < 0.4:
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV).astype(np.int16)
        hsv[..., 0] = (hsv[..., 0] + int(rng.uniform(-12, 12))) % 180
        hsv[..., 1] = np.clip(hsv[..., 1] * rng.uniform(0.6, 1.4), 0, 255)
        img = cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)

    if rng.random() < 0.45:
        x0 = int(rng.uniform(0, w))
        poly = np.array([[x0, 0], [x0 + rng.integers(40, 260), 0],
                         [x0 + rng.integers(-100, 360), h], [x0 - rng.integers(40, 220), h]])
        sh = img.copy()
        cv2.fillPoly(sh, [poly], (0, 0, 0))
        a = float(rng.uniform(0.15, 0.5))
        img = cv2.addWeighted(img, 1 - a, sh, a, 0)

    if rng.random() < 0.3:
        k = int(rng.choice([3, 5]))
        img = cv2.GaussianBlur(img, (k, k), 0)

    if rng.random() < 0.4:
        img = cv2.add(img, rng.integers(0, 18, size=img.shape, dtype=np.uint8))

    return img, error_px


def make_dataset_cls():
    import torch

    class TubDataset(torch.utils.data.Dataset):
        def __init__(self, samples, norm_px, img_w, img_h, roi_frac, train, seed=0):
            self.samples = samples
            self.norm_px = norm_px
            self.img_w, self.img_h, self.roi_frac = img_w, img_h, roi_frac
            self.train = train
            self.rng = np.random.default_rng(seed)

        def __len__(self):
            return len(self.samples)

        def __getitem__(self, i):
            path, err, conf = self.samples[i]
            img = cv2.imread(path)
            if img is None:
                img = np.zeros((self.img_h * 4, self.img_w * 2, 3), np.uint8)
            if self.train:
                img, err = augment(img, err, self.norm_px, self.rng)
            x = preprocess(img, self.img_w, self.img_h, self.roi_frac)
            y = np.array([err / self.norm_px, conf], dtype=np.float32)
            return torch.from_numpy(x), torch.from_numpy(y)

    return TubDataset


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True, help="comma-separated train tub dirs")
    ap.add_argument("--val", default="", help="comma-separated val tubs (else --val-split)")
    ap.add_argument("--val-split", type=float, default=0.15)
    ap.add_argument("--epochs", type=int, default=30)
    ap.add_argument("--batch", type=int, default=64)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--norm-px", type=float, default=DEFAULT_NORM_PX)
    ap.add_argument("--img-w", type=int, default=IMG_W)
    ap.add_argument("--img-h", type=int, default=IMG_H)
    ap.add_argument("--workers", type=int, default=0)
    ap.add_argument("--patience", type=int, default=10)
    ap.add_argument("--device", default="auto")
    ap.add_argument("--out", default=str(TMR_ROOT / "weights" / "drive_net.pt"))
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--no-aug", action="store_true")
    args = ap.parse_args()

    import torch
    import torch.nn.functional as F

    device = ("cuda" if torch.cuda.is_available() else "cpu") \
        if args.device == "auto" else args.device
    torch.manual_seed(args.seed)

    train_samples = read_tubs(args.data)
    if not train_samples:
        print("ERROR: no training samples found")
        sys.exit(1)

    if args.val:
        val_samples = read_tubs(args.val)
    else:
        rng = np.random.default_rng(args.seed)
        idx = rng.permutation(len(train_samples))
        n_val = int(len(idx) * args.val_split)
        val_idx, tr_idx = set(idx[:n_val].tolist()), idx[n_val:]
        val_samples = [train_samples[i] for i in sorted(val_idx)]
        train_samples = [train_samples[i] for i in tr_idx]

    print(f"[TRAIN] device={device}  train={len(train_samples)}  val={len(val_samples)}")
    print(f"[TRAIN] img={args.img_w}x{args.img_h}  norm_px={args.norm_px}  aug={not args.no_aug}")

    TubDataset = make_dataset_cls()
    tr_ds = TubDataset(train_samples, args.norm_px, args.img_w, args.img_h,
                       ROI_FRAC, train=not args.no_aug, seed=args.seed)
    va_ds = TubDataset(val_samples, args.norm_px, args.img_w, args.img_h,
                       ROI_FRAC, train=False, seed=args.seed + 1)
    tr_dl = torch.utils.data.DataLoader(tr_ds, batch_size=args.batch, shuffle=True,
                                        num_workers=args.workers, drop_last=False)
    va_dl = torch.utils.data.DataLoader(va_ds, batch_size=args.batch, shuffle=False,
                                        num_workers=args.workers)

    model = build_model().to(device)
    opt = torch.optim.Adam(model.parameters(), lr=args.lr, weight_decay=1e-4)
    sched = torch.optim.lr_scheduler.ReduceLROnPlateau(opt, factor=0.5, patience=4)

    best_rmse = float("inf")
    best_state = None
    bad = 0
    hist = {"train": [], "val_rmse_px": []}

    for ep in range(1, args.epochs + 1):
        model.train()
        t0 = time.time()
        tr_loss = 0.0
        for xb, yb in tr_dl:
            xb, yb = xb.to(device), yb.to(device)
            opt.zero_grad()
            out = model(xb)
            loss = F.mse_loss(out[:, 0], yb[:, 0]) + 0.2 * F.binary_cross_entropy_with_logits(out[:, 1], yb[:, 1])
            loss.backward()
            opt.step()
            tr_loss += loss.item() * xb.size(0)
        tr_loss /= max(1, len(tr_ds))

        model.eval()
        sq = 0.0
        n = 0
        with torch.no_grad():
            for xb, yb in va_dl:
                xb, yb = xb.to(device), yb.to(device)
                pred_err = model(xb)[:, 0] * args.norm_px
                tgt_err = yb[:, 0] * args.norm_px
                sq += float(((pred_err - tgt_err) ** 2).sum().item())
                n += xb.size(0)
        val_rmse = (sq / max(1, n)) ** 0.5
        sched.step(val_rmse)
        hist["train"].append(tr_loss)
        hist["val_rmse_px"].append(val_rmse)
        print(f"  ep {ep:3d}/{args.epochs}  train_loss {tr_loss:.4f}  "
              f"val_RMSE {val_rmse:6.1f}px  ({time.time()-t0:.1f}s)")

        if val_rmse < best_rmse - 0.5:
            best_rmse = val_rmse
            best_state = {k: v.cpu().clone() for k, v in model.state_dict().items()}
            bad = 0
        else:
            bad += 1
            if bad >= args.patience:
                print(f"[TRAIN] early stop at epoch {ep} (best val_RMSE {best_rmse:.1f}px)")
                break

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    meta = {"img_w": args.img_w, "img_h": args.img_h, "roi_frac": ROI_FRAC,
            "norm_px": args.norm_px, "val_rmse_px": best_rmse,
            "train_n": len(train_samples), "val_n": len(val_samples)}
    torch.save({"model": best_state or model.state_dict(), "meta": meta}, out_path)
    out_path.with_suffix(".json").write_text(json.dumps(meta, indent=2))
    print(f"[TRAIN] saved {out_path}  (best val_RMSE {best_rmse:.1f}px)")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax1 = plt.subplots(figsize=(7, 4))
        ax1.plot(hist["train"], "b-", label="train loss")
        ax1.set_xlabel("epoch"); ax1.set_ylabel("train loss", color="b")
        ax2 = ax1.twinx()
        ax2.plot(hist["val_rmse_px"], "r-", label="val RMSE px")
        ax2.set_ylabel("val RMSE (px)", color="r")
        fig.tight_layout()
        fig.savefig(out_path.with_name("drive_train.png"), dpi=110)
        print(f"[TRAIN] curve -> {out_path.with_name('drive_train.png')}")
    except Exception as e:
        print(f"[TRAIN] (no curve: {e})")


if __name__ == "__main__":
    main()
