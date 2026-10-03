#!/usr/bin/env python3
"""Train the end-to-end steering network (behavioral cloning).

Reads one or more tubs (frames/ + labels.csv) produced by
tools/gen_synth_driving.py or tools/record_driving.py, conditions the labels
(normalization, optional temporal smoothing, optional |error| balancing),
applies heavy augmentation (see vision/drive_data.py) and trains one of the
networks defined in vision/drive_net.py:

    --arch pilotnet    NVIDIA PilotNet (default, ~0.25 M params)
    --arch mobilenet   MobileNetV2-style, depthwise separable (~0.17 M params)

The regression target is `error_px / norm_px`. `--throttle` adds a third head
trained on the tubs' optional `throttle` column. Everything needed to reproduce
the preprocessing (input size, ROI, norm, arch) is written next to the weights
(drive_net.json), and inference/export read it from there.

GPU: picked automatically when available, with mixed precision (AMP). The same
code runs on the PC, on the Pi (slowly) and on Colab/Kaggle -- the notebook
notebooks/train_colab.ipynb calls `train()` below.

Outputs (in the directory of --out):
  drive_net.pt          {"model": best state_dict, "meta": {...}}
  drive_net.json        meta sidecar
  drive_net_last.pt     last-epoch weights (to resume or compare)
  drive_history.json    per-epoch train loss, val MSE / RMSE / MAE, lr
  drive_train.png       loss + val-RMSE curves

Usage:
    python TMR2026/tools/train_drive.py --data datasets/drive_synth --epochs 30
    python TMR2026/tools/train_drive.py --data datasets/a,datasets/b \
        --val datasets/val --epochs 60 --batch 128 --arch mobilenet
    python TMR2026/tools/train_drive.py --data datasets/real --smooth 5 \
        --balance --shift-gain 0.9 --throttle
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import torch

HERE = Path(__file__).resolve().parent
TMR_ROOT = HERE.parent
sys.path.insert(0, str(TMR_ROOT))

from vision.drive_net import (ARCHS, DEFAULT_ARCH, DEFAULT_NORM_PX, IMG_H, IMG_W,
                              ROI_BOTTOM, ROI_FRAC, build_model, count_params)
from vision.drive_data import (AugConfig, balance_weights, make_dataset,
                               read_tubs, split_samples, worker_init_fn)


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--data", required=True, help="comma-separated train tub dirs")
    ap.add_argument("--val", default="", help="comma-separated val tubs (else --val-split)")
    ap.add_argument("--val-split", type=float, default=0.15)
    ap.add_argument("--arch", choices=ARCHS, default=DEFAULT_ARCH)
    ap.add_argument("--width", type=float, default=1.0, help="mobilenet width multiplier")
    ap.add_argument("--epochs", type=int, default=30)
    ap.add_argument("--batch", type=int, default=64)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--weight-decay", type=float, default=1e-4)
    ap.add_argument("--loss", choices=("huber", "mse"), default="huber",
                    help="steering loss; huber is robust to teacher-label outliers")
    ap.add_argument("--norm-px", type=float, default=DEFAULT_NORM_PX)
    ap.add_argument("--img-w", type=int, default=IMG_W)
    ap.add_argument("--img-h", type=int, default=IMG_H)
    ap.add_argument("--roi-frac", type=float, default=ROI_FRAC,
                    help="top of the ROI as a fraction of frame height")
    ap.add_argument("--roi-bottom", type=float, default=ROI_BOTTOM,
                    help="bottom of the ROI (cuts the bumper if visible)")
    ap.add_argument("--smooth", type=int, default=0,
                    help="temporal moving-average window on recorded labels (0=off)")
    ap.add_argument("--balance", action="store_true",
                    help="oversample large |error| (curves) with inverse-frequency weights")
    ap.add_argument("--throttle", action="store_true",
                    help="train a throttle head on the tubs' `throttle` column")
    ap.add_argument("--throttle-max", type=float, default=100.0,
                    help="value of `throttle` that maps to 1.0")
    ap.add_argument("--shift-gain", type=float, default=1.0,
                    help="label px per image px of lateral-shift augmentation")
    ap.add_argument("--workers", type=int, default=0)
    ap.add_argument("--patience", type=int, default=10, help="early-stopping patience")
    ap.add_argument("--min-delta", type=float, default=0.5,
                    help="val RMSE (px) improvement that resets patience")
    ap.add_argument("--device", default="auto")
    ap.add_argument("--no-amp", action="store_true", help="disable mixed precision on CUDA")
    ap.add_argument("--out", default=str(TMR_ROOT / "weights" / "drive_net.pt"))
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--no-aug", action="store_true")
    return ap


def _loss_fn(out, y, kind: str, use_throttle: bool):
    import torch.nn.functional as F
    if kind == "huber":
        steer = F.smooth_l1_loss(out[:, 0], y[:, 0], beta=0.1)
    else:
        steer = F.mse_loss(out[:, 0], y[:, 0])
    loss = steer + 0.2 * F.binary_cross_entropy_with_logits(out[:, 1], y[:, 1])
    if use_throttle:
        m = ~torch.isnan(y[:, 2])
        if m.any():
            loss = loss + 0.5 * F.binary_cross_entropy_with_logits(out[m, 2], y[m, 2])
    return loss


def evaluate(model, loader, device, norm_px: float) -> dict:
    """Steering metrics in label pixels over a loader."""
    model.eval()
    se = ae = 0.0
    n = 0
    with torch.no_grad():
        for xb, yb in loader:
            xb, yb = xb.to(device), yb.to(device)
            d = (model(xb)[:, 0].float() - yb[:, 0]) * norm_px
            se += float((d ** 2).sum())
            ae += float(d.abs().sum())
            n += xb.size(0)
    mse = se / max(1, n)
    return {"mse_px2": mse, "rmse_px": math.sqrt(mse), "mae_px": ae / max(1, n)}


def plot_history(hist: dict, path: Path) -> bool:
    # A standalone Figure, not pyplot: matplotlib.use("Agg") here would switch
    # the backend of the whole process, and inside a notebook every plot after
    # train() would then silently stop rendering.
    try:
        from matplotlib.figure import Figure
    except Exception as e:
        print(f"[TRAIN] (no curve: {e})")
        return False
    ep = np.arange(1, len(hist["train_loss"]) + 1)
    fig = Figure(figsize=(11, 4))
    a1, a2 = fig.subplots(1, 2)
    a1.plot(ep, hist["train_loss"], "b-", label="train loss")
    a1.set_xlabel("epoch"); a1.set_ylabel("loss"); a1.set_yscale("log")
    a1.grid(alpha=.3); a1.legend()
    a2.plot(ep, hist["val_rmse_px"], "r-", label="val RMSE (px)")
    a2.plot(ep, hist["val_mae_px"], "r--", alpha=.6, label="val MAE (px)")
    if hist.get("best_epoch"):
        a2.axvline(hist["best_epoch"], color="k", ls=":", label="best")
    a2.set_xlabel("epoch"); a2.set_ylabel("px"); a2.grid(alpha=.3); a2.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    return True


def train(args) -> dict:
    """Train with parsed `args` (see build_parser). Returns the history dict."""
    device = ("cuda" if torch.cuda.is_available() else "cpu") \
        if args.device == "auto" else args.device
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)

    roots = (TMR_ROOT, TMR_ROOT.parent)
    samples = read_tubs(args.data, roots, args.smooth, args.throttle_max)
    if not samples:
        raise SystemExit("ERROR: no training samples found")
    if args.val:
        train_s = samples
        val_s = read_tubs(args.val, roots, args.smooth, args.throttle_max)
    else:
        train_s, val_s = split_samples(samples, args.val_split, args.seed)
    if not val_s:
        raise SystemExit("ERROR: empty validation set (use --val or --val-split > 0)")

    use_thr = bool(args.throttle)
    if use_thr and not any(s.throttle is not None for s in train_s):
        print("[TRAIN] WARN: --throttle but no tub has a `throttle` column; ignoring")
        use_thr = False
    n_out = 3 if use_thr else 2

    aug = None
    if not args.no_aug:
        aug = AugConfig(shift_gain=args.shift_gain)

    tr_ds = make_dataset(train_s, args.norm_px, args.img_w, args.img_h,
                         args.roi_frac, args.roi_bottom, aug, args.seed)
    va_ds = make_dataset(val_s, args.norm_px, args.img_w, args.img_h,
                         args.roi_frac, args.roi_bottom, None, args.seed + 1)

    pin = device.startswith("cuda")
    if args.balance:
        w = balance_weights(train_s, args.norm_px)
        sampler = torch.utils.data.WeightedRandomSampler(
            torch.as_tensor(w, dtype=torch.double), len(train_s), replacement=True)
        tr_dl = torch.utils.data.DataLoader(
            tr_ds, batch_size=args.batch, sampler=sampler, num_workers=args.workers,
            worker_init_fn=worker_init_fn, pin_memory=pin,
            persistent_workers=args.workers > 0)
    else:
        tr_dl = torch.utils.data.DataLoader(
            tr_ds, batch_size=args.batch, shuffle=True, num_workers=args.workers,
            worker_init_fn=worker_init_fn, pin_memory=pin,
            persistent_workers=args.workers > 0)
    va_dl = torch.utils.data.DataLoader(va_ds, batch_size=args.batch, shuffle=False,
                                        num_workers=args.workers, pin_memory=pin)

    model = build_model(args.arch, n_out, args.width, args.img_w, args.img_h).to(device)
    print(f"[TRAIN] device={device}  arch={args.arch}  params={count_params(model):,}  "
          f"train={len(train_s)}  val={len(val_s)}")
    print(f"[TRAIN] img={args.img_w}x{args.img_h}  roi=[{args.roi_frac},{args.roi_bottom}]  "
          f"norm_px={args.norm_px}  aug={aug is not None}  smooth={args.smooth}  "
          f"balance={args.balance}  throttle={use_thr}  loss={args.loss}")

    opt = torch.optim.AdamW(model.parameters(), lr=args.lr,
                            weight_decay=args.weight_decay)
    sched = torch.optim.lr_scheduler.ReduceLROnPlateau(opt, factor=0.5, patience=4)
    use_amp = device.startswith("cuda") and not args.no_amp
    scaler = torch.amp.GradScaler("cuda", enabled=use_amp)

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    meta = {"arch": args.arch, "width": args.width, "n_out": n_out,
            "img_w": args.img_w, "img_h": args.img_h,
            "roi_frac": args.roi_frac, "roi_bottom": args.roi_bottom,
            "norm_px": args.norm_px, "throttle_max": args.throttle_max,
            "smooth": args.smooth, "shift_gain": args.shift_gain,
            "train_n": len(train_s), "val_n": len(val_s)}

    best_rmse = float("inf")
    best_state = None
    bad = 0
    hist = {"train_loss": [], "val_mse_px2": [], "val_rmse_px": [],
            "val_mae_px": [], "lr": [], "best_epoch": 0}

    for ep in range(1, args.epochs + 1):
        model.train()
        t0 = time.time()
        tot = 0.0
        seen = 0
        for xb, yb in tr_dl:
            xb = xb.to(device, non_blocking=True)
            yb = yb.to(device, non_blocking=True)
            opt.zero_grad(set_to_none=True)
            with torch.autocast("cuda", enabled=use_amp):
                out = model(xb)
            loss = _loss_fn(out.float(), yb, args.loss, use_thr)
            scaler.scale(loss).backward()
            scaler.unscale_(opt)
            torch.nn.utils.clip_grad_norm_(model.parameters(), 5.0)
            scaler.step(opt)
            scaler.update()
            tot += loss.item() * xb.size(0)
            seen += xb.size(0)
        tr_loss = tot / max(1, seen)

        m = evaluate(model, va_dl, device, args.norm_px)
        sched.step(m["rmse_px"])
        hist["train_loss"].append(tr_loss)
        hist["val_mse_px2"].append(m["mse_px2"])
        hist["val_rmse_px"].append(m["rmse_px"])
        hist["val_mae_px"].append(m["mae_px"])
        hist["lr"].append(opt.param_groups[0]["lr"])
        print(f"  ep {ep:3d}/{args.epochs}  train_loss {tr_loss:.4f}  "
              f"val_RMSE {m['rmse_px']:6.1f}px  MAE {m['mae_px']:5.1f}px  "
              f"lr {hist['lr'][-1]:.1e}  ({time.time() - t0:.1f}s)", flush=True)

        if m["rmse_px"] < best_rmse - args.min_delta:
            best_rmse = m["rmse_px"]
            best_state = {k: v.detach().cpu().clone() for k, v in model.state_dict().items()}
            hist["best_epoch"] = ep
            bad = 0
            # Save on every improvement: a Colab session can be reclaimed at any
            # moment, and the best weights must already be on disk when it is.
            torch.save({"model": best_state, "meta": {**meta, "val_rmse_px": best_rmse,
                                                      "epoch": ep}}, out_path)
        else:
            bad += 1
            if bad >= args.patience:
                print(f"[TRAIN] early stop at epoch {ep} "
                      f"(best val_RMSE {best_rmse:.1f}px @ epoch {hist['best_epoch']})")
                break

    meta.update({"val_rmse_px": best_rmse, "epoch": hist["best_epoch"]})
    torch.save({"model": best_state or model.state_dict(), "meta": meta}, out_path)
    torch.save({"model": model.state_dict(), "meta": meta},
               out_path.with_name(out_path.stem + "_last.pt"))
    out_path.with_suffix(".json").write_text(json.dumps(meta, indent=2))
    out_path.with_name("drive_history.json").write_text(json.dumps(hist, indent=2))
    print(f"[TRAIN] saved {out_path}  (best val_RMSE {best_rmse:.1f}px)")
    if plot_history(hist, out_path.with_name("drive_train.png")):
        print(f"[TRAIN] curves -> {out_path.with_name('drive_train.png')}")
    return hist


def main() -> None:
    train(build_parser().parse_args())


if __name__ == "__main__":
    main()
