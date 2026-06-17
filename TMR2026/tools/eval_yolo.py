#!/usr/bin/env python3
"""Evaluate weights/tmr_signs.pt on traffic_lights/<split>/ at several
confidence thresholds to choose YOLO_CONF.

Reports P / R / F1 / TP / FP / FN per class and total for each threshold.
A prediction counts as a TP if IoU >= --iou and the class matches.

Usage (from any directory):
    python TMR2026/tools/eval_yolo.py
    python TMR2026/tools/eval_yolo.py --split test --imgsz 320
    python TMR2026/tools/eval_yolo.py --confs 0.20,0.30,0.40,0.50,0.60

Optimization: a single inference pass per image at the lowest --probe-conf
(0.05), with detections filtered post-hoc per threshold. The internal NMS may
vary slightly, but it is enough to choose the threshold.
"""
from __future__ import annotations

import argparse
import sys
from collections import defaultdict
from pathlib import Path

HERE = Path(__file__).resolve().parent
TMR_ROOT = HERE.parent
REPO_ROOT = TMR_ROOT.parent

DEFAULT_WEIGHTS = TMR_ROOT / "weights" / "tmr_signs.pt"
DEFAULT_DATASET = REPO_ROOT / "traffic_lights"

CLASS_NAMES = ["green", "left", "red", "right", "stop", "straight", "yellow"]


def iou(b1, b2) -> float:
    ax1, ay1, ax2, ay2 = b1
    bx1, by1, bx2, by2 = b2
    iw = max(0.0, min(ax2, bx2) - max(ax1, bx1))
    ih = max(0.0, min(ay2, by2) - max(ay1, by1))
    inter = iw * ih
    a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    union = a + b - inter
    return inter / union if union > 0 else 0.0


def load_gt(label_path: Path, img_w: int, img_h: int):
    boxes = []
    if not label_path.exists():
        return boxes
    for line in label_path.read_text().splitlines():
        parts = line.split()
        if len(parts) < 5:
            continue
        cls = int(parts[0])
        cx, cy, w, h = map(float, parts[1:5])
        x1 = (cx - w / 2) * img_w
        y1 = (cy - h / 2) * img_h
        x2 = (cx + w / 2) * img_w
        y2 = (cy + h / 2) * img_h
        boxes.append((cls, x1, y1, x2, y2))
    return boxes


def match(preds, gts, iou_thr: float = 0.5):
    """Greedy match: predicciones ordenadas por conf desc compiten por GTs."""
    used = set()
    tp_per_cls: dict[int, int] = defaultdict(int)
    fp_per_cls: dict[int, int] = defaultdict(int)
    for _, pc, px1, py1, px2, py2 in preds:
        best_iou = 0.0
        best_j = -1
        for j, g in enumerate(gts):
            if j in used or g[0] != pc:
                continue
            io = iou((px1, py1, px2, py2), g[1:])
            if io > best_iou:
                best_iou = io
                best_j = j
        if best_iou >= iou_thr:
            tp_per_cls[pc] += 1
            used.add(best_j)
        else:
            fp_per_cls[pc] += 1
    fn_per_cls: dict[int, int] = defaultdict(int)
    for j, g in enumerate(gts):
        if j not in used:
            fn_per_cls[g[0]] += 1
    return tp_per_cls, fp_per_cls, fn_per_cls


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", default=str(DEFAULT_WEIGHTS))
    ap.add_argument("--dataset", default=str(DEFAULT_DATASET))
    ap.add_argument("--split", default="valid", choices=["train", "valid", "test"])
    ap.add_argument("--imgsz", type=int, default=320)
    ap.add_argument("--iou",   type=float, default=0.5)
    ap.add_argument("--confs", default="0.25,0.35,0.45,0.55")
    ap.add_argument("--probe-conf", type=float, default=0.05)
    ap.add_argument("--limit", type=int, default=0,
                    help="Limit the number of images (0 = all)")
    args = ap.parse_args()

    confs = sorted(float(c) for c in args.confs.split(","))

    img_dir = Path(args.dataset) / args.split / "images"
    lbl_dir = Path(args.dataset) / args.split / "labels"
    if not img_dir.exists():
        print(f"ERROR: {img_dir} does not exist")
        sys.exit(1)

    images = sorted(img_dir.glob("*.jpg")) + sorted(img_dir.glob("*.png"))
    if args.limit > 0:
        images = images[: args.limit]
    if not images:
        print(f"ERROR: no images in {img_dir}")
        sys.exit(1)

    print(f"[EVAL] model:   {args.weights}")
    print(f"[EVAL] split:   {args.split}  ({len(images)} images)")
    print(f"[EVAL] imgsz:   {args.imgsz}   min IoU TP: {args.iou}")
    print(f"[EVAL] confs:   {confs}")

    from ultralytics import YOLO
    import cv2

    print("[EVAL] cargando modelo ...")
    model = YOLO(args.weights)

    print("[EVAL] inferencia ...")
    cache = []
    for i, img_path in enumerate(images):
        img = cv2.imread(str(img_path))
        if img is None:
            continue
        h, w = img.shape[:2]
        res = model(img, imgsz=args.imgsz, conf=args.probe_conf, verbose=False)[0]
        dets = []
        for box in res.boxes:
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            x1, y1, x2, y2 = (float(v) for v in box.xyxy[0])
            dets.append((conf, cls, x1, y1, x2, y2))
        dets.sort(key=lambda d: -d[0])
        gts = load_gt(lbl_dir / (img_path.stem + ".txt"), w, h)
        cache.append((img_path.name, w, h, dets, gts))
        if (i + 1) % 100 == 0:
            print(f"  {i + 1}/{len(images)}")

    print()
    header = (f"{'conf':>5} | {'class':<10} {'P':>7} {'R':>7} {'F1':>7} "
              f"{'TP':>5} {'FP':>5} {'FN':>5}")
    print(header)
    print("-" * len(header))

    for conf in confs:
        tp_t: dict[int, int] = defaultdict(int)
        fp_t: dict[int, int] = defaultdict(int)
        fn_t: dict[int, int] = defaultdict(int)
        for fname, w, h, dets, gts in cache:
            preds = [d for d in dets if d[0] >= conf]
            tp, fp, fn = match(preds, gts, iou_thr=args.iou)
            for k, v in tp.items(): tp_t[k] += v
            for k, v in fp.items(): fp_t[k] += v
            for k, v in fn.items(): fn_t[k] += v

        cls_order = [4, 0, 1, 2, 3, 5, 6]
        for c in cls_order:
            tp = tp_t[c]; fp = fp_t[c]; fn = fn_t[c]
            p = tp / (tp + fp) if (tp + fp) > 0 else 0.0
            r = tp / (tp + fn) if (tp + fn) > 0 else 0.0
            f1 = 2 * p * r / (p + r) if (p + r) > 0 else 0.0
            print(f"{conf:>5.2f} | {CLASS_NAMES[c]:<10} {p:>7.2%} {r:>7.2%} "
                  f"{f1:>7.2%} {tp:>5} {fp:>5} {fn:>5}")
        all_tp = sum(tp_t.values())
        all_fp = sum(fp_t.values())
        all_fn = sum(fn_t.values())
        p = all_tp / (all_tp + all_fp) if (all_tp + all_fp) > 0 else 0.0
        r = all_tp / (all_tp + all_fn) if (all_tp + all_fn) > 0 else 0.0
        f1 = 2 * p * r / (p + r) if (p + r) > 0 else 0.0
        print(f"{conf:>5.2f} | {'ALL':<10} {p:>7.2%} {r:>7.2%} {f1:>7.2%} "
              f"{all_tp:>5} {all_fp:>5} {all_fn:>5}")
        print("-" * len(header))


if __name__ == "__main__":
    main()
