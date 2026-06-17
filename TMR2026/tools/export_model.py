"""Export weights/tmr_signs.pt to NCNN for the Raspberry Pi 5.

Why NCNN?
  The Pi 5 runs YOLO on the CPU (ARM Cortex-A76). Plain PyTorch gets ~6-8 FPS
  with yolov8n@320; NCNN (the format Ultralytics recommends for the Raspberry
  Pi) gets 3-4x more at the SAME accuracy. SignDetector automatically loads
  `weights/tmr_signs_ncnn_model/` if it exists and falls back to the `.pt`
  otherwise.

Usage (from TMR2026/, on the PC or the Pi -- the result is portable):

    python tools/export_model.py                 # export to NCNN imgsz=320
    python tools/export_model.py --imgsz 416     # more range, a bit slower

The result is left in weights/tmr_signs_ncnn_model/ (param + bin + metadata
with the names of the 7 classes). It is committed to git so the Pi does not
have to export anything.
"""

import argparse
import sys
from pathlib import Path

HERE    = Path(__file__).resolve().parent.parent
WEIGHTS = HERE / "weights" / "tmr_signs.pt"

DEFAULT_IMGSZ = 320


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--imgsz", type=int, default=DEFAULT_IMGSZ,
                    help=f"inference size (default {DEFAULT_IMGSZ})")
    ap.add_argument("--weights", type=Path, default=WEIGHTS,
                    help="path to the .pt to export")
    args = ap.parse_args()

    if not args.weights.exists():
        print(f"[EXPORT] {args.weights} does not exist")
        return 1

    from ultralytics import YOLO

    print(f"[EXPORT] Loading {args.weights} ...")
    model = YOLO(str(args.weights))
    print(f"[EXPORT] Classes: {model.names}")

    print(f"[EXPORT] Exporting to NCNN (imgsz={args.imgsz}, FP16) ...")
    out = model.export(format="ncnn", imgsz=args.imgsz, half=True)

    print(f"[EXPORT] Done: {out}")
    print("[EXPORT] SignDetector will use it automatically on the next startup.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
