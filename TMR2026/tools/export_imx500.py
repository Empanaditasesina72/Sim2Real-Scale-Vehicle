"""Convert tmr_signs.pt to the IMX500 NPU format (.rpk).

The Pi AI Camera's Sony IMX500 runs the model INSIDE the sensor: the Pi
receives the already-inferred tensors in each frame's metadata and the CPU
stays free. For that, the model must be quantized (INT8) and packaged as a
.rpk with Sony's toolchain -- this script does that, via Ultralytics' `imx`
export.

WARNING: runs on LINUX ONLY (the Pi itself works; Windows has no
  imx500-converter). Quantization uses the traffic_lights/ dataset for
  calibration and can take 15-60 min on the Pi 5 -- done ONCE.

Prerequisites (once, on the Pi):
    sudo apt install -y imx500-all imx500-tools default-jre
    pip3 install --break-system-packages model-compression-toolkit "imx500-converter[pt]"
    # (ultralytics tries to auto-install anything missing)

Usage (from TMR2026/):
    python tools/export_imx500.py                  # full export
    python tools/export_imx500.py --fraction 0.1   # faster calibration

When finished it leaves:
    weights/tmr_signs_imx500.rpk           <- loaded by main.py (config.py)
    weights/tmr_signs_imx500_labels.txt    <- model class order

main.py detects it on its own at the next startup:
    [VISION] Backend: IMX500 NPU (on-chip inference)
"""

import argparse
import shutil
import sys
from pathlib import Path

HERE     = Path(__file__).resolve().parent.parent
WEIGHTS  = HERE / "weights" / "tmr_signs.pt"
DATA     = HERE.parent / "traffic_lights" / "data.yaml"
DST_RPK  = HERE / "weights" / "tmr_signs_imx500.rpk"
DST_LBL  = HERE / "weights" / "tmr_signs_imx500_labels.txt"

FALLBACK_LABELS = ("green", "left", "red", "right", "stop", "straight", "yellow")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--fraction", type=float, default=0.25,
                    help="fraction of the dataset for INT8 calibration "
                         "(default 0.25 -- raise to 1.0 for max accuracy)")
    args = ap.parse_args()

    if not sys.platform.startswith("linux"):
        print("[IMX500] Sony's imx500-converter only exists on Linux.")
        print("[IMX500] Run this script ON THE RASPBERRY PI (or a Linux box):")
        print("[IMX500]     cd ~/Carrito/TMR2026 && python tools/export_imx500.py")
        return 1

    if not WEIGHTS.exists():
        print(f"[IMX500] {WEIGHTS} does not exist")
        return 1
    if not DATA.exists():
        print(f"[IMX500] Calibration dataset does not exist: {DATA}")
        return 1

    from ultralytics import YOLO

    print(f"[IMX500] Loading {WEIGHTS} ...")
    model = YOLO(str(WEIGHTS))
    print(f"[IMX500] Classes: {model.names}")
    print(f"[IMX500] Exporting to imx format (INT8, {args.fraction:.0%} "
          f"calibration of {DATA.name}) ...")
    print("[IMX500] This takes 15-60 min on the Pi 5. Once only. Be patient.")

    out = model.export(format="imx", data=str(DATA), fraction=args.fraction)
    out_dir = Path(out)
    print(f"[IMX500] Raw export at: {out_dir}")

    rpks = sorted(out_dir.rglob("*.rpk"))
    if not rpks:
        print("[IMX500] ERROR: the export produced no .rpk.")
        print("[IMX500] Check that imx500-tools and java are installed.")
        return 1
    shutil.copy2(rpks[0], DST_RPK)
    print(f"[IMX500] .rpk ready: {DST_RPK}")

    labels = sorted(out_dir.rglob("labels.txt"))
    if labels:
        shutil.copy2(labels[0], DST_LBL)
    else:
        DST_LBL.write_text("\n".join(FALLBACK_LABELS) + "\n", encoding="utf-8")
    print(f"[IMX500] labels:     {DST_LBL}")

    print("=" * 60)
    print("[IMX500] DONE. At the next startup main.py will show:")
    print("[IMX500]   [VISION] Backend: IMX500 NPU (on-chip inference)")
    print("[IMX500] To go back to the CPU path: config.py -> USE_IMX500_NPU=False")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(main())
