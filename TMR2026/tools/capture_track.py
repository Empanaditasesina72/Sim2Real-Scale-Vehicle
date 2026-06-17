#!/usr/bin/env python3
"""Record BGR camera frames for offline lane-pipeline calibration (HSV/BEV).

Interactive mode (default): Enter to capture, 'q' + Enter to quit.
Automatic mode: --auto N -> one frame every N seconds until Ctrl+C.

Usage:
    cd /home/angel01/Carrito
    python TMR2026/tools/capture_track.py
    python TMR2026/tools/capture_track.py --auto 1.5

Output: TMR2026/tools/captures/track_<timestamp>.jpg (BGR JPG)

Requirements: the systemd service must not be running (it uses the camera).
    sudo systemctl stop carrito_tmr
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
TMR_ROOT = HERE.parent
sys.path.insert(0, str(TMR_ROOT))

import cv2
from vision.camera_stream import CameraStream

OUT_DIR = HERE / "captures"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--auto", type=float, default=None,
                    help="Auto-capture cada N segundos (omitir = interactivo)")
    ap.add_argument("--width",  type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps",    type=int, default=30)
    args = ap.parse_args()

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    print(f"[CAP] Carpeta de salida: {OUT_DIR}")

    cam = CameraStream(width=args.width, height=args.height, fps=args.fps)
    cam.start()

    saved = 0

    def save_one() -> None:
        nonlocal saved
        f = cam.get_frame()
        if f is None:
            print("[CAP] frame=None, omitido")
            return
        ts = time.strftime("%Y%m%d_%H%M%S") + f"_{int(time.time() * 1000) % 1000:03d}"
        path = OUT_DIR / f"track_{ts}.jpg"
        cv2.imwrite(str(path), f)
        saved += 1
        print(f"[CAP] {saved:03d}  {path.name}  ({f.shape[1]}x{f.shape[0]})")

    try:
        if args.auto is not None:
            interval = max(0.1, float(args.auto))
            print(f"[CAP] AUTO — 1 frame cada {interval:.2f} s. Ctrl+C para salir.")
            while True:
                save_one()
                time.sleep(interval)
        else:
            print("[CAP] INTERACTIVO — Enter para capturar, 'q' + Enter para salir.")
            while True:
                line = sys.stdin.readline()
                if line == "":
                    break
                if line.strip().lower() == "q":
                    break
                save_one()
    except KeyboardInterrupt:
        pass
    finally:
        print(f"[CAP] Total guardado: {saved} frames")
        cam.stop()


if __name__ == "__main__":
    main()
