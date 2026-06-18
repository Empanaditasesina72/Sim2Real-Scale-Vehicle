#!/usr/bin/env python3
"""Record a real driving tub (frames + steering labels) for behavioral cloning.

Writes the SAME tub format as tools/gen_synth_driving.py:
    <out>/frames/NNNNNN.jpg
    <out>/labels.csv     header: frame,error_px,confidence,source
    <out>/tub.json

Frame sources (--source):
    sim      Unity simulator via SimulatorClient (127.0.0.1:5005). This tool
             DRIVES the car (autopilot) so it traverses the track while it
             records -> a real driving dataset.
    camera   Pi camera (vision/camera_stream.py). The car is driven externally
             (e.g. you push it, or run main.py AUTONOMOUS in another process);
             this tool only records frames + expert labels.
    video    a recorded .mp4/.avi
    images   a folder/glob of existing frames (re-label a capture set as a tub)

Label source (--label):
    expert   (default) error_px = LanePipeline.process(frame).error_px. The
             classic CV pipeline is the teacher; the network learns to imitate
             it but generalizes. Works for every source.
    human    read the gamepad and log your steering (mapped to px via
             --human-scale). Only meaningful for --source sim / camera, where
             you are actually driving.

Examples:
    # Generate a sim driving dataset with the CV pipeline as the expert:
    python TMR2026/tools/record_driving.py --source sim --max 4000 --throttle 18

    # Turn a folder of captured track photos into an expert-labeled tub:
    python TMR2026/tools/record_driving.py --source images \
        --path TMR2026/tools/captures --out datasets/track_real

    # Drive the sim yourself and clone YOUR steering:
    python TMR2026/tools/record_driving.py --source sim --label human --max 6000
"""
from __future__ import annotations

import argparse
import csv
import glob
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
TMR_ROOT = HERE.parent
sys.path.insert(0, str(TMR_ROOT))

from vision.lane_pipeline import LanePipeline
from control.pid_controller import PIDController

try:
    from config import (SERVO_CENTER_ANGLE as SERVO_CENTER,
                        SERVO_MIN_ANGLE as SERVO_MIN,
                        SERVO_MAX_ANGLE as SERVO_MAX,
                        STEER_KP, STEER_KI, STEER_KD)
except Exception:
    SERVO_CENTER, SERVO_MIN, SERVO_MAX = 90.0, 58.0, 122.0
    STEER_KP, STEER_KI, STEER_KD = 0.09, 0.002, 0.025


class Writer:
    def __init__(self, out: Path, w: int, h: int, source: str):
        self.out = out
        self.frames_dir = out / "frames"
        self.frames_dir.mkdir(parents=True, exist_ok=True)
        self.f = open(out / "labels.csv", "w", newline="")
        self.wr = csv.writer(self.f)
        self.wr.writerow(["frame", "error_px", "confidence", "source"])
        self.n = 0
        self.w, self.h, self.source = w, h, source

    def add(self, frame, error_px, conf):
        name = f"frames/{self.n:06d}.jpg"
        cv2.imwrite(str(self.out / name), frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
        self.wr.writerow([name, f"{error_px:.3f}", f"{conf:.3f}", self.source])
        self.n += 1

    def close(self):
        self.f.close()
        (self.out / "tub.json").write_text(json.dumps(
            {"width": self.w, "height": self.h, "count": self.n,
             "source": self.source, "label": "error_px"}, indent=2))
        print(f"[REC] wrote {self.n} frames -> {self.out}")


class Gamepad:
    """Minimal pygame steering reader (lazy). Returns steer in [-1, 1]."""

    def __init__(self):
        import pygame
        self.pygame = pygame
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            raise RuntimeError("no gamepad detected")
        self.js = pygame.joystick.Joystick(0)
        self.js.init()
        print(f"[REC] gamepad: {self.js.get_name()}")

    def steer(self) -> float:
        self.pygame.event.pump()
        v = self.js.get_axis(0)
        return 0.0 if abs(v) < 0.08 else float(max(-1.0, min(1.0, v)))


def frame_source(args):
    """Yield (frame_bgr, driver) where driver(angle, throttle) actuates if possible."""
    src = args.source
    if src == "sim":
        from sim_hardware_mocks import SimulatorClient
        sim = SimulatorClient(host=args.host, port=args.port)
        def driver(angle, throttle):
            sim.steering.set_angle(angle)
            sim.motor.set_speed(throttle)
        try:
            t_end = time.time() + args.warmup
            while time.time() < t_end:
                _ = sim.camera.get_latest_frame()
                time.sleep(0.03)
            while True:
                f = sim.camera.get_latest_frame()
                yield (f, driver)
        finally:
            try: sim.motor.stop()
            except Exception: pass

    elif src == "camera":
        from vision.camera_stream import CameraStream
        cam = CameraStream()
        cam.start()
        try:
            time.sleep(args.warmup)
            while True:
                yield (cam.get_frame(), None)
        finally:
            cam.stop()

    elif src == "video":
        cap = cv2.VideoCapture(args.path)
        if not cap.isOpened():
            raise SystemExit(f"cannot open video {args.path}")
        while True:
            ok, f = cap.read()
            if not ok:
                break
            yield (f, None)
        cap.release()

    elif src == "images":
        p = Path(args.path)
        files = sorted(glob.glob(str(p / "*.jpg")) + glob.glob(str(p / "*.png"))) \
            if p.is_dir() else sorted(glob.glob(args.path))
        for fp in files:
            yield (cv2.imread(fp), None)
    else:
        raise SystemExit(f"unknown source {src}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", required=True,
                    choices=["sim", "camera", "video", "images"])
    ap.add_argument("--label", default="expert", choices=["expert", "human"])
    ap.add_argument("--path", default="", help="for video/images sources")
    ap.add_argument("--out", default=str(TMR_ROOT / "datasets" / "drive_rec"))
    ap.add_argument("--max", type=int, default=4000, help="max frames (0=unlimited)")
    ap.add_argument("--throttle", type=float, default=18.0, help="sim autopilot duty %%")
    ap.add_argument("--human-scale", type=float, default=150.0,
                    help="map human steer [-1,1] -> error_px")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--warmup", type=float, default=2.0)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5005)
    ap.add_argument("--every", type=int, default=1, help="keep 1 of every N frames")
    args = ap.parse_args()

    if args.source in ("video", "images") and not args.path:
        raise SystemExit("--path is required for --source video/images")

    lane = LanePipeline(frame_w=args.width, frame_h=args.height)
    pid = PIDController(kp=STEER_KP, ki=STEER_KI, kd=STEER_KD,
                        output_limits=(-(SERVO_CENTER - SERVO_MIN),
                                       (SERVO_MAX - SERVO_CENTER)))
    pad = Gamepad() if args.label == "human" else None

    writer = Writer(Path(args.out), args.width, args.height, args.source)
    print(f"[REC] source={args.source} label={args.label} -> {args.out}")
    print("[REC] Ctrl+C to stop.")

    last = time.monotonic()
    seen = 0
    try:
        for frame, driver in frame_source(args):
            if frame is None:
                time.sleep(0.01)
                continue
            now = time.monotonic()
            dt = max(1e-3, now - last)
            last = now

            res = lane.process(frame)
            error_px, conf = res.error_px, res.confidence

            if pad is not None:
                steer = pad.steer()
                error_px = steer * args.human_scale
                conf = 1.0
                angle = SERVO_CENTER + steer * (SERVO_MAX - SERVO_CENTER)
            else:
                angle = SERVO_CENTER + pid.compute(error_px, dt)
            angle = max(SERVO_MIN, min(SERVO_MAX, angle))

            if driver is not None:
                driver(angle, args.throttle)

            seen += 1
            if seen % max(1, args.every) == 0:
                writer.add(frame, error_px, conf)
                if writer.n % 200 == 0:
                    print(f"  [REC] {writer.n} frames  err:{error_px:+.0f}px conf:{conf:.0%}")
            if args.max and writer.n >= args.max:
                break
    except KeyboardInterrupt:
        print("\n[REC] stopped by user")
    finally:
        writer.close()


if __name__ == "__main__":
    main()
