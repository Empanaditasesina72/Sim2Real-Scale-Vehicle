"""On-device perception-to-actuation latency benchmark (Raspberry Pi 5).

RUN THIS ON THE PI. It measures the REAL per-control-cycle latency of the
production pipeline on the physical hardware (Pi 5 + IMX500), so the paper can
report an on-device number instead of a simulated one. It is SAFE: it runs the
full perception + decision path but NEVER writes to the motor (the car does
not move).

Formal latency definition (state this verbatim in the paper):
    Per-cycle latency L = t_read + t_lane + t_sign + t_control, where
      t_read    = read the latest camera frame,
      t_lane    = LanePipeline.process()  (BEV + HSV + sliding windows),
      t_sign    = sign gating (get detections + stop/red decision),
      t_control = PID.compute() + servo-command formation.
    On the IMX500 path, YOLO inference runs ON THE SENSOR in parallel, so it
    does not enter L (that is the point of the monolithic on-sensor design);
    L is the CPU work the Pi 5 performs each 50 Hz (20 ms) control cycle.
    A cycle "misses the deadline" when L > 20 ms.

Output:
    validation_results/bench_latency_pi.csv   (per-cycle, latency_ms column)
Then analyse the distribution with:
    python tools/latency_stats.py validation_results/bench_latency_pi.csv \
           --deadline 20 --label "Pi 5 + IMX500 (on-device)"

Usage (from TMR2026/):
    python tools/bench_latency.py                 # 3000 cycles, ~60 s
    python tools/bench_latency.py --cycles 6000 --hz 50
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import numpy as np

from config import (
    SERVO_CENTER_ANGLE as SERVO_CENTER,
    SERVO_MIN_ANGLE    as SERVO_MIN,
    SERVO_MAX_ANGLE    as SERVO_MAX,
    USE_IMX500_NPU, IMX500_RPK_PATH, IMX500_LABELS_PATH, IMX500_CONF,
)
from vision.lane_pipeline import LanePipeline
from control.pid_controller import PIDController

CAMERA_W, CAMERA_H, CAMERA_FPS = 640, 480, 30
PID_KP, PID_KI, PID_KD = 0.08, 0.002, 0.025
YOLO_MODEL, YOLO_CONF, YOLO_IMGSZ = "weights/tmr_signs.pt", 0.55, 320


def _pi_temp_c():
    try:
        out = subprocess.check_output(["vcgencmd", "measure_temp"], timeout=2).decode()
        return float(out.strip().split("=")[1].split("'")[0])
    except Exception:
        return None


def _build_vision():
    """Same backend selection as main.py: IMX500 NPU if the .rpk exists, else CPU."""
    if USE_IMX500_NPU and os.path.isfile(IMX500_RPK_PATH):
        try:
            from vision.imx500_detector import IMX500CameraStream
            npu = IMX500CameraStream(
                rpk_path=IMX500_RPK_PATH, labels_path=IMX500_LABELS_PATH,
                width=CAMERA_W, height=CAMERA_H, fps=CAMERA_FPS, conf=IMX500_CONF,
            )
            print("[BENCH] Backend: IMX500 NPU (on-chip inference)")
            return npu, npu, "IMX500 NPU"
        except Exception as e:
            print(f"[BENCH] NPU unavailable ({e}) - falling back to CPU path.")

    from vision.camera_stream import CameraStream
    from vision.sign_detector import SignDetector
    cam = CameraStream(width=CAMERA_W, height=CAMERA_H, fps=CAMERA_FPS)
    sign = SignDetector(model_path=YOLO_MODEL, conf=YOLO_CONF, imgsz=YOLO_IMGSZ)
    print("[BENCH] Backend: CPU (CameraStream + SignDetector NCNN/.pt)")
    return cam, sign, "CPU NCNN"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--cycles", type=int, default=3000, help="control cycles to time")
    ap.add_argument("--hz", type=float, default=50.0, help="target loop rate")
    ap.add_argument("--out", default=None, help="output CSV path")
    args = ap.parse_args()

    deadline_ms = 1000.0 / args.hz
    out = args.out or str(ROOT / "validation_results" / "bench_latency_pi.csv")
    os.makedirs(os.path.dirname(out), exist_ok=True)

    camera, sign_det, backend = _build_vision()
    lane_pipe = LanePipeline(frame_w=CAMERA_W, frame_h=CAMERA_H, debug=False)
    pid = PIDController(
        kp=PID_KP, ki=PID_KI, kd=PID_KD, setpoint=0.0,
        output_limits=(-(SERVO_CENTER - SERVO_MIN), (SERVO_MAX - SERVO_CENTER)),
        integral_limits=(-25.0, 25.0),
    )

    camera.start()
    if sign_det is not camera:
        sign_det.start()

    print(f"[BENCH] Warming up... backend={backend}  temp={_pi_temp_c()} C")
    t_wait = time.monotonic()
    while camera.get_frame() is None and time.monotonic() - t_wait < 10:
        time.sleep(0.05)
    if camera.get_frame() is None:
        print("[BENCH] ERROR: no camera frames. Is the camera connected?")
        return 1

    rows = []
    period = 1.0 / args.hz
    t_last = time.monotonic()
    temp0 = _pi_temp_c()

    print(f"[BENCH] Measuring {args.cycles} cycles at {args.hz:.0f} Hz "
          f"(deadline {deadline_ms:.1f} ms)...")
    for i in range(args.cycles):
        c0 = time.perf_counter()
        frame = camera.get_frame()
        c1 = time.perf_counter()
        if frame is None:
            time.sleep(period); continue

        lane = lane_pipe.process(frame)
        c2 = time.perf_counter()

        if sign_det is not camera:
            sign_det.update_frame(frame)
        stop_like = sign_det.has_sign("stop_sign") or sign_det.has_sign("red")
        _ = sign_det.closest_sign("stop_sign") or sign_det.closest_sign("red")
        c3 = time.perf_counter()

        corr = pid.compute(lane.error_px, period)
        servo_cmd = max(SERVO_MIN, min(SERVO_MAX, SERVO_CENTER + corr))  # noqa: F841
        c4 = time.perf_counter()

        rows.append((
            i,
            (c1 - c0) * 1000.0,
            (c2 - c1) * 1000.0,
            (c3 - c2) * 1000.0,
            (c4 - c3) * 1000.0,
            (c4 - c0) * 1000.0,
        ))

        dt = time.monotonic() - t_last
        if dt < period:
            time.sleep(period - dt)
        t_last = time.monotonic()

        if i % 500 == 0 and i:
            print(f"  ... {i}/{args.cycles}  temp={_pi_temp_c()} C")

    temp1 = _pi_temp_c()
    camera.stop()
    if sign_det is not camera:
        sign_det.stop()

    import csv
    with open(out, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["cycle", "t_read_ms", "t_lane_ms", "t_sign_ms",
                    "t_control_ms", "latency_ms"])
        w.writerows(rows)

    lat = np.array([r[5] for r in rows][1:], dtype=np.float64)  # drop warm-up cycle
    print("=" * 58)
    print(f"  ON-DEVICE LATENCY  ({backend})")
    print("=" * 58)
    print(f"  cycles       : {lat.size}")
    print(f"  mean/median  : {lat.mean():.2f} / {np.median(lat):.2f} ms")
    print(f"  std (jitter) : {lat.std(ddof=1):.2f} ms")
    print(f"  p95 / p99    : {np.percentile(lat,95):.2f} / {np.percentile(lat,99):.2f} ms")
    print(f"  max          : {lat.max():.2f} ms")
    print(f"  under {deadline_ms:.0f} ms   : {100*np.mean(lat<=deadline_ms):.2f} %")
    print(f"  CPU temp     : {temp0} -> {temp1} C")
    print("=" * 58)
    print(f"[BENCH] CSV: {out}")
    print(f"[BENCH] Full distribution + figure:")
    print(f'        python tools/latency_stats.py "{out}" --deadline {deadline_ms:.0f} '
          f'--label "Pi 5 + IMX500 (on-device)"')
    return 0


if __name__ == "__main__":
    sys.exit(main())
