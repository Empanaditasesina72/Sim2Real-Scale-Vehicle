"""Physical straight-line braking experiment (Raspberry Pi 5 + real car).

RUN THIS ON THE PI, WITH THE CAR. It drives the car straight at a low fixed
speed toward a STOP sign and lets the REAL controller (AutonomousFSM + PID +
ToF) brake it, then logs the final stopping distance. Repeats for N trials so
the paper can report mean +/- std and a success rate instead of a single run,
and compare against the simulator (SIL stopped at 292.5 mm, setpoint 270 mm).

This is the physical counterpart of Test 2 (P2). Steering is held straight,
so it isolates the braking controller and is safe on a short straight track.

>>> SAFETY - read before running <<<
  * FIRST run with the drive wheels OFF THE GROUND to confirm behaviour.
  * Low speed only (default cruise 25 % PWM).
  * Emergency ToF cutoff: brakes hard if the front sensor reads < 120 mm.
  * Press Ctrl+C at any time -> immediate brake + exit.
  * Keep a hand ready to catch the car. Supervise every trial.

Setup per trial:
  * A short straight (>= ~1.2 m) with a STOP sign (red) at the end.
  * Mark a start line; place the car there each trial.

Output:
  validation_results/braking_physical.csv
    columns: trial, stopped_mm, min_mm, overshoot, within_tol, duration_s

Usage (from TMR2026/):
  python tools/bench_braking_physical.py --trials 10 --cruise 25
"""

from __future__ import annotations

import argparse
import csv
import os
import statistics
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from config import (
    PIN_MOTOR_RPWM, PIN_MOTOR_LPWM,
    SERVO_CENTER_ANGLE, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE,
    STOP_TARGET_MM, STOP_TOLERANCE_MM, EMERGENCY_STOP_MM,
    USE_IMX500_NPU, IMX500_RPK_PATH, IMX500_LABELS_PATH, IMX500_CONF,
)
from hardware.motor import MotorDriver
from hardware.steering_driver import SteeringDriver
from hardware.distance_sensor import DistanceSensor
from control.pid_controller import PIDController
from control.fsm import AutonomousFSM, FSMState

CAMERA_W, CAMERA_H, CAMERA_FPS = 640, 480, 30
PID_KP, PID_KI, PID_KD = 0.08, 0.002, 0.025
LOOP_HZ = 50
TRIAL_TIMEOUT_S = 25.0


def _build_vision():
    if USE_IMX500_NPU and os.path.isfile(IMX500_RPK_PATH):
        try:
            from vision.imx500_detector import IMX500CameraStream
            npu = IMX500CameraStream(
                rpk_path=IMX500_RPK_PATH, labels_path=IMX500_LABELS_PATH,
                width=CAMERA_W, height=CAMERA_H, fps=CAMERA_FPS, conf=IMX500_CONF)
            return npu, npu
        except Exception as e:
            print(f"[BRAKE] NPU unavailable ({e}) - CPU path.")
    from vision.camera_stream import CameraStream
    from vision.sign_detector import SignDetector
    cam = CameraStream(width=CAMERA_W, height=CAMERA_H, fps=CAMERA_FPS)
    sign = SignDetector(model_path="weights/tmr_signs.pt", conf=0.55, imgsz=320)
    return cam, sign


def run_trial(fsm, camera, sign_det, sensor, cruise_pwm) -> dict:
    fsm.MAX_AUTO_PWM = float(cruise_pwm)      # cap cruise speed for safety
    fsm.PRECAUCION_PWM = min(fsm.PRECAUCION_PWM, cruise_pwm * 0.6)
    fsm.activate()

    t0 = time.monotonic()
    t_last = t0
    min_mm = 1e9
    stopped_readings = []

    while time.monotonic() - t0 < TRIAL_TIMEOUT_S:
        now = time.monotonic()
        dt = now - t_last
        t_last = now

        if sign_det is not camera and camera.get_frame() is not None:
            sign_det.update_frame(camera.get_frame())

        front = sensor.front_mm
        if front is not None:
            min_mm = min(min_mm, front)
            if front < EMERGENCY_STOP_MM:        # hard safety cutoff
                fsm.motor.brake()
                break

        # straight-line braking test: force straight steering, real speed/brake logic
        fsm.lane_error = 0.0
        fsm.lane_conf = 1.0
        fsm.lidar_mm = front
        fsm.sign_visible = (sign_det.has_sign("stop_sign") or sign_det.has_sign("red"))
        closest = sign_det.closest_sign("stop_sign") or sign_det.closest_sign("red")
        fsm.sign_distance_mm = (closest.distance_m * 1000.0
                                if closest and closest.distance_m else None)
        fsm.update(dt)

        if fsm.state in (FSMState.ESPERA,):       # stopped -> sample distance
            if front is not None and front < 1000:
                stopped_readings.append(front)
            if len(stopped_readings) >= 15:       # ~0.3 s of stopped samples
                break

        time.sleep(max(0.0, (1.0 / LOOP_HZ) - (time.monotonic() - now)))

    fsm.deactivate()

    stopped = statistics.median(stopped_readings) if stopped_readings else None
    return {
        "stopped_mm": round(stopped, 1) if stopped else "",
        "min_mm": round(min_mm, 1) if min_mm < 1e9 else "",
        "overshoot": (min_mm < STOP_TARGET_MM - 2.5 * STOP_TOLERANCE_MM) if min_mm < 1e9 else "",
        "within_tol": (abs(stopped - STOP_TARGET_MM) <= STOP_TOLERANCE_MM) if stopped else "",
        "duration_s": round(time.monotonic() - t0, 2),
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--trials", type=int, default=10)
    ap.add_argument("--cruise", type=float, default=25.0, help="cruise PWM %% (keep low)")
    ap.add_argument("--out", default=str(ROOT / "validation_results" / "braking_physical.csv"))
    args = ap.parse_args()

    print("=" * 60)
    print("  PHYSICAL BRAKING EXPERIMENT (P2)  -  car will MOVE")
    print(f"  setpoint {STOP_TARGET_MM:.0f} mm, tolerance +/-{STOP_TOLERANCE_MM:.0f} mm, "
          f"cruise {args.cruise:.0f}% PWM")
    print("  Ctrl+C = emergency brake + exit")
    print("=" * 60)

    motor = MotorDriver(pin_rpwm=PIN_MOTOR_RPWM, pin_lpwm=PIN_MOTOR_LPWM)
    steering = SteeringDriver()
    sensor = DistanceSensor()
    camera, sign_det = _build_vision()
    pid = PIDController(kp=PID_KP, ki=PID_KI, kd=PID_KD, setpoint=0.0,
                        output_limits=(-(SERVO_CENTER_ANGLE - SERVO_MIN_ANGLE),
                                       (SERVO_MAX_ANGLE - SERVO_CENTER_ANGLE)),
                        integral_limits=(-25.0, 25.0))
    fsm = AutonomousFSM(motor, steering, pid)

    sensor.start()
    camera.start()
    if sign_det is not camera:
        sign_det.start()
    steering.center()

    results = []
    try:
        for k in range(1, args.trials + 1):
            input(f"\n[Trial {k}/{args.trials}] Place car at the start line, "
                  f"press Enter (Ctrl+C to stop)...")
            r = run_trial(fsm, camera, sign_det, sensor, args.cruise)
            r["trial"] = k
            results.append(r)
            print(f"  -> stopped {r['stopped_mm']} mm  (min {r['min_mm']} mm, "
                  f"within tol: {r['within_tol']})")
    except KeyboardInterrupt:
        print("\n[BRAKE] Aborted by user.")
    finally:
        motor.brake()
        steering.center()
        time.sleep(0.1)
        camera.stop()
        if sign_det is not camera:
            sign_det.stop()
        sensor.stop()
        motor.cleanup()

    if results:
        os.makedirs(os.path.dirname(args.out), exist_ok=True)
        with open(args.out, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=["trial", "stopped_mm", "min_mm",
                                              "overshoot", "within_tol", "duration_s"])
            w.writeheader()
            w.writerows(results)
        dists = [r["stopped_mm"] for r in results if isinstance(r["stopped_mm"], (int, float))]
        oks = sum(1 for r in results if r["within_tol"] is True)
        print("\n" + "=" * 60)
        if dists:
            print(f"  trials with a stop : {len(dists)}/{len(results)}")
            print(f"  stopping distance  : {statistics.mean(dists):.1f} +/- "
                  f"{statistics.pstdev(dists):.1f} mm  (setpoint {STOP_TARGET_MM:.0f})")
            print(f"  within +/-{STOP_TOLERANCE_MM:.0f} mm : {oks}/{len(results)} "
                  f"({100*oks/len(results):.0f}%)")
            print(f"  vs SIL (292.5 mm)  : diff "
                  f"{statistics.mean(dists) - 292.5:+.1f} mm")
        print(f"  CSV: {args.out}")
        print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(main())
