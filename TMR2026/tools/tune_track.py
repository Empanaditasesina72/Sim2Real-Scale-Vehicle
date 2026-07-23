"""Live on-track calibration tool (RUN ON THE PI, with --display / a monitor).

This is what makes the physical car follow the lane "like Unity": it shows the
camera + BEV + white mask + detected lane + PID output LIVE, with sliders to
tune the values that depend on your real track and lighting. When it looks
right, press 's' to save -> main.py loads them automatically on the next run.

It NEVER touches the motors (the car does not move). Pure vision tuning.

Sliders:
    V_min      HSV white brightness floor (raise if black leaks into the mask)
    S_max      HSV white saturation ceiling (lower to reject grey reflections)
    roi %      where the region-of-interest starts (top of the lane strip)
    r_bias %   how far the target sits toward the right line (TMR lane)
    Kp/Ki/Kd   steering PID gains (watch the servo target respond)

Keys:  s = save track_calib.json   |   r = reset defaults   |   q/ESC = quit

Workflow:
    1. python tools/tune_track.py         (on the Pi, car on the track)
    2. drag sliders until the green lane line is stable and centered
    3. press 's'
    4. python main.py --display           (now uses your calibration)
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import cv2
import numpy as np

from vision.camera_stream import CameraStream
from vision.lane_pipeline import LanePipeline
from control.pid_controller import PIDController

try:
    from config import (SERVO_CENTER_ANGLE as SC, SERVO_MIN_ANGLE as SMIN,
                        SERVO_MAX_ANGLE as SMAX)
except Exception:
    SC, SMIN, SMAX = 90.0, 58.0, 122.0

CAMERA_W, CAMERA_H, CAMERA_FPS = 640, 480, 30
WIN = "Track Tuner  (s=save  r=reset  q=quit)"
CALIB_PATH = ROOT / "track_calib.json"

DEFAULTS = dict(v_min=130, s_max=60, roi=50, rbias=70, kp=80, ki=20, kd=25)


def _set(name, val):
    cv2.setTrackbarPos(name, WIN, val)


def _reset():
    for k, v in {"V_min": DEFAULTS["v_min"], "S_max": DEFAULTS["s_max"],
                 "roi %": DEFAULTS["roi"], "r_bias %": DEFAULTS["rbias"],
                 "Kp x1000": DEFAULTS["kp"], "Ki x1e4": DEFAULTS["ki"],
                 "Kd x1000": DEFAULTS["kd"]}.items():
        _set(k, v)


def main() -> int:
    cam = CameraStream(width=CAMERA_W, height=CAMERA_H, fps=CAMERA_FPS)
    cam.start()

    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.createTrackbar("V_min",    WIN, DEFAULTS["v_min"], 255, lambda *_: None)
    cv2.createTrackbar("S_max",    WIN, DEFAULTS["s_max"], 120, lambda *_: None)
    cv2.createTrackbar("roi %",    WIN, DEFAULTS["roi"],   60,  lambda *_: None)
    cv2.createTrackbar("r_bias %", WIN, DEFAULTS["rbias"], 90,  lambda *_: None)
    cv2.createTrackbar("Kp x1000", WIN, DEFAULTS["kp"],    300, lambda *_: None)
    cv2.createTrackbar("Ki x1e4",  WIN, DEFAULTS["ki"],    200, lambda *_: None)
    cv2.createTrackbar("Kd x1000", WIN, DEFAULTS["kd"],    100, lambda *_: None)

    print("[TUNE] Point the camera at the track. Adjust sliders, then press 's'.")

    while True:
        frame = cam.get_frame()
        if frame is None:
            if (cv2.waitKey(30) & 0xFF) in (ord("q"), 27):
                break
            continue

        v_min = cv2.getTrackbarPos("V_min", WIN)
        s_max = cv2.getTrackbarPos("S_max", WIN)
        roi   = max(20, cv2.getTrackbarPos("roi %", WIN)) / 100.0
        rbias = cv2.getTrackbarPos("r_bias %", WIN) / 100.0
        kp    = cv2.getTrackbarPos("Kp x1000", WIN) / 1000.0
        ki    = cv2.getTrackbarPos("Ki x1e4",  WIN) / 10000.0
        kd    = cv2.getTrackbarPos("Kd x1000", WIN) / 1000.0

        lp = LanePipeline(
            frame_w=CAMERA_W, frame_h=CAMERA_H, debug=True,
            right_bias=rbias, roi_frac=roi,
            hsv_white_lo=[0, 0, v_min], hsv_white_hi=[179, s_max, 255],
        )
        lane = lp.process(frame)
        pid = PIDController(kp=kp, ki=ki, kd=kd, setpoint=0.0,
                            output_limits=(-(SC - SMIN), (SMAX - SC)))
        corr = pid.compute(lane.error_px, 0.02)
        servo = max(SMIN, min(SMAX, SC + corr))

        vis = lp.draw_debug(frame, lane)
        if lane.bev_frame is not None and lane.mask_frame is not None:
            vis[0:180, 0:320]   = cv2.resize(lane.bev_frame,  (320, 180))
            vis[0:180, 320:640] = cv2.resize(lane.mask_frame, (320, 180))
            cv2.putText(vis, "BEV", (8, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(vis, "white mask", (328, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        info = [
            f"err:{lane.error_px:+6.1f}px  conf:{lane.confidence:.0%}  servo:{servo:5.1f}deg",
            f"V_min={v_min} S_max={s_max} roi={roi:.2f} r_bias={rbias:.2f}",
            f"Kp={kp:.3f} Ki={ki:.4f} Kd={kd:.3f}",
        ]
        for i, txt in enumerate(info):
            cv2.putText(vis, txt, (8, 210 + i * 22), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 255, 255), 2, cv2.LINE_AA)

        cv2.imshow(WIN, vis)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        elif key == ord("r"):
            _reset()
        elif key == ord("s"):
            calib = {
                "hsv_white_lo": [0, 0, v_min],
                "hsv_white_hi": [179, s_max, 255],
                "right_bias": rbias,
                "roi_frac": roi,
                "pid": {"kp": kp, "ki": ki, "kd": kd},
            }
            with open(CALIB_PATH, "w", encoding="utf-8") as f:
                json.dump(calib, f, indent=2)
            print(f"[TUNE] Saved -> {CALIB_PATH}")
            print(f"[TUNE] {json.dumps(calib)}")

    cam.stop()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
