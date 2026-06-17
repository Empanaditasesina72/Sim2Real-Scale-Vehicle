"""Combined camera + lane pipeline + PID + YOLO preview.

Replicates what AUTONOMOUS mode computes but does NOT touch motors or servo.
Useful for:
  - Checking that the camera sees the track and white lines are isolated well.
  - Calibrating BEV_SRC_RATIO safely (no traction hardware is initialized).
  - Seeing how the PID responds (P / I / D / correction) to the current gains.
  - Verifying YOLO detections live.

Usage (from TMR2026/):
  python3 tools/test_camera.py            # with YOLO
  python3 tools/test_camera.py --no-yolo  # lane + PID only (faster startup)

Quit: 'q' or ESC in the window.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import cv2

from vision.camera_stream    import CameraStream
from vision.lane_pipeline    import LanePipeline
from vision.sign_detector    import SignDetector
from control.pid_controller  import PIDController


CAMERA_W, CAMERA_H, CAMERA_FPS = 640, 480, 30
SERVO_CENTER, SERVO_MIN, SERVO_MAX = 90.0, 45.0, 135.0
PID_KP, PID_KI, PID_KD = 0.08, 0.002, 0.025
PID_OUT_MIN = -(SERVO_CENTER - SERVO_MIN)
PID_OUT_MAX =  (SERVO_MAX - SERVO_CENTER)

YOLO_MODEL = "weights/tmr_signs.pt"
YOLO_CONF, YOLO_IMGSZ = 0.55, 320

USE_YOLO = "--no-yolo" not in sys.argv


def _draw_panel(img, x, y, w, h, lines):
    """Semi-transparent box with border + multi-line text (same as main.py)."""
    ov = img.copy()
    cv2.rectangle(ov, (x, y), (x + w, y + h), (0, 0, 0), -1)
    cv2.addWeighted(ov, 0.55, img, 0.45, 0, dst=img)
    cv2.rectangle(img, (x, y), (x + w, y + h), (255, 220, 0), 1)
    for i, line in enumerate(lines):
        cv2.putText(img, line, (x + 8, y + 20 + i * 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 220, 0), 1, cv2.LINE_AA)


def draw_overlay(frame, lane_pipe, lane, pid, angle_target, fps, dets):
    """Replicates _render_debug_view from main.py (no real motor/FSM/lidar)."""
    H, W = frame.shape[:2]

    vis = lane_pipe.draw_debug(frame, lane)

    if lane.bev_frame is not None and lane.mask_frame is not None:
        vis[0:180, 0:320]   = cv2.resize(lane.bev_frame,  (320, 180))
        vis[0:180, 320:640] = cv2.resize(lane.mask_frame, (320, 180))
        cv2.putText(vis, "BEV (bird's-eye)", (8, 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        cv2.putText(vis, "HSV white mask",   (328, 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

    _draw_panel(vis, 8, 200, 320, 160, lines=[
        f"MODE  : TEST",
        f"err   :{lane.error_px:+7.1f}px  conf:{lane.confidence:.0%}",
        f"P     :{pid.last_p:+7.2f}   kp={pid.kp:.3f}",
        f"I     :{pid.last_i:+7.2f}   ki={pid.ki:.3f}",
        f"D     :{pid.last_d:+7.2f}   kd={pid.kd:.3f}",
        f"corr  :{pid.last_output:+7.2f}d -> servo {angle_target:5.1f}d",
        f"lidar :---     FPS:{fps:5.1f}",
    ])

    if dets:
        obj_lines = ["DETECTED OBJECTS:"]
        for d in dets[:5]:
            dist = f" @{(d.distance_m or 0)*100:.0f}cm" if d.distance_m else ""
            obj_lines.append(f"- {d.label}  {d.confidence:.0%}{dist}")
    else:
        obj_lines = [
            "DETECTED OBJECTS:",
            "- (none)" if USE_YOLO else "- YOLO OFF (--no-yolo)",
        ]
    _draw_panel(vis, 336, 200, 296, 160, lines=obj_lines)

    cv2.putText(vis, f"TEST  duty:  0%   ('q'/ESC quit)",
                (8, H - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 255), 2, cv2.LINE_AA)

    for d in dets:
        cv2.rectangle(vis, (d.x1, d.y1), (d.x2, d.y2), (0, 255, 0), 2)
        dist_txt = f" {(d.distance_m or 0) * 100:.0f}cm" if d.distance_m else ""
        label_txt = f"{d.label} {d.confidence:.0%}{dist_txt}"
        (tw, th), _ = cv2.getTextSize(label_txt, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        ly = max(d.y1 - 6, th + 4)
        cv2.rectangle(vis, (d.x1, ly - th - 4), (d.x1 + tw + 4, ly + 2),
                      (0, 0, 0), -1)
        cv2.putText(vis, label_txt, (d.x1 + 2, ly - 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

    return vis


def main():
    print("[TEST] Starting camera + lane + PID preview (NO motors)")
    if not USE_YOLO:
        print("[TEST] YOLO disabled by --no-yolo flag")

    cam = CameraStream(width=CAMERA_W, height=CAMERA_H, fps=CAMERA_FPS)
    cam.start()

    lane_pipe = LanePipeline(frame_w=CAMERA_W, frame_h=CAMERA_H, debug=True)

    pid = PIDController(
        kp=PID_KP, ki=PID_KI, kd=PID_KD,
        setpoint=0.0,
        output_limits=(PID_OUT_MIN, PID_OUT_MAX),
        integral_limits=(-25.0, 25.0),
    )

    sign_det = None
    if USE_YOLO:
        sign_det = SignDetector(
            model_path=YOLO_MODEL, conf=YOLO_CONF, imgsz=YOLO_IMGSZ,
        )
        sign_det.start()

    t_prev = time.monotonic()
    fps_t0 = t_prev
    fps_count = 0
    fps = 0.0

    try:
        while True:
            frame = cam.get_frame()
            if frame is None:
                time.sleep(0.005)
                continue

            now = time.monotonic()
            dt = max(1e-3, now - t_prev)
            t_prev = now

            lane = lane_pipe.process(frame)

            correction = pid.compute(lane.error_px, dt)
            angle_target = max(
                SERVO_MIN, min(SERVO_MAX, SERVO_CENTER + correction)
            )

            dets = []
            if sign_det is not None:
                sign_det.update_frame(frame)
                dets = sign_det.get_detections()

            fps_count += 1
            if now - fps_t0 >= 0.5:
                fps = fps_count / (now - fps_t0)
                fps_count = 0
                fps_t0 = now

            vis = draw_overlay(frame, lane_pipe, lane, pid, angle_target, fps, dets)
            cv2.imshow("TMR 2026 - Vision Debug (TEST)", vis)

            sign_txt = (
                ", ".join(f"{d.label}({d.confidence:.0%})" for d in dets)
                or "-"
            )
            print(
                f"\r[TEST] err:{lane.error_px:+6.1f}px conf:{lane.confidence:.0%}  "
                f"P:{pid.last_p:+5.2f} I:{pid.last_i:+5.2f} D:{pid.last_d:+5.2f}  "
                f"corr:{pid.last_output:+5.2f}d angle:{angle_target:5.1f}d  "
                f"fps:{fps:4.1f}  signs:{sign_txt}    ",
                end="", flush=True,
            )

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
    finally:
        print("\n[TEST] Closing...")
        cam.stop()
        if sign_det is not None:
            sign_det.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
