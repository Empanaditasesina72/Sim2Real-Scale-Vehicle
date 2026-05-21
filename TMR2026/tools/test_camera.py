# -*- coding: utf-8 -*-
"""
test_camera.py — Preview combinado cámara + lane pipeline + PID + YOLO.

Replica lo que computa el modo AUTONOMOUS pero NO toca motores ni servo.
Útil para:
  • Comprobar que la cámara ve la pista y los blancos se aíslan bien.
  • Calibrar BEV_SRC_RATIO sin riesgo (no se inicializa hardware de tracción).
  • Ver cómo responde el PID (P / I / D / corrección) a las ganancias actuales.
  • Verificar detecciones de YOLO en vivo.

Uso (desde TMR2026/):
  python3 tools/test_camera.py            # con YOLO
  python3 tools/test_camera.py --no-yolo  # solo lane + PID (más rápido al iniciar)

Salir: tecla 'q' o ESC en la ventana.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

# Permitir ejecución desde cualquier CWD: agregar TMR2026/ al sys.path.
ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import cv2

from vision.camera_stream    import CameraStream
from vision.lane_pipeline    import LanePipeline
from vision.sign_detector    import SignDetector
from control.pid_controller  import PIDController


# ── Mismas constantes que main.py ──────────────────────────────────────────────
CAMERA_W, CAMERA_H, CAMERA_FPS = 640, 480, 30
SERVO_CENTER, SERVO_MIN, SERVO_MAX = 90.0, 45.0, 135.0
PID_KP, PID_KI, PID_KD = 0.08, 0.002, 0.025
PID_OUT_MIN = -(SERVO_CENTER - SERVO_MIN)
PID_OUT_MAX =  (SERVO_MAX - SERVO_CENTER)

YOLO_MODEL = "weights/tmr_signs.pt"
YOLO_CONF, YOLO_IMGSZ = 0.55, 320

USE_YOLO = "--no-yolo" not in sys.argv


def draw_overlay(frame, lane, pid, angle_target, fps, dets):
    """Dibuja BEV + máscara + bboxes + caja con valores PID sobre el frame."""
    vis = frame.copy()
    H, W = vis.shape[:2]

    # Línea central del frame y centro detectado del carril.
    cv2.line(vis, (W // 2, H), (W // 2, H // 2), (0, 150, 150), 1)
    cx = max(0, min(W - 1, W // 2 + int(lane.error_px)))
    color = (0, 255, 0) if lane.confidence >= 0.5 else (0, 80, 255)
    cv2.line(vis, (cx, H), (cx, H // 2), color, 3)

    # Mosaicos BEV + máscara en la franja superior.
    if lane.bev_frame is not None and lane.mask_frame is not None:
        small_bev  = cv2.resize(lane.bev_frame,  (320, 180))
        small_mask = cv2.resize(lane.mask_frame, (320, 180))
        vis[0:180, 0:320]   = small_bev
        vis[0:180, 320:640] = small_mask

    # Bounding boxes YOLO.
    for d in dets:
        cv2.rectangle(vis, (d.x1, d.y1), (d.x2, d.y2), (0, 0, 255), 2)
        dist_txt = f" {(d.distance_m or 0) * 100:.0f}cm" if d.distance_m else ""
        cv2.putText(
            vis, f"{d.label} {d.confidence:.0%}{dist_txt}",
            (d.x1, max(d.y1 - 6, 12)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1, cv2.LINE_AA,
        )

    # Caja con valores PID (panel inferior-izquierdo).
    panel_x, panel_y = 8, 200
    panel_w, panel_h = 320, 140
    overlay = vis.copy()
    cv2.rectangle(
        overlay, (panel_x, panel_y),
        (panel_x + panel_w, panel_y + panel_h),
        (0, 0, 0), thickness=-1,
    )
    vis = cv2.addWeighted(overlay, 0.55, vis, 0.45, 0)

    y = panel_y + 20
    line_h = 20

    def put(text):
        nonlocal y
        cv2.putText(
            vis, text, (panel_x + 8, y),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 220, 0), 1, cv2.LINE_AA,
        )
        y += line_h

    put(f"err   :{lane.error_px:+7.1f}px  conf:{lane.confidence:.0%}")
    put(f"P     :{pid.last_p:+7.2f}   kp={pid.kp:.3f}")
    put(f"I     :{pid.last_i:+7.2f}   ki={pid.ki:.3f}")
    put(f"D     :{pid.last_d:+7.2f}   kd={pid.kd:.3f}")
    put(f"corr  :{pid.last_output:+7.2f}d  servo->{angle_target:5.1f}d")
    put(f"FPS   :{fps:5.1f}    YOLO:{'ON' if USE_YOLO else 'OFF'}")

    return vis


def main():
    print("[TEST] Iniciando preview cámara + lane + PID (SIN motores)")
    if not USE_YOLO:
        print("[TEST] YOLO deshabilitado por flag --no-yolo")

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

            # PID — solo cómputo, no toca el servo.
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

            vis = draw_overlay(frame, lane, pid, angle_target, fps, dets)
            cv2.imshow("TMR test - camara + lane + PID", vis)

            sign_txt = (
                ", ".join(f"{d.label}({d.confidence:.0%})" for d in dets)
                or "—"
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
        print("\n[TEST] Cerrando...")
        cam.stop()
        if sign_det is not None:
            sign_det.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
