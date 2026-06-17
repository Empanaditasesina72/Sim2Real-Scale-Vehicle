#!/usr/bin/env python3
"""4-panel visualization of the TMR 2026 vision module.

Panels:
  [TL] Y plane (640x480) -- grayscale lores stream
  [TR] Binary mask + morphology of the lane ROI (white lines)
  [BL] Main frame (640x360) with NPU bboxes and lane line
  [BR] Real-time FPS plot (60-frame sliding window)

Controls:
  Q / Esc  -> quit
  R        -> recalibrate AE/AWB (also reloads vision_config.yaml)

Usage:
  python3 test_vision.py
"""

import sys
import time
import logging

import cv2
import numpy as np

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)8s] %(name)s — %(message)s",
    datefmt="%H:%M:%S",
)

from vision_module import (
    VisionModule, VisionFSMState,
    _MAIN_W, _MAIN_H, _LORES_W, _LORES_H,
    _ROI_Y0, _ROI_Y1, _THRESH, _MORPH_K,
)

P_W = 640
P_H = 360
Y_H = 480

CANVAS_W = P_W * 2
CANVAS_H = P_H * 2

_COLORS = {
    "STOP":     (0,   0,   255),
    "SEMAFORO": (0,   200, 255),
    "PERSONA":  (255, 100, 0  ),
    "AUTO":     (200, 0,   200),
}

_FSM_COLOR = {
    VisionFSMState.LANE_FOLLOWING:  (0, 200, 0),
    VisionFSMState.SIGN_APPROACH:   (0, 200, 200),
    VisionFSMState.SIGN_BRAKING:    (0, 100, 255),
    VisionFSMState.STOPPED_AT_SIGN: (0, 0,   255),
    VisionFSMState.RESUMING:        (100, 200, 0),
    VisionFSMState.END_OF_TRACK:    (0, 0,   200),
    VisionFSMState.TELEOP_OVERRIDE: (200, 200, 0),
}


def _put(img, text, pos, color=(255, 255, 255), scale=0.5, thickness=1):
    cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX,
                scale, color, thickness, cv2.LINE_AA)


def _panel_y_plane(y_plane: np.ndarray) -> np.ndarray:
    """Panel TL: plano Y a color falso — escala de grises como BGR."""
    if y_plane is None:
        return np.zeros((P_H, P_W, 3), dtype=np.uint8)
    panel = cv2.cvtColor(cv2.resize(y_plane, (P_W, P_H)), cv2.COLOR_GRAY2BGR)
    roi0 = int(_ROI_Y0 * P_H / _LORES_H)
    roi1 = int(_ROI_Y1 * P_H / _LORES_H)
    cv2.line(panel, (0, roi0), (P_W, roi0), (0, 200, 200), 1)
    cv2.line(panel, (0, roi1), (P_W, roi1), (0, 200, 200), 1)
    _put(panel, "Plano Y (lores)", (8, 20), (200, 200, 0), 0.55, 1)
    return panel


def _panel_mask(y_plane: np.ndarray) -> np.ndarray:
    """Panel TR: binary mask of the lane ROI after MORPH_OPEN."""
    if y_plane is None:
        return np.zeros((P_H, P_W, 3), dtype=np.uint8)
    roi = y_plane[_ROI_Y0:_ROI_Y1, :]
    _, binary = cv2.threshold(roi, _THRESH, 255, cv2.THRESH_BINARY)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (_MORPH_K, _MORPH_K))
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)

    full = np.zeros((_LORES_H, _LORES_W), dtype=np.uint8)
    full[_ROI_Y0:_ROI_Y1, :] = binary

    panel = cv2.cvtColor(cv2.resize(full, (P_W, P_H)), cv2.COLOR_GRAY2BGR)

    hist = np.sum(binary, axis=0).astype(np.float32)
    if hist.max() > 0:
        hist_norm = (hist / hist.max() * 60).astype(int)
        for x, h in enumerate(hist_norm):
            xv = int(x * P_W / _LORES_W)
            if h > 0:
                cv2.line(panel, (xv, P_H - 1), (xv, P_H - 1 - h), (0, 200, 0), 1)

    _put(panel, f"Mascara thresh={_THRESH}", (8, 20), (200, 200, 0), 0.55, 1)
    return panel


def _panel_main(frame, vs, lane_error: float, lane_conf: float) -> np.ndarray:
    """Panel BL: main frame scaled to 640x360 with bboxes and lane line."""
    if frame is None:
        p = np.zeros((P_H, P_W, 3), dtype=np.uint8)
        _put(p, "Sin frame de camara", (20, P_H // 2), (100, 100, 100), 0.7, 2)
        return p

    panel = cv2.resize(frame, (P_W, P_H))
    sx = P_W / _MAIN_W
    sy = P_H / _MAIN_H

    for det in vs.raw_detections:
        c = _COLORS.get(det.label, (180, 180, 180))
        x1, y1 = int(det.x1*sx), int(det.y1*sy)
        x2, y2 = int(det.x2*sx), int(det.y2*sy)
        cv2.rectangle(panel, (x1, y1), (x2, y2), c, 2)
        _put(panel, f"{det.label} {det.confidence:.0%}", (x1, max(y1-4, 12)), c)

    mid   = P_W // 2
    cx    = max(0, min(P_W-1, mid + int(lane_error)))
    col   = (0, 255, 0) if lane_conf >= 0.30 else (0, 60, 255)
    cv2.line(panel, (mid, P_H), (mid, P_H//2), (0, 150, 150), 1)
    cv2.line(panel, (cx,  P_H), (cx,  P_H//2), col, 2)

    fsm_col = _FSM_COLOR.get(vs.fsm_state, (200, 200, 200))
    _put(panel, vs.fsm_state.name, (8, 20), fsm_col, 0.55, 2)
    _put(panel, f"err:{lane_error:+.0f}px  conf:{lane_conf:.0%}",
         (8, 42), col, 0.5, 1)
    if vs.stop_detected:
        d = f"{vs.stop_distance_mm:.0f}" if vs.stop_distance_mm else "?"
        _put(panel, f"STOP {d}mm", (8, 64), (0, 60, 255), 0.55, 2)
    return panel


def _panel_fps(fps_history: list) -> np.ndarray:
    """Panel BR: real-time FPS plot."""
    panel = np.zeros((P_H, P_W, 3), dtype=np.uint8)
    if not fps_history:
        _put(panel, "FPS graph", (P_W//2 - 40, P_H//2), (100, 100, 100), 0.6)
        return panel

    n      = len(fps_history)
    max_fps = max(fps_history) if max(fps_history) > 0 else 30
    step_x = P_W / max(n, 1)

    for fps_line, color in [(15, (0, 50, 200)), (25, (0, 150, 100)), (30, (60, 60, 60))]:
        y = int(P_H - (fps_line / max_fps) * (P_H - 30))
        cv2.line(panel, (0, y), (P_W, y), color, 1)
        _put(panel, str(fps_line), (P_W - 28, y - 2), color, 0.4)

    pts = []
    for i, f in enumerate(fps_history):
        x = int(i * step_x)
        y = int(P_H - (f / max_fps) * (P_H - 30))
        y = max(5, min(P_H - 5, y))
        pts.append((x, y))

    for i in range(1, len(pts)):
        col = (0, 200, 0) if fps_history[i] >= 25 else (0, 100, 255) if fps_history[i] >= 15 else (0, 0, 200)
        cv2.line(panel, pts[i-1], pts[i], col, 2)

    cur_fps = fps_history[-1]
    _put(panel, f"FPS: {cur_fps:.1f}", (8, 22), (200, 200, 0), 0.65, 2)
    return panel


def main():
    vm = VisionModule(display_overlay=False)
    vm.start()

    fps_history: list[float] = []
    MAX_HIST = 120

    print("test_vision.py iniciado — Q/Esc=salir  R=recalibrar")

    try:
        while True:
            vs      = vm.get_state()
            frame   = vm.get_latest_frame()
            y_plane = vm.get_latest_y_plane()

            fps_history.append(vs.fps)
            if len(fps_history) > MAX_HIST:
                fps_history.pop(0)

            p_tl = _panel_y_plane(y_plane)
            p_tr = _panel_mask(y_plane)
            p_bl = _panel_main(frame, vs, vs.lane_error, vs.lane_confidence)
            p_br = _panel_fps(fps_history)

            def to_ph(img):
                h, w = img.shape[:2]
                if h != P_H:
                    img = cv2.resize(img, (P_W, P_H))
                return img

            row_top = np.hstack([to_ph(p_tl), to_ph(p_tr)])
            row_bot = np.hstack([to_ph(p_bl), to_ph(p_br)])
            canvas  = np.vstack([row_top, row_bot])

            cv2.line(canvas, (P_W, 0), (P_W, CANVAS_H), (60, 60, 60), 1)
            cv2.line(canvas, (0, P_H), (CANVAS_W, P_H), (60, 60, 60), 1)

            cv2.imshow("TMR2026 — test_vision (Q=salir)", canvas)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), ord('Q'), 27):
                break
            elif key in (ord('r'), ord('R')):
                print("[TEST] Recalibrando AE/AWB...")
                vm.recalibrate_lighting()

            time.sleep(0.01)

    except KeyboardInterrupt:
        pass
    finally:
        vm.stop()
        cv2.destroyAllWindows()
        print("test_vision.py terminado.")


if __name__ == "__main__":
    main()
