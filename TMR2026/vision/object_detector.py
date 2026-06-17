"""Post-processing of the IMX500 detections.

The detections already come resolved by the NPU in camera_manager.py.
This module provides high-level logic:
  - Filter detections by class and threshold.
  - Compute the estimated distance to STOP signs.
  - Detect a traffic-light color using the bbox region.
  - Provide structured information to the autonomous controller.
"""

from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np

from hardware.camera_manager import Detection
from config import (
    CAMERA_WIDTH, CAMERA_HEIGHT,
    STOP_SIGN_REAL_HEIGHT_M, CAMERA_FOCAL_LENGTH_PX,
    DETECTION_CONFIDENCE, DETECTION_MIN_FRAMES,
    OVERTAKE_LANE_RATIO, OVERTAKE_MIN_BBOX_AREA, OVERTAKE_TRIGGER_Y_MIN,
    PARK_GAP_CAMERA_ZONE,
)


@dataclass
class TrafficLightState:
    color: str
    confidence: float


class ObjectDetector:
    """
    Interpreta las detecciones del IMX500 y devuelve eventos semánticos
    para el controlador autónomo.

    Incluye filtro temporal: una detección solo se confirma si aparece
    en DETECTION_MIN_FRAMES frames consecutivos. Elimina falsos positivos.

    Uso:
        od = ObjectDetector()
        result = od.analyze(detections, frame)
        if result.stop_distance_mm:
            ...
    """

    def __init__(self):
        self._consecutive: dict[str, int] = {
            "STOP": 0, "SEMAFORO": 0, "PERSONA": 0, "AUTO": 0
        }

    @dataclass
    class AnalysisResult:
        stop_sign_detected: bool = False
        stop_sign_distance_mm: Optional[float] = None
        stop_sign_bbox: Optional[tuple] = None

        traffic_light: Optional["TrafficLightState"] = None

        person_detected: bool = False
        car_detected: bool = False
        car_bbox: Optional[tuple] = None
        car_in_lane: bool = False
        car_in_park_zone: bool = False
        closest_object_mm: Optional[float] = None

    def analyze(
        self,
        detections: list[Detection],
        frame: np.ndarray,
        tof_distance_mm: Optional[float] = None,
    ) -> "ObjectDetector.AnalysisResult":
        """
        Analiza la lista de detecciones y el frame opcional para extraer
        información semántica.

        Parameters
        ----------
        detections : list[Detection]
            Detecciones del NPU del IMX500.
        frame : np.ndarray
            Frame BGR original (para clasificación de color de semáforo).
        tof_distance_mm : float | None
            Lectura actual del VL53L0X (para confirmar distancia de STOP).
        """
        result      = self.AnalysisResult()
        seen_labels = set()

        for det in detections:
            if det.confidence < DETECTION_CONFIDENCE:
                continue

            seen_labels.add(det.label)

            if det.label == "STOP":
                if frame is not None and self._has_red_color(frame, det):
                    result.stop_sign_bbox = (det.x1, det.y1, det.x2, det.y2)
                    dist_bbox = det.estimated_distance_m()
                    if dist_bbox is not None:
                        if tof_distance_mm is not None and tof_distance_mm < 1200:
                            result.stop_sign_distance_mm = tof_distance_mm
                        else:
                            result.stop_sign_distance_mm = dist_bbox * 1000

            elif det.label == "SEMAFORO":
                tl = self._classify_traffic_light(frame, det)
                if tl.color != "unknown":
                    result.traffic_light = tl

            elif det.label == "PERSONA":
                result.person_detected = True

            elif det.label == "AUTO":
                bbox = (det.x1, det.y1, det.x2, det.y2)
                area = det.width * det.height
                if result.car_bbox is None:
                    result.car_bbox = bbox
                else:
                    prev = result.car_bbox
                    prev_area = (prev[2]-prev[0]) * (prev[3]-prev[1])
                    if area > prev_area:
                        result.car_bbox = bbox

        for label in self._consecutive:
            if label in seen_labels:
                self._consecutive[label] = min(
                    self._consecutive[label] + 1, DETECTION_MIN_FRAMES + 2)
            else:
                self._consecutive[label] = 0

        def confirmed(label: str) -> bool:
            return self._consecutive[label] >= DETECTION_MIN_FRAMES

        result.stop_sign_detected = (result.stop_sign_bbox is not None
                                     and confirmed("STOP"))
        result.car_detected = (result.car_bbox is not None
                               and confirmed("AUTO"))

        if tof_distance_mm is not None:
            result.closest_object_mm = tof_distance_mm

        if result.car_bbox is not None:
            x1, y1, x2, y2 = result.car_bbox
            cx      = (x1 + x2) / 2
            area    = (x2 - x1) * (y2 - y1)
            frame_cx = CAMERA_WIDTH / 2

            result.car_in_lane = (
                abs(cx - frame_cx) < CAMERA_WIDTH * OVERTAKE_LANE_RATIO
                and area >= OVERTAKE_MIN_BBOX_AREA
                and y2 >= OVERTAKE_TRIGGER_Y_MIN
            )
            result.car_in_park_zone = cx > CAMERA_WIDTH * PARK_GAP_CAMERA_ZONE

        return result

    def _has_red_color(self, frame: np.ndarray, det: Detection) -> bool:
        """
        Verifica que el bbox de la señal STOP contiene suficiente rojo.
        Filtra señales impresas con colores equivocados o falsos positivos.
        Retorna True si al menos el 8% de los píxeles del bbox son rojos.
        """
        pad = 4
        x1 = max(0, det.x1 + pad)
        y1 = max(0, det.y1 + pad)
        x2 = min(frame.shape[1] - 1, det.x2 - pad)
        y2 = min(frame.shape[0] - 1, det.y2 - pad)
        if x2 <= x1 or y2 <= y1:
            return True

        roi = frame[y1:y2, x1:x2]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        mask1 = cv2.inRange(hsv, np.array([0,  80, 80]),
                                 np.array([10, 255, 255]))
        mask2 = cv2.inRange(hsv, np.array([155, 80, 80]),
                                 np.array([180, 255, 255]))
        red_ratio = (np.sum(mask1 > 0) + np.sum(mask2 > 0)) / roi.size
        return red_ratio >= 0.06

    def _classify_traffic_light(
        self, frame: np.ndarray, det: Detection
    ) -> TrafficLightState:
        """
        Clasifica el color del semáforo usando la región del bbox.

        Estrategia:
          1. Recortar el bbox con margen pequeño.
          2. Convertir a HSV.
          3. Calcular máscara para rojo, amarillo y verde.
          4. El color con mayor número de píxeles detectados gana.
        """
        pad = 4
        x1 = max(0, det.x1 + pad)
        y1 = max(0, det.y1 + pad)
        x2 = min(frame.shape[1] - 1, det.x2 - pad)
        y2 = min(frame.shape[0] - 1, det.y2 - pad)

        if x2 <= x1 or y2 <= y1:
            return TrafficLightState("unknown", 0.0)

        roi = frame[y1:y2, x1:x2]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        ranges = {
            "red":    [((0, 100, 100), (10, 255, 255)),
                       ((160, 100, 100), (180, 255, 255))],
            "yellow": [((20, 100, 100), (35, 255, 255))],
            "green":  [((45, 60, 60), (85, 255, 255))],
        }

        scores: dict[str, int] = {}
        for color, range_list in ranges.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for (lo, hi) in range_list:
                mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
            scores[color] = int(np.sum(mask > 0))

        best_color = max(scores, key=scores.get)
        total_px   = roi.shape[0] * roi.shape[1]
        confidence = scores[best_color] / total_px if total_px > 0 else 0.0

        if confidence < 0.05:
            best_color = "unknown"
            confidence = 0.0

        return TrafficLightState(color=best_color, confidence=confidence)
