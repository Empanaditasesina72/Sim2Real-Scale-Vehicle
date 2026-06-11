# -*- coding: utf-8 -*-
"""
sign_detector.py — Detección de señales de tráfico con YOLOv8n (hilo independiente).

Corre en un hilo demonio a ~8-12 FPS en Pi 5 CPU.
El hilo de control nunca espera al detector — consume el último resultado disponible.

Clases esperadas en el modelo (índices ajustables en SIGN_CLASSES):
  0 → stop_sign
  1 → crosswalk

Modelo por defecto: weights/tmr_signs.pt (entrenado para señales TMR).
Fallback:           weights/yolov8n.pt    (modelo COCO — usa stop sign y persona).
"""

import os
import threading
import time
from typing import Optional

import cv2
import numpy as np

try:
    from config import STOP_SIGN_REAL_HEIGHT_M, CAMERA_FOCAL_LENGTH_PX
except ImportError:
    # Valores de respaldo si no se corre desde TMR2026/ como CWD.
    # Calibrados 2026-05-25 con señal mini (~4 cm) sobre cono.
    STOP_SIGN_REAL_HEIGHT_M = 0.04
    CAMERA_FOCAL_LENGTH_PX  = 490.0

# Altura real (m) de cada clase del modelo tmr_signs.pt — para estimar
# distancia por pinhole. Solo estas 7 clases se aceptan del modelo.
SIGN_REAL_HEIGHT_M = {
    "stop":     STOP_SIGN_REAL_HEIGHT_M,  # octágono (medido 4 cm)
    "red":      0.06,   # luz de semáforo
    "green":    0.06,
    "yellow":   0.06,
    "left":     0.05,   # flecha direccional
    "right":    0.05,
    "straight": 0.05,
}


# ── Detector de STOP por COLOR (respaldo cuando YOLO falla) ──────────────────
# Busca regiones de color rojo/granate/púrpura en HSV.
# Se activa SOLO si YOLO no detecta nada para no duplicar bboxes.
#
# Cubre dos rangos de rojo (el matiz cruza el límite 0/179 de HSV).
_RED_HSV_LO_1 = np.array([  0, 100,  60])   # rojo brillante
_RED_HSV_HI_1 = np.array([ 12, 255, 255])
_RED_HSV_LO_2 = np.array([165, 100,  60])   # rojo oscuro / magenta
_RED_HSV_HI_2 = np.array([179, 255, 255])
# Púrpura / morado (señales TMR estilizadas con fondo morado)
_PURPLE_HSV_LO = np.array([120,  60,  40])
_PURPLE_HSV_HI = np.array([160, 255, 255])

# Área mínima del contorno (px²). Subido de 600 → 1500 para evitar que
# cajas/objetos rojos lejanos (que se ven pequeños) se confundan con STOP.
# 1500 px ≈ 38×38 px, una señal real grande o de medio plano.
_COLOR_MIN_AREA = 1500
# Fracción mínima del bbox que debe estar pintada del color objetivo.
# Una señal real tiene su rojo concentrado; una caja con detalles rojos
# tiene baja "compacidad". Esto descarta blobs irregulares.
_COLOR_FILL_RATIO_MIN = 0.55
# Razón de aspecto permitida (ancho/alto). STOP octogonal ≈ 1.0,
# rectangulares ≈ 0.7–1.3. Apretamos el rango para excluir rectángulos.
_COLOR_ASPECT_MIN = 0.65
_COLOR_ASPECT_MAX = 1.50


def _detect_red_blob(frame_bgr: np.ndarray):
    """
    Devuelve (x1, y1, x2, y2, area) del contorno rojo/púrpura más grande,
    o None si no encuentra nada plausible.

    Filtros aplicados:
      • área >= _COLOR_MIN_AREA      → rechaza objetos lejanos
      • aspect en [0.65, 1.50]       → rechaza cajas alargadas
      • fill_ratio >= 0.55           → rechaza blobs huecos / irregulares
    """
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    m1 = cv2.inRange(hsv, _RED_HSV_LO_1,    _RED_HSV_HI_1)
    m2 = cv2.inRange(hsv, _RED_HSV_LO_2,    _RED_HSV_HI_2)
    m3 = cv2.inRange(hsv, _PURPLE_HSV_LO,   _PURPLE_HSV_HI)
    mask = cv2.bitwise_or(cv2.bitwise_or(m1, m2), m3)
    # Limpiar ruido
    k = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
    best = None
    best_area = 0
    for c in cnts:
        x, y, w, h = cv2.boundingRect(c)
        bbox_area = w * h
        if bbox_area < _COLOR_MIN_AREA:
            continue
        aspect = w / max(1.0, h)
        if not (_COLOR_ASPECT_MIN <= aspect <= _COLOR_ASPECT_MAX):
            continue
        # fill_ratio: cuánto del bbox está realmente pintado del color
        contour_area = cv2.contourArea(c)
        fill_ratio = contour_area / max(1.0, bbox_area)
        if fill_ratio < _COLOR_FILL_RATIO_MIN:
            continue
        if bbox_area > best_area:
            best_area = bbox_area
            best = (x, y, x + w, y + h, bbox_area)
    return best


class Detection:
    """Una detección confirmada de señal."""
    __slots__ = ("label", "confidence", "x1", "y1", "x2", "y2", "distance_m")

    def __init__(self, label: str, confidence: float,
                 x1: int, y1: int, x2: int, y2: int,
                 distance_m: Optional[float] = None):
        self.label      = label
        self.confidence = confidence
        self.x1 = x1; self.y1 = y1
        self.x2 = x2; self.y2 = y2
        self.distance_m = distance_m   # estimación pinhole a partir del bbox

    @property
    def area(self) -> int:
        return (self.x2 - self.x1) * (self.y2 - self.y1)

    @property
    def cx(self) -> int:
        return (self.x1 + self.x2) // 2

    @property
    def height_px(self) -> int:
        return max(1, self.y2 - self.y1)


class SignDetector:
    """
    Detector de señales STOP y crucero peatonal con YOLOv8n.

    Uso::

        sd = SignDetector("weights/tmr_signs.pt", conf=0.55, imgsz=320)
        sd.start()
        sd.update_frame(frame)          # llamar en cada frame de la cámara
        dets = sd.get_detections()      # non-blocking, retorna última lista
        sd.stop()
    """

    # Clases de señales relevantes para TMR (ajustar según modelo entrenado)
    SIGN_CLASSES = {"stop_sign", "stop sign", "crosswalk", "cross walk"}

    # Frecuencia máxima del detector (Hz). Con el modelo NCNN la Pi 5 infiere
    # en ~25-30 ms, así que 15 Hz cabe sobrado y la histéresis de 3 frames
    # confirma una señal en ~200 ms. Si solo está el .pt (PyTorch, más lento)
    # el hilo simplemente corre a su máximo real — este valor es solo el tope.
    MAX_HZ = 15.0

    # Histéresis: una etiqueta se publica solo si aparece en N frames seguidos.
    # Con el detector por color de respaldo activo, 3 frames es seguro
    # (evita parpadeos cuando algo rojo aparece momentáneamente).
    HYSTERESIS_FRAMES = 3

    def __init__(
        self,
        model_path: str  = "weights/tmr_signs.pt",
        conf:       float = 0.55,
        imgsz:      int   = 320,
        hysteresis_frames: int = HYSTERESIS_FRAMES,
    ):
        self._conf   = conf
        self._imgsz  = imgsz
        self._hysteresis = max(1, hysteresis_frames)
        self._model  = None

        self._frame:      Optional[np.ndarray] = None
        self._frame_lock  = threading.Lock()
        self._results:    list[Detection] = []
        self._result_lock = threading.Lock()

        # Contador de frames consecutivos por etiqueta — clave de la histéresis
        self._consecutive: dict[str, int] = {}
        # Última detección cruda vista por etiqueta (para re-emitir cuando se confirma)
        self._last_raw:    dict[str, Detection] = {}

        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None

        # Cargar modelo (puede tardar ~3 s en Pi 5 con NCNN/ONNX)
        self._model_path = model_path
        self._load_model()

    # ─── Ciclo de vida ────────────────────────────────────────────────────────

    def start(self) -> None:
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._detect_loop,
            name="SignDetector",
            daemon=True,
        )
        self._thread.start()
        print(f"[YOLO] Hilo de detección iniciado (imgsz={self._imgsz}, conf={self._conf})")

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=2.0)

    # ─── API pública (thread-safe) ────────────────────────────────────────────

    def update_frame(self, frame: np.ndarray) -> None:
        """Provee un nuevo frame al detector. No bloqueante."""
        with self._frame_lock:
            self._frame = frame   # referencia, no copia — frame no se modifica

    def get_detections(self) -> list[Detection]:
        """Retorna la lista de detecciones más reciente. No bloqueante."""
        with self._result_lock:
            return list(self._results)

    def has_sign(self, label: str) -> bool:
        """True si la etiqueta está en las detecciones actuales."""
        return any(d.label == label for d in self.get_detections())

    def has_any_sign(self) -> bool:
        """True si hay alguna señal relevante detectada."""
        return len(self.get_detections()) > 0

    def closest_sign(self, label: Optional[str] = None) -> Optional[Detection]:
        """
        Retorna la detección con menor `distance_m` (la más cercana).
        Filtra por etiqueta si se pasa.  None si no hay nada.
        """
        dets = self.get_detections()
        if label is not None:
            dets = [d for d in dets if d.label == label]
        dets = [d for d in dets if d.distance_m is not None]
        if not dets:
            return None
        return min(dets, key=lambda d: d.distance_m)

    # ─── Carga de modelo ─────────────────────────────────────────────────────

    def _resolve_model_path(self) -> str:
        """
        Prefiere la versión NCNN exportada (`<modelo>_ncnn_model/`) si existe
        junto al `.pt`. En la Pi 5 (CPU ARM) NCNN es 3-4× más rápido que
        PyTorch con la misma precisión. Se genera con `tools/export_model.py`
        y viene versionada en el repo — si falta, se usa el `.pt` normal.
        """
        path = self._model_path
        if path.endswith(".pt"):
            ncnn_dir = path[:-3] + "_ncnn_model"
            if os.path.isdir(ncnn_dir):
                return ncnn_dir
        return path

    def _load_model(self) -> None:
        try:
            from ultralytics import YOLO
            path = self._resolve_model_path()
            # task="detect" explícito: los exports NCNN no traen el task en
            # la metadata y ultralytics imprimiría un warning cada arranque.
            self._model = YOLO(path, task="detect")
            # Warm-up: una inferencia dummy para compilar el grafo
            dummy = np.zeros((self._imgsz, self._imgsz, 3), dtype=np.uint8)
            self._model(dummy, imgsz=self._imgsz, conf=self._conf, verbose=False)
            backend = "NCNN" if path.endswith("_ncnn_model") else "PyTorch"
            print(f"[YOLO] Modelo cargado ({backend}): {path}")
        except Exception as e:
            print(f"[YOLO] Modelo no disponible ({e}).")
            print("[YOLO] Usando detector de STOP por COLOR (rojo) como respaldo.")
            self._model = None

    # ─── Hilo de detección ────────────────────────────────────────────────────

    def _detect_loop(self) -> None:
        min_interval = 1.0 / self.MAX_HZ

        while not self._stop_event.is_set():
            t0 = time.monotonic()

            with self._frame_lock:
                frame = self._frame

            if frame is None:
                time.sleep(0.05)
                continue

            # YOLO solo si el modelo cargó (necesita ultralytics).
            raw_dets = []
            if self._model is not None:
                try:
                    results = self._model(
                        frame,
                        imgsz=self._imgsz,
                        conf=self._conf,
                        verbose=False,
                    )
                    raw_dets = self._parse_results(results, frame.shape)
                except Exception as e:
                    print(f"[YOLO] Error de inferencia: {e}")
                    raw_dets = []

            # ─── Respaldo por color cuando YOLO no detecta STOP ───────────────
            # Si YOLO no encontró 'stop_sign' pero hay una región roja/púrpura
            # grande en el frame (señal con estilo distinto al training set),
            # la reportamos como stop_sign con confianza moderada (0.55).
            has_yolo_stop = any(d.label == "stop_sign" for d in raw_dets)
            if not has_yolo_stop:
                blob = _detect_red_blob(frame)
                if blob is not None:
                    x1, y1, x2, y2, area = blob
                    height_px = max(1, y2 - y1)
                    distance_m = (STOP_SIGN_REAL_HEIGHT_M
                                  * CAMERA_FOCAL_LENGTH_PX) / height_px
                    raw_dets.append(Detection(
                        "stop_sign", 0.55, x1, y1, x2, y2,
                        distance_m=distance_m,
                    ))

            # ─── Histéresis: sólo publicamos etiquetas con N frames seguidos ───
            confirmed = self._apply_hysteresis(raw_dets)

            with self._result_lock:
                self._results = confirmed

            # Throttle — no saturar la CPU
            elapsed = time.monotonic() - t0
            sleep   = max(0.0, min_interval - elapsed)
            time.sleep(sleep)

    def _apply_hysteresis(self, raw_dets: list[Detection]) -> list[Detection]:
        """
        Filtro temporal: una detección sólo se confirma (y se publica) cuando
        aparece con la misma etiqueta en `self._hysteresis` frames consecutivos.

        - Guarda el contador por etiqueta en `self._consecutive`.
        - Guarda la última detección cruda por etiqueta en `self._last_raw`
          (usando la de mayor área si hay varias en el frame — suele ser la
          más cercana, que es la que le interesa a la FSM).
        """
        seen_this_frame: dict[str, Detection] = {}
        for d in raw_dets:
            prev = seen_this_frame.get(d.label)
            if prev is None or d.area > prev.area:
                seen_this_frame[d.label] = d

        # Actualizar contadores
        for label in list(self._consecutive.keys()):
            if label not in seen_this_frame:
                self._consecutive[label] = 0

        for label, det in seen_this_frame.items():
            self._consecutive[label] = self._consecutive.get(label, 0) + 1
            self._last_raw[label]    = det

        # Emitir sólo las confirmadas
        confirmed: list[Detection] = []
        for label, count in self._consecutive.items():
            if count >= self._hysteresis and label in self._last_raw:
                confirmed.append(self._last_raw[label])
        return confirmed

    def _parse_results(self, results, img_shape) -> list[Detection]:
        ih, iw = img_shape[:2]
        dets: list[Detection] = []

        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                conf   = float(box.conf[0])
                label  = (self._model.names.get(cls_id, str(cls_id))
                          .lower().replace(" ", "_"))

                # Aceptar las 7 clases del modelo tmr_signs.pt:
                #   green, left, red, right, stop, straight, yellow
                if label not in SIGN_REAL_HEIGHT_M:
                    continue

                x1, y1, x2, y2 = (int(v) for v in box.xyxy[0])

                # Normalizar si el modelo retorna coordenadas normalizadas [0,1]
                if x2 <= 1 and y2 <= 1:
                    x1 = int(x1 * iw); y1 = int(y1 * ih)
                    x2 = int(x2 * iw); y2 = int(y2 * ih)

                # Ignorar bboxes muy pequeños (señal muy lejana)
                area = (x2 - x1) * (y2 - y1)
                if area < 150:
                    continue

                # "stop" se expone como "stop_sign" (lo que ya consume el FSM);
                # las demás clases conservan su nombre del modelo.
                normalized = "stop_sign" if label == "stop" else label

                # Distancia por pinhole con la altura real de CADA clase:
                #   dist_m = (alto_real_m × focal_px) / alto_bbox_px
                height_px  = max(1, y2 - y1)
                real_h     = SIGN_REAL_HEIGHT_M[label]
                distance_m = (real_h * CAMERA_FOCAL_LENGTH_PX) / height_px

                dets.append(Detection(
                    normalized, conf, x1, y1, x2, y2,
                    distance_m=distance_m,
                ))

        return dets
