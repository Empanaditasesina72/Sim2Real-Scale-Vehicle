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

import threading
import time
from typing import Optional

import numpy as np


class Detection:
    """Una detección confirmada de señal."""
    __slots__ = ("label", "confidence", "x1", "y1", "x2", "y2")

    def __init__(self, label: str, confidence: float,
                 x1: int, y1: int, x2: int, y2: int):
        self.label      = label
        self.confidence = confidence
        self.x1 = x1; self.y1 = y1
        self.x2 = x2; self.y2 = y2

    @property
    def area(self) -> int:
        return (self.x2 - self.x1) * (self.y2 - self.y1)

    @property
    def cx(self) -> int:
        return (self.x1 + self.x2) // 2


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

    # Frecuencia máxima del detector (Hz) — Pi 5 CPU puede con ~15 FPS a 320px
    MAX_HZ = 12.0

    def __init__(
        self,
        model_path: str  = "weights/tmr_signs.pt",
        conf:       float = 0.55,
        imgsz:      int   = 320,
    ):
        self._conf   = conf
        self._imgsz  = imgsz
        self._model  = None

        self._frame:      Optional[np.ndarray] = None
        self._frame_lock  = threading.Lock()
        self._results:    list[Detection] = []
        self._result_lock = threading.Lock()

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

    # ─── Carga de modelo ─────────────────────────────────────────────────────

    def _load_model(self) -> None:
        try:
            from ultralytics import YOLO
            self._model = YOLO(self._model_path)
            # Warm-up: una inferencia dummy para compilar el grafo
            dummy = np.zeros((self._imgsz, self._imgsz, 3), dtype=np.uint8)
            self._model(dummy, imgsz=self._imgsz, conf=self._conf, verbose=False)
            print(f"[YOLO] Modelo cargado: {self._model_path}")
        except Exception as e:
            print(f"[YOLO] ERROR al cargar modelo: {e}")
            print("[YOLO] El detector de señales estará desactivado.")
            self._model = None

    # ─── Hilo de detección ────────────────────────────────────────────────────

    def _detect_loop(self) -> None:
        min_interval = 1.0 / self.MAX_HZ

        while not self._stop_event.is_set():
            t0 = time.monotonic()

            with self._frame_lock:
                frame = self._frame

            if frame is None or self._model is None:
                time.sleep(0.05)
                continue

            try:
                results = self._model(
                    frame,
                    imgsz=self._imgsz,
                    conf=self._conf,
                    verbose=False,
                )
                detections = self._parse_results(results, frame.shape)
            except Exception as e:
                print(f"[YOLO] Error de inferencia: {e}")
                detections = []

            with self._result_lock:
                self._results = detections

            # Throttle — no saturar la CPU
            elapsed = time.monotonic() - t0
            sleep   = max(0.0, min_interval - elapsed)
            time.sleep(sleep)

    def _parse_results(self, results, img_shape) -> list[Detection]:
        ih, iw = img_shape[:2]
        dets: list[Detection] = []

        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                conf   = float(box.conf[0])
                label  = (self._model.names.get(cls_id, str(cls_id))
                          .lower().replace(" ", "_"))

                # Filtrar solo clases relevantes para TMR
                if not any(k in label for k in ("stop", "crosswalk", "cross")):
                    continue

                x1, y1, x2, y2 = (int(v) for v in box.xyxy[0])

                # Normalizar si el modelo retorna coordenadas normalizadas [0,1]
                if x2 <= 1 and y2 <= 1:
                    x1 = int(x1 * iw); y1 = int(y1 * ih)
                    x2 = int(x2 * iw); y2 = int(y2 * ih)

                # Ignorar bboxes muy pequeños (señal muy lejana)
                area = (x2 - x1) * (y2 - y1)
                if area < 400:
                    continue

                # Normalizar etiqueta
                normalized = "stop_sign" if "stop" in label else "crosswalk"
                dets.append(Detection(normalized, conf, x1, y1, x2, y2))

        return dets
