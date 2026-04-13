# -*- coding: utf-8 -*-
"""
camera_manager.py — Pi AI Camera (Sony IMX500) vía Picamera2.

Estrategia NPU:
  El IMX500 ejecuta la inferencia YOLOv8 directamente en su acelerador
  neuronal integrado (NPU on-chip).  La Pi 5 recibe:
    1. El frame BGR888 como numpy array (para OpenCV / lane detection).
    2. Los tensores de salida del modelo embebidos en los metadatos de la
       captura (imx500.get_outputs).  Estos se parsean en objetos Detection.

  Beneficio: la CPU de la Pi 5 NO corre inferencia — solo parsea tensores
  pequeños.  La RAM no se satura con modelos pesados cargados en Python.

Requisito en Raspberry Pi OS:
  sudo apt install imx500-all picamera2
"""

import threading
import time
import queue
from dataclasses import dataclass, field
from typing import Optional

import numpy as np
from picamera2 import Picamera2
from picamera2.devices.imx500 import IMX500

from config import (
    IMX500_MODEL_PATH,
    CAMERA_WIDTH,
    CAMERA_HEIGHT,
    CAMERA_FPS,
    CAMERA_AWB_MODE,
    CAMERA_CONTRAST,
    CAMERA_SATURATION,
    CAMERA_SHARPNESS,
    CAMERA_DENOISE,
    CAMERA_BUFFERS,
    DETECTION_CONFIDENCE,
    CLASSES_OF_INTEREST,
    STOP_SIGN_REAL_HEIGHT_M,
    CAMERA_FOCAL_LENGTH_PX,
)


# ------------------------------------------------------------------
# Estructuras de datos
# ------------------------------------------------------------------
@dataclass
class Detection:
    label: str          # nombre legible (ej. "STOP")
    class_id: int
    confidence: float
    x1: int; y1: int
    x2: int; y2: int

    @property
    def width(self)  -> int: return self.x2 - self.x1
    @property
    def height(self) -> int: return self.y2 - self.y1
    @property
    def area(self)   -> int: return self.width * self.height
    @property
    def cx(self)     -> int: return (self.x1 + self.x2) // 2
    @property
    def cy(self)     -> int: return (self.y1 + self.y2) // 2

    def estimated_distance_m(self) -> Optional[float]:
        """
        Estima la distancia usando la altura del bounding box y la altura real
        conocida del objeto (método de perspectiva pin-hole).

        Solo preciso para objetos con altura real conocida (señal STOP).
        """
        if self.label != "STOP" or self.height < 5:
            return None
        return (STOP_SIGN_REAL_HEIGHT_M * CAMERA_FOCAL_LENGTH_PX) / self.height


@dataclass
class CameraFrame:
    image: np.ndarray               # BGR888, shape (H, W, 3)
    detections: list[Detection] = field(default_factory=list)
    timestamp: float = field(default_factory=time.monotonic)


# ------------------------------------------------------------------
# Gestor principal
# ------------------------------------------------------------------
class CameraManager:
    """
    Captura y procesado de la Pi AI Camera en hilo dedicado.

    El hilo produce CameraFrame objetos que el bucle principal consume
    mediante get_latest_frame() — siempre devuelve el frame más reciente,
    nunca bloquea más de unos ms.
    """

    def __init__(self):
        # --- IMX500: carga el modelo en el NPU de la cámara ---
        self._imx500 = IMX500(IMX500_MODEL_PATH)
        self._picam2 = Picamera2(self._imx500.camera_num)

        cfg = self._picam2.create_preview_configuration(
            main={"format": "BGR888", "size": (CAMERA_WIDTH, CAMERA_HEIGHT)},
            controls={
                "FrameRate"          : CAMERA_FPS,
                # Balance de blancos — Indoor corrige el tono azulado en interiores
                "AwbEnable"          : True,
                "AwbMode"            : CAMERA_AWB_MODE,
                # Exposición automática
                "AeEnable"           : True,
                # Calidad de imagen para detección
                "Contrast"           : CAMERA_CONTRAST,
                "Saturation"         : CAMERA_SATURATION,
                "Sharpness"          : CAMERA_SHARPNESS,
                "NoiseReductionMode" : CAMERA_DENOISE,
            },
            buffer_count=CAMERA_BUFFERS,
        )
        self._picam2.configure(cfg)

        # Cola de profundidad 1: el consumidor siempre lee el frame más fresco.
        self._queue: queue.Queue[CameraFrame] = queue.Queue(maxsize=1)

        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._last_frame: CameraFrame | None = None
        self._frame_lock = threading.Lock()

        # Mapa de class_id → label legible (se construye al iniciar)
        self._class_map: dict[int, str] = {}

    # ------------------------------------------------------------------
    # Control del hilo
    # ------------------------------------------------------------------
    def start(self):
        self._picam2.start()
        # Dar tiempo al IMX500 para cargar modelo y al AWB para estabilizarse
        time.sleep(3.0)

        # Intentar activar autofoco continuo (solo en cámaras con VCM)
        try:
            self._picam2.set_controls({
                "AfMode"  : 2,   # Continuous autofocus
                "AfSpeed" : 1,   # Fast
            })
            print("[CAM] Autoenfoque continuo activado")
        except Exception:
            print("[CAM] Autoenfoque no disponible — enfoque fijo")

        self._build_class_map()

        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._capture_loop,
            name="Camera-IMX500",
            daemon=True,
        )
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=3.0)
        self._picam2.stop()

    # ------------------------------------------------------------------
    # API pública (thread-safe)
    # ------------------------------------------------------------------
    def get_latest_frame(self) -> Optional[CameraFrame]:
        """
        Retorna el frame más reciente sin bloquear.
        Retorna None si aún no se ha capturado ninguno.
        """
        with self._frame_lock:
            return self._last_frame

    def get_frame_blocking(self, timeout: float = 0.1) -> Optional[CameraFrame]:
        """Espera hasta `timeout` segundos por un frame nuevo."""
        try:
            return self._queue.get(timeout=timeout)
        except queue.Empty:
            return None

    # ------------------------------------------------------------------
    # Hilo de captura
    # ------------------------------------------------------------------
    def _capture_loop(self):
        while not self._stop_event.is_set():
            try:
                # Captura sincronizada: frame + metadata en la misma petición
                request = self._picam2.capture_request()
                image = request.make_array("main")          # numpy BGR

                # --- Inferencia NPU: leer tensores de salida del IMX500 ---
                metadata = request.get_metadata()
                request.release()

                detections = self._parse_npu_output(metadata, image.shape)

                frame = CameraFrame(image=image, detections=detections)

                # Actualizar frame más reciente (para polling no bloqueante)
                with self._frame_lock:
                    self._last_frame = frame

                # Publicar en cola (descartar viejo si está llena)
                try:
                    self._queue.put_nowait(frame)
                except queue.Full:
                    try:
                        self._queue.get_nowait()
                        self._queue.put_nowait(frame)
                    except queue.Empty:
                        pass

            except Exception as e:
                # No crashear el hilo por errores transitorios de captura
                time.sleep(0.01)

    # ------------------------------------------------------------------
    # Parseo de salida del NPU
    # ------------------------------------------------------------------
    def _parse_npu_output(
        self, metadata: dict, img_shape: tuple
    ) -> list[Detection]:
        """
        Extrae detecciones del tensor de salida del IMX500.

        EfficientDet Lite0 _pp exporta 4 tensores (formato TensorFlow/COCO):
          [0] boxes   : (1, N, 4)  [ymin, xmin, ymax, xmax] normalizados 0-1
          [1] classes : (1, N)     class ID (float)
          [2] scores  : (1, N)     confianza
          [3] count   : (1,)       número de detecciones válidas

        NOTA: el orden boxes/classes/scores varía según el modelo compilado.
        El código intenta el formato EfficientDet y cae al formato YOLOv8
        si el primero falla, para ser robusto ante modelos alternativos.
        """
        np_outputs = self._imx500.get_outputs(metadata, add_batch=True)
        if np_outputs is None:
            return []

        ih, iw = img_shape[:2]

        try:
            # ── Formato EfficientDet (4 outputs): boxes, classes, scores, count ──
            if len(np_outputs) >= 4:
                boxes   = np_outputs[0][0]         # (N, 4) [y1,x1,y2,x2] norm
                classes = np_outputs[1][0]         # (N,)
                scores  = np_outputs[2][0]         # (N,)
                count   = int(np_outputs[3][0])    # número real de detecciones
                tf_format = True                   # coordenadas en [y,x,y,x]
            else:
                # ── Formato YOLOv8 / NanoDet (3 outputs): boxes, scores, classes ──
                boxes   = np_outputs[0][0]
                scores  = np_outputs[1][0]
                classes = np_outputs[2][0]
                count   = len(scores)
                tf_format = False
        except (IndexError, TypeError):
            return []

        detections: list[Detection] = []
        for i in range(min(count, len(scores))):
            score  = float(scores[i])
            cls_id = int(classes[i])

            if score < DETECTION_CONFIDENCE:
                continue

            label = self._class_map.get(cls_id, f"cls_{cls_id}")
            if label not in CLASSES_OF_INTEREST.values():
                continue

            box = boxes[i]
            if tf_format:
                # [ymin, xmin, ymax, xmax] normalizados → píxeles
                y1 = int(box[0] * ih)
                x1 = int(box[1] * iw)
                y2 = int(box[2] * ih)
                x2 = int(box[3] * iw)
            else:
                # [xmin, ymin, xmax, ymax] normalizados → píxeles
                x1 = int(box[0] * iw)
                y1 = int(box[1] * ih)
                x2 = int(box[2] * iw)
                y2 = int(box[3] * ih)

            # Sanitizar
            x1, x2 = sorted([max(0, x1), min(iw - 1, x2)])
            y1, y2 = sorted([max(0, y1), min(ih - 1, y2)])

            detections.append(Detection(
                label=label,
                class_id=cls_id,
                confidence=score,
                x1=x1, y1=y1, x2=x2, y2=y2,
            ))

        return detections

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _build_class_map(self):
        """
        Construye el mapa class_id → nombre usando los intrínsecos del
        modelo cargado en el IMX500, y luego filtra con CLASSES_OF_INTEREST.
        """
        try:
            intrinsics = self._imx500.network_intrinsics
            if intrinsics and intrinsics.labels:
                for idx, raw_label in enumerate(intrinsics.labels):
                    label_lower = raw_label.strip().lower()
                    for known, friendly in CLASSES_OF_INTEREST.items():
                        if known in label_lower:
                            self._class_map[idx] = friendly
                            break
        except Exception:
            # Si el modelo no expone etiquetas, se usa class_id como clave
            pass

    @property
    def resolution(self) -> tuple[int, int]:
        return CAMERA_WIDTH, CAMERA_HEIGHT
