# -*- coding: utf-8 -*-
"""
vision_module.py — Módulo de visión de nivel producción para TMR 2026.

Características principales
----------------------------
• Sony IMX500 AI Camera — YOLO11n en NPU on-chip (cero inferencia en CPU)
• Resolución stream: 2028 × 1520 | NPU interno: 640 × 640
• 30 FPS fijos con FrameDurationLimits=(33333,33333)
• AE/AWB auto → bloqueado tras 1.5 s de estabilización
• Overlay de bboxes vía pre_callback (MappedArray) — sin copia extra de frame
• Hilo de captura desacoplado del bucle de control (VisionState thread-safe)
• Fusión de distancia: LiDAR (prioridad) + pin-hole con histéresis 3 frames
• FSM de frenado: IDLE→DETECTION→APPROACHING→BRAKING_PID→STOPPED(5 s)→RESUMING
• Auto-reconexión tras 3 fallos consecutivos de captura
• Vigilante de FPS: avisa si < 20 FPS durante > 2 s
• Logger "tmr.vision"

Instalación (Raspberry Pi OS)
------------------------------
  sudo apt install imx500-all python3-picamera2
  pip install opencv-python-headless numpy

Uso rápido
----------
  vm = VisionModule()
  vm.start()
  state = vm.get_state()     # dict-like, siempre fresco, nunca bloquea
  vm.stop()
"""

from __future__ import annotations

import copy
import logging
import threading
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional

import cv2
import numpy as np

log = logging.getLogger("tmr.vision")

# ─────────────────────────────────────────────────────────────────────────────
# Constantes internas
# ─────────────────────────────────────────────────────────────────────────────
_YOLO11N_MODEL   = "/usr/share/imx500-models/imx500_network_yolo11n_pp.rpk"
_STREAM_W        = 2028
_STREAM_H        = 1520
_FPS             = 30

# Longitud focal estimada para IMX500 a 2028 px de ancho (~75° FOV diagonal)
_FOCAL_LENGTH_PX = 1562.0

# Clases COCO de interés → etiqueta interna TMR
_COCO_CLASSES: dict[str, str] = {
    "stop sign":     "STOP",
    "traffic light": "SEMAFORO",
    "person":        "PERSONA",
    "car":           "AUTO",
}

# AE / AWB
_AWB_WARMUP_S = 1.5   # segundos de estabilización antes de bloquear

# Filtro temporal
_CONF_THRESHOLD  = 0.28   # umbral mínimo de confianza del NPU
_TEMPORAL_FRAMES = 2      # frames consecutivos para confirmar una detección

# Distancia STOP
_STOP_REAL_H_M  = 0.18   # altura real de la señal STOP del TMR (metros)
_DIST_HIST_LEN  = 3      # frames de histéresis para fusión de distancia

# FSM de frenado
_BRAKE_START_MM = 700    # empieza a frenar (mm)
_BRAKE_PID_MM   = 350    # activa PID de precisión (mm)
_STOP_TARGET_MM = 270    # objetivo de parada ≤ 30 cm (mm)
_STOP_TOL_MM    = 30     # ventana de aceptación (mm)
_STOP_WAIT_S    = 5.0    # espera exacta en STOPPED (sin sleep, sin deriva)

# Vigilante FPS
_FPS_WARN_THRESH = 20.0  # FPS mínimo aceptable
_FPS_WARN_DUR_S  =  2.0  # segundos bajo umbral antes de advertir

# Reconexión automática
_MAX_CONSEC_FAIL = 3     # fallos consecutivos antes de reconectar


# ─────────────────────────────────────────────────────────────────────────────
# Enums
# ─────────────────────────────────────────────────────────────────────────────

class BrakeFSMState(Enum):
    IDLE        = auto()   # sin señal STOP visible
    DETECTION   = auto()   # STOP detectado, fuera de rango de frenado
    APPROACHING = auto()   # dentro del rango, frenado lineal
    BRAKING_PID = auto()   # PID de precisión, últimos centímetros
    STOPPED     = auto()   # detenido, contando 5 s exactos
    RESUMING    = auto()   # ramp de aceleración gradual


# ─────────────────────────────────────────────────────────────────────────────
# Dataclasses públicas
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class DetectionResult:
    """Una detección confirmada del NPU (post-filtro temporal)."""
    label:      str
    class_id:   int
    confidence: float
    x1: int
    y1: int
    x2: int
    y2: int

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


@dataclass
class VisionState:
    """
    Estado semántico completo del módulo de visión.

    Producido cada frame y publicado de forma atómica.
    No mutar después de publicar — tratar como inmutable.
    """
    # ── Señal STOP ───────────────────────────────────────────────
    stop_detected:    bool              = False
    stop_distance_mm: Optional[float]  = None
    stop_bbox:        Optional[tuple]  = None   # (x1, y1, x2, y2) px

    # ── Semáforo ─────────────────────────────────────────────────
    traffic_light_color: str   = "unknown"  # "red"|"yellow"|"green"|"unknown"
    traffic_light_conf:  float = 0.0

    # ── Otros objetos ────────────────────────────────────────────
    person_detected: bool             = False
    car_detected:    bool             = False
    car_bbox:        Optional[tuple]  = None
    car_in_lane:     bool             = False   # True → obstáculo en nuestro carril
    car_in_park_zone: bool            = False   # True → auto en zona lateral derecha

    # ── FSM de frenado ───────────────────────────────────────────
    brake_state:     BrakeFSMState = BrakeFSMState.IDLE
    brake_speed_pct: float         = 100.0  # velocidad sugerida [0–100 %]

    # ── Meta ─────────────────────────────────────────────────────
    fps:             float = 0.0
    frame_id:        int   = 0
    timestamp:       float = field(default_factory=time.monotonic)
    raw_detections:  list  = field(default_factory=list)


# ─────────────────────────────────────────────────────────────────────────────
# Clase principal
# ─────────────────────────────────────────────────────────────────────────────

class VisionModule:
    """
    Módulo de visión TMR 2026 basado en Sony IMX500 + YOLO11n NPU.

    El hilo interno captura frames y actualiza `VisionState` de forma
    atómica.  El bucle de control externo llama a `get_state()` sin
    bloquearse ni esperar sincronización.

    Ejemplo::

        vm = VisionModule()
        vm.start()
        while running:
            s = vm.get_state()
            motor.set_throttle(s.brake_speed_pct * max_pwm / 100)
        vm.stop()
    """

    def __init__(
        self,
        model_path: str = _YOLO11N_MODEL,
        display_overlay: bool = False,
    ) -> None:
        """
        Parameters
        ----------
        model_path : str
            Ruta al archivo .rpk del modelo YOLO11n compilado para IMX500.
        display_overlay : bool
            Si True activa el pre_callback que dibuja bboxes sobre el stream
            vía MappedArray (acceso directo al buffer DMA, sin copia).
        """
        self._model_path      = model_path
        self._display_overlay = display_overlay

        # Objetos de Picamera2 — se crean en start() / _init_camera()
        self._imx500 = None
        self._picam2 = None

        # Estado publicado (reemplazado atómicamente, nunca mutado in-place)
        self._state      = VisionState()
        self._state_lock = threading.Lock()

        # Frame BGR más reciente (para lane detector externo)
        self._latest_frame: Optional[np.ndarray] = None
        self._frame_lock   = threading.Lock()

        # Hilo de captura
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None

        # Filtro temporal: frames consecutivos confirmados por etiqueta
        self._consec: dict[str, int] = {v: 0 for v in _COCO_CLASSES.values()}

        # Histéresis de distancia: últimas N estimaciones válidas
        self._dist_hist: list[float] = []

        # FSM de frenado (estado interno mutable solo en el hilo de captura)
        self._fsm_state      = BrakeFSMState.IDLE
        self._stopped_since  = 0.0
        self._resume_start   = 0.0

        # Mapa class_id (int) → etiqueta TMR (str)
        self._class_map: dict[int, str] = {}

        # Vigilante de FPS
        self._fps_ts: list[float] = []   # timestamps del último segundo
        self._low_fps_since = 0.0

        # Contador de fallos de captura
        self._capture_fails = 0

        log.info("VisionModule creado — modelo: %s", model_path)

    # ─────────────────────────────────────────────────────────────
    # Ciclo de vida
    # ─────────────────────────────────────────────────────────────

    def start(self) -> None:
        """Inicializa la cámara y arranca el hilo de captura."""
        self._init_camera()
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._capture_loop,
            name="VisionModule-Capture",
            daemon=True,
        )
        self._thread.start()
        log.info("VisionModule iniciado — hilo de captura activo.")

    def stop(self) -> None:
        """Detiene el hilo de captura y libera la cámara."""
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=4.0)
        if self._picam2 is not None:
            try:
                self._picam2.stop()
            except Exception:
                pass
        log.info("VisionModule detenido.")

    # ─────────────────────────────────────────────────────────────
    # API pública (thread-safe)
    # ─────────────────────────────────────────────────────────────

    def get_state(self) -> VisionState:
        """
        Retorna el estado semántico más reciente.

        Nunca bloquea más de unos microsegundos.  El objeto retornado
        es la referencia al último VisionState publicado; no se muta
        después de publicarse, por lo que es seguro leerlo sin lock
        adicional.
        """
        with self._state_lock:
            return self._state

    def get_latest_frame(self) -> Optional[np.ndarray]:
        """
        Retorna el frame BGR más reciente capturado por el NPU.

        Thread-safe, no bloqueante.  Retorna None hasta que el primer frame
        esté disponible (~2 s tras start() mientras estabiliza AE/AWB).
        Usar este frame para el detector de carril (lane_detector.process).
        """
        with self._frame_lock:
            return self._latest_frame

    def recalibrate_lighting(self) -> None:
        """
        Re-calibra AE/AWB ante cambios de iluminación durante la competencia.

        Desbloquea AE/AWB, espera `_AWB_WARMUP_S` para estabilización y
        vuelve a bloquear con los nuevos valores.  Puede llamarse desde
        cualquier hilo; bloquea al invocador durante ~1.5 s.
        """
        if self._picam2 is None:
            log.warning("recalibrate_lighting: cámara no iniciada.")
            return
        log.info("Recalibrando AE/AWB...")
        self._picam2.set_controls({"AeEnable": True, "AwbEnable": True})
        time.sleep(_AWB_WARMUP_S)
        self._lock_ae_awb()
        log.info("Recalibración completada.")

    def estimate_distance_fused(
        self,
        bbox: Optional[tuple],
        lidar_mm: Optional[float] = None,
    ) -> Optional[float]:
        """
        Estima la distancia a la señal STOP fusionando LiDAR y pin-hole.

        Prioridad de fuentes
        --------------------
        1. LiDAR si lectura válida (0 < lidar_mm < 1200).
        2. Pin-hole: d_mm = (h_real_m × focal_px / h_px) × 1000.
        3. Mediana de las últimas `_DIST_HIST_LEN` estimaciones válidas
           (histéresis para frames sin detección puntual).

        Parameters
        ----------
        bbox : tuple (x1, y1, x2, y2) del bbox de la señal, o None.
        lidar_mm : lectura VL53L0X en mm, o None si no disponible.

        Returns
        -------
        Distancia estimada en mm, o None si no hay datos suficientes.
        """
        estimate: Optional[float] = None

        if lidar_mm is not None and 0 < lidar_mm < 1200:
            estimate = float(lidar_mm)
        elif bbox is not None:
            h_px = bbox[3] - bbox[1]
            if h_px >= 5:
                dist_m = (_STOP_REAL_H_M * _FOCAL_LENGTH_PX) / h_px
                estimate = dist_m * 1000.0

        if estimate is not None:
            self._dist_hist.append(estimate)
            if len(self._dist_hist) > _DIST_HIST_LEN:
                self._dist_hist.pop(0)

        if estimate is None and self._dist_hist:
            estimate = float(np.median(self._dist_hist))

        return estimate

    # ─────────────────────────────────────────────────────────────
    # Inicialización de cámara
    # ─────────────────────────────────────────────────────────────

    def _init_camera(self) -> None:
        """Construye y configura Picamera2 + IMX500.  Bloquea hasta estar lista."""
        from picamera2 import Picamera2
        from picamera2.devices.imx500 import IMX500

        self._imx500 = IMX500(self._model_path)
        self._picam2 = Picamera2(self._imx500.camera_num)

        cfg = self._picam2.create_preview_configuration(
            main={
                "format": "BGR888",                     # OpenCV nativo — sin cvtColor
                "size":   (_STREAM_W, _STREAM_H),
            },
            controls={
                "FrameDurationLimits": (33333, 33333),  # 30 FPS fijo
                "AeEnable":           True,
                "AwbEnable":          True,
                "AwbMode":            4,                # Indoor: corrige tono azulado
                "Contrast":           1.5,
                "Saturation":         1.8,
                "Sharpness":          4.0,
                "NoiseReductionMode": 2,                # CDN_Fast
            },
            buffer_count=4,
        )
        self._picam2.configure(cfg)

        # Overlay de bboxes sobre buffer DMA (sin copia de frame)
        if self._display_overlay:
            self._picam2.pre_callback = self._overlay_callback

        self._picam2.start()

        # Esperar estabilización de AE/AWB
        log.info("Estabilizando AE/AWB (%.1f s)...", _AWB_WARMUP_S)
        time.sleep(_AWB_WARMUP_S)
        self._lock_ae_awb()

        # Autofoco continuo (solo en módulos con actuador VCM)
        try:
            self._picam2.set_controls({"AfMode": 2, "AfSpeed": 1})
            log.info("Autoenfoque continuo activado.")
        except Exception:
            log.debug("Autoenfoque no disponible — enfoque fijo.")

        self._build_class_map()
        log.info("Cámara lista: %d×%d @ %d FPS", _STREAM_W, _STREAM_H, _FPS)

    def _lock_ae_awb(self) -> None:
        """
        Lee los valores actuales de exposición y balance de blancos
        del metadato de la cámara y los fija para eliminar parpadeo.
        """
        try:
            meta  = self._picam2.capture_metadata()
            exp   = meta.get("ExposureTime")
            gain  = meta.get("AnalogueGain")
            cgains = meta.get("ColourGains")

            controls: dict = {"AeEnable": False}
            if exp   is not None: controls["ExposureTime"] = exp
            if gain  is not None: controls["AnalogueGain"] = gain
            if cgains is not None:
                controls["AwbEnable"]   = False
                controls["ColourGains"] = tuple(cgains)

            self._picam2.set_controls(controls)
            log.info(
                "AE/AWB bloqueados — exp=%s µs  gain=%.2f  cgains=%s",
                exp, gain or 0.0, cgains,
            )
        except Exception as exc:
            log.warning("No se pudo bloquear AE/AWB: %s", exc)

    def _build_class_map(self) -> None:
        """
        Construye el mapa {class_id → etiqueta TMR} usando los intrínsecos
        del modelo cargado en el IMX500.  Si el modelo no expone etiquetas
        el mapa queda vacío y las detecciones se descartan.
        """
        try:
            intrinsics = self._imx500.network_intrinsics
            if intrinsics and intrinsics.labels:
                for idx, raw_label in enumerate(intrinsics.labels):
                    lower = raw_label.strip().lower()
                    for coco_name, friendly in _COCO_CLASSES.items():
                        if coco_name in lower:
                            self._class_map[idx] = friendly
                            break
            log.debug("Mapa de clases: %s", self._class_map)
        except Exception as exc:
            log.warning("No se pudo construir mapa de clases: %s", exc)

    # ─────────────────────────────────────────────────────────────
    # Hilo de captura
    # ─────────────────────────────────────────────────────────────

    def _capture_loop(self) -> None:
        """
        Bucle principal del hilo de captura.

        Por cada frame:
          1. Captura request (frame BGR + metadatos NPU).
          2. Parsea tensores de salida del IMX500.
          3. Aplica filtro temporal por etiqueta.
          4. Genera VisionState semántico.
          5. Actualiza vigilante de FPS.
          6. Publica estado de forma atómica.
        """
        while not self._stop_event.is_set():
            try:
                request = self._picam2.capture_request()
                frame   = request.make_array("main")   # BGR en memoria
                meta    = request.get_metadata()
                request.release()
            except Exception as exc:
                self._capture_fails += 1
                log.warning(
                    "Error de captura (%d/%d): %s",
                    self._capture_fails, _MAX_CONSEC_FAIL, exc,
                )
                if self._capture_fails >= _MAX_CONSEC_FAIL:
                    log.error("Demasiados fallos — reconectando cámara...")
                    self._reconnect()
                time.sleep(0.05)
                continue

            self._capture_fails = 0

            # Publicar frame para el lane detector externo
            with self._frame_lock:
                self._latest_frame = frame

            raw_dets   = self._parse_npu(meta, frame.shape)
            conf_dets  = self._apply_temporal_filter(raw_dets)
            new_state  = self._analyze(conf_dets, frame)
            self._update_fps_watchdog(new_state)

            with self._state_lock:
                self._state = new_state

    # ─────────────────────────────────────────────────────────────
    # Reconexión automática
    # ─────────────────────────────────────────────────────────────

    def _reconnect(self) -> None:
        """Para y reinicia la cámara tras fallos persistentes de captura."""
        try:
            self._picam2.stop()
        except Exception:
            pass
        time.sleep(1.0)
        try:
            self._init_camera()
            self._capture_fails = 0
            log.info("Reconexión de cámara exitosa.")
        except Exception as exc:
            log.error("Reconexión fallida: %s", exc)

    # ─────────────────────────────────────────────────────────────
    # Parseo de salida NPU (YOLO11n _pp / EfficientDet)
    # ─────────────────────────────────────────────────────────────

    def _parse_npu(
        self, metadata: dict, img_shape: tuple
    ) -> list[DetectionResult]:
        """
        Extrae detecciones del tensor de salida del IMX500.

        Soporta dos formatos de salida:

        YOLO _pp (3 outputs)
          [0] boxes   (1, N, 4)  [x1, y1, x2, y2] normalizados [0–1]
          [1] scores  (1, N)     confianza
          [2] classes (1, N)     class_id (float)

        EfficientDet / TF _pp (4 outputs)
          [0] boxes   (1, N, 4)  [ymin, xmin, ymax, xmax] normalizados [0–1]
          [1] classes (1, N)
          [2] scores  (1, N)
          [3] count   (1,)
        """
        np_outputs = self._imx500.get_outputs(metadata, add_batch=True)
        if np_outputs is None:
            return []

        ih, iw = img_shape[:2]

        try:
            if len(np_outputs) >= 4:
                # ── Formato EfficientDet / TF ──────────────────────
                boxes   = np_outputs[0][0]       # (N, 4)
                classes = np_outputs[1][0]       # (N,)
                scores  = np_outputs[2][0]       # (N,)
                count   = int(np_outputs[3][0])
                tf_fmt  = True
            else:
                # ── Formato YOLO11n ────────────────────────────────
                boxes   = np_outputs[0][0]       # (N, 4)
                scores  = np_outputs[1][0]       # (N,)
                classes = np_outputs[2][0]       # (N,)
                count   = len(scores)
                tf_fmt  = False
        except (IndexError, TypeError):
            return []

        results: list[DetectionResult] = []
        for i in range(min(count, len(scores))):
            score  = float(scores[i])
            cls_id = int(classes[i])

            if score < _CONF_THRESHOLD:
                continue

            label = self._class_map.get(cls_id)
            if label is None:
                continue

            box = boxes[i]
            if tf_fmt:
                y1 = int(box[0] * ih); x1 = int(box[1] * iw)
                y2 = int(box[2] * ih); x2 = int(box[3] * iw)
            else:
                x1 = int(box[0] * iw); y1 = int(box[1] * ih)
                x2 = int(box[2] * iw); y2 = int(box[3] * ih)

            x1, x2 = sorted([max(0, x1), min(iw - 1, x2)])
            y1, y2 = sorted([max(0, y1), min(ih - 1, y2)])

            results.append(DetectionResult(
                label=label, class_id=cls_id, confidence=score,
                x1=x1, y1=y1, x2=x2, y2=y2,
            ))

        return results

    # ─────────────────────────────────────────────────────────────
    # Filtro temporal
    # ─────────────────────────────────────────────────────────────

    def _apply_temporal_filter(
        self, detections: list[DetectionResult]
    ) -> list[DetectionResult]:
        """
        Confirma únicamente detecciones presentes en ≥ _TEMPORAL_FRAMES
        frames consecutivos.  Elimina falsos positivos de un solo frame.
        """
        seen = {d.label for d in detections}
        for label in self._consec:
            if label in seen:
                self._consec[label] = min(
                    self._consec[label] + 1, _TEMPORAL_FRAMES + 2
                )
            else:
                self._consec[label] = 0

        return [d for d in detections
                if self._consec[d.label] >= _TEMPORAL_FRAMES]

    # ─────────────────────────────────────────────────────────────
    # Análisis semántico
    # ─────────────────────────────────────────────────────────────

    def _analyze(
        self, detections: list[DetectionResult], frame: np.ndarray
    ) -> VisionState:
        """
        Convierte la lista de detecciones confirmadas en un VisionState
        semántico y ejecuta un paso de la FSM de frenado.
        """
        state = VisionState(raw_detections=detections)

        W, H = frame.shape[1], frame.shape[0]
        frame_cx = W / 2

        for det in detections:
            if det.label == "STOP":
                if self._has_red_color(frame, det):
                    state.stop_detected = True
                    state.stop_bbox     = (det.x1, det.y1, det.x2, det.y2)

            elif det.label == "SEMAFORO":
                color, conf = self._classify_traffic_light(frame, det)
                if conf > state.traffic_light_conf:
                    state.traffic_light_color = color
                    state.traffic_light_conf  = conf

            elif det.label == "PERSONA":
                state.person_detected = True

            elif det.label == "AUTO":
                bbox = (det.x1, det.y1, det.x2, det.y2)
                if state.car_bbox is None:
                    state.car_bbox = bbox
                else:
                    prev_area = (state.car_bbox[2] - state.car_bbox[0]) * \
                                (state.car_bbox[3] - state.car_bbox[1])
                    if det.area > prev_area:
                        state.car_bbox = bbox

        # ── Clasificar posición del auto ──────────────────────────
        if state.car_bbox is not None:
            x1, y1, x2, y2 = state.car_bbox
            cx   = (x1 + x2) / 2
            area = (x2 - x1) * (y2 - y1)
            state.car_detected   = True
            state.car_in_lane    = (
                abs(cx - frame_cx) < W * 0.35
                and area >= 2500
                and y2 >= int(H * 0.60)
            )
            state.car_in_park_zone = cx > W * 0.55

        # ── Distancia fusionada al STOP ────────────────────────────
        if state.stop_detected:
            state.stop_distance_mm = self.estimate_distance_fused(state.stop_bbox)

        # ── FSM de frenado ─────────────────────────────────────────
        state.brake_state     = self._fsm_state
        state.brake_speed_pct = self._step_brake_fsm(state)

        return state

    # ─────────────────────────────────────────────────────────────
    # FSM de frenado
    # ─────────────────────────────────────────────────────────────

    def _step_brake_fsm(self, state: VisionState) -> float:
        """
        Ejecuta un paso de la FSM de frenado y devuelve la velocidad
        sugerida en porcentaje [0–100].

        El controlador externo (AutonomousController) puede usar este
        valor directamente o pasarlo por su propio PID de velocidad.

        Transiciones
        ------------
        IDLE        → DETECTION   : señal STOP aparece en frame
        DETECTION   → APPROACHING : dist < _BRAKE_START_MM
        APPROACHING → BRAKING_PID : dist < _BRAKE_PID_MM
        BRAKING_PID → STOPPED     : |dist − target| ≤ tolerancia
        STOPPED     → RESUMING    : timer exacto ≥ _STOP_WAIT_S (no-sleep)
        RESUMING    → IDLE        : ramp 2 s completada
        cualquiera  → IDLE        : señal desaparece (salvo STOPPED/RESUMING)
        """
        dist_mm = state.stop_distance_mm
        now     = time.monotonic()

        # Pérdida de señal fuera de estados de parada
        if not state.stop_detected and self._fsm_state not in (
            BrakeFSMState.IDLE, BrakeFSMState.STOPPED, BrakeFSMState.RESUMING
        ):
            log.debug("STOP perdido en estado %s → IDLE", self._fsm_state.name)
            self._fsm_state = BrakeFSMState.IDLE
            self._dist_hist.clear()

        match self._fsm_state:

            case BrakeFSMState.IDLE:
                if state.stop_detected:
                    self._fsm_state = BrakeFSMState.DETECTION
                    log.info("FSM IDLE→DETECTION  dist=%s mm",
                             f"{dist_mm:.0f}" if dist_mm else "?")
                return 100.0

            case BrakeFSMState.DETECTION:
                if dist_mm is not None and dist_mm < _BRAKE_START_MM:
                    self._fsm_state = BrakeFSMState.APPROACHING
                    log.info("FSM DETECTION→APPROACHING  dist=%.0f mm", dist_mm)
                return 100.0

            case BrakeFSMState.APPROACHING:
                if dist_mm is None:
                    return 25.0   # sin distancia → precaución
                if dist_mm < _BRAKE_PID_MM:
                    self._fsm_state = BrakeFSMState.BRAKING_PID
                    log.info("FSM APPROACHING→BRAKING_PID  dist=%.0f mm", dist_mm)
                # Rampa lineal: 100 % en BRAKE_START → 20 % en BRAKE_PID
                span  = _BRAKE_START_MM - _BRAKE_PID_MM
                ratio = max(0.0, (dist_mm - _BRAKE_PID_MM) / span)
                return max(20.0, ratio * 100.0)

            case BrakeFSMState.BRAKING_PID:
                if dist_mm is None:
                    return 8.0
                error = dist_mm - _STOP_TARGET_MM
                if abs(error) <= _STOP_TOL_MM:
                    self._fsm_state    = BrakeFSMState.STOPPED
                    self._stopped_since = now
                    log.info("FSM BRAKING_PID→STOPPED  dist=%.0f mm", dist_mm)
                    return 0.0
                # Proporcional simple — el PID externo refina la señal
                speed = max(5.0, min(error * 0.035, 20.0))
                return speed

            case BrakeFSMState.STOPPED:
                elapsed = now - self._stopped_since
                if elapsed >= _STOP_WAIT_S:
                    self._fsm_state   = BrakeFSMState.RESUMING
                    self._resume_start = now
                    log.info("FSM STOPPED→RESUMING  espera=%.2f s", elapsed)
                return 0.0   # detenido — 0 % PWM

            case BrakeFSMState.RESUMING:
                RAMP_S = 2.0   # segundos para llegar a velocidad normal
                t      = min((now - self._resume_start) / RAMP_S, 1.0)
                if t >= 1.0:
                    self._fsm_state = BrakeFSMState.IDLE
                    self._dist_hist.clear()
                    log.info("FSM RESUMING→IDLE")
                return t * 100.0

            case _:
                return 100.0

    # ─────────────────────────────────────────────────────────────
    # Verificación de color rojo en señal STOP
    # ─────────────────────────────────────────────────────────────

    def _has_red_color(self, frame: np.ndarray, det: DetectionResult) -> bool:
        """
        Verifica que el ROI del bbox contiene suficiente rojo (HSV).

        Filtra falsos positivos donde el modelo detecta objetos no rojos.
        Retorna True si ≥ 6 % de los píxeles del ROI son rojos.

        Nota: frame es BGR en memoria (Picamera2 format='RGB888').
        """
        pad = 4
        x1 = max(0, det.x1 + pad);          y1 = max(0, det.y1 + pad)
        x2 = min(frame.shape[1] - 1, det.x2 - pad)
        y2 = min(frame.shape[0] - 1, det.y2 - pad)
        if x2 <= x1 or y2 <= y1:
            return True   # bbox inválido → aceptar de todas formas

        roi = frame[y1:y2, x1:x2]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Rojo ocupa dos rangos en HSV (0–10° y 160–180°)
        mask1 = cv2.inRange(hsv, np.array([0,   80, 80]), np.array([10,  255, 255]))
        mask2 = cv2.inRange(hsv, np.array([155, 80, 80]), np.array([180, 255, 255]))
        red_ratio = (np.sum(mask1 > 0) + np.sum(mask2 > 0)) / roi.size
        return red_ratio >= 0.06

    # ─────────────────────────────────────────────────────────────
    # Clasificación de color de semáforo
    # ─────────────────────────────────────────────────────────────

    def _classify_traffic_light(
        self, frame: np.ndarray, det: DetectionResult
    ) -> tuple[str, float]:
        """
        Clasifica el color del semáforo usando máscaras HSV en el ROI.

        Estrategia
        ----------
        1. Recortar ROI con margen de 4 px.
        2. Convertir a HSV.
        3. Calcular máscara para rojo, amarillo y verde.
        4. Ganar el color con mayor número de píxeles detectados.
        5. Descartar si la confianza < 5 % (color fantasma).

        Returns
        -------
        (color_str, confidence_float) donde color_str es
        "red" | "yellow" | "green" | "unknown".
        """
        pad = 4
        x1 = max(0, det.x1 + pad);          y1 = max(0, det.y1 + pad)
        x2 = min(frame.shape[1] - 1, det.x2 - pad)
        y2 = min(frame.shape[0] - 1, det.y2 - pad)
        if x2 <= x1 or y2 <= y1:
            return "unknown", 0.0

        roi = frame[y1:y2, x1:x2]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        ranges: dict[str, list[tuple]] = {
            "red":    [((0, 100, 100), (10, 255, 255)),
                       ((160, 100, 100), (180, 255, 255))],
            "yellow": [((20, 100, 100), (35, 255, 255))],
            "green":  [((45,  60,  60), (85, 255, 255))],
        }
        scores: dict[str, int] = {}
        for color, rlist in ranges.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lo, hi in rlist:
                mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
            scores[color] = int(np.sum(mask > 0))

        best  = max(scores, key=scores.get)
        total = roi.shape[0] * roi.shape[1]
        conf  = scores[best] / total if total > 0 else 0.0

        if conf < 0.05:
            return "unknown", 0.0
        return best, conf

    # ─────────────────────────────────────────────────────────────
    # Overlay pre_callback (MappedArray — sin copia)
    # ─────────────────────────────────────────────────────────────

    def _overlay_callback(self, request) -> None:
        """
        Dibuja bboxes directamente sobre el buffer DMA del frame usando
        MappedArray.  Invocado por Picamera2 antes de entregar el request.
        No hace copias del frame — acceso directo a la memoria de la cámara.
        """
        from picamera2 import MappedArray

        _COLOR: dict[str, tuple] = {
            "STOP":     (0,   0,   255),
            "SEMAFORO": (0,   200, 255),
            "PERSONA":  (255, 100,   0),
            "AUTO":     (200,   0, 200),
        }

        with self._state_lock:
            dets = list(self._state.raw_detections)

        with MappedArray(request, "main") as m:
            for det in dets:
                color = _COLOR.get(det.label, (180, 180, 180))
                cv2.rectangle(m.array,
                              (det.x1, det.y1), (det.x2, det.y2),
                              color, 2)
                cv2.putText(
                    m.array,
                    f"{det.label} {det.confidence:.0%}",
                    (det.x1, max(det.y1 - 6, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA,
                )

    # ─────────────────────────────────────────────────────────────
    # Vigilante de FPS
    # ─────────────────────────────────────────────────────────────

    def _update_fps_watchdog(self, state: VisionState) -> None:
        """
        Calcula FPS con ventana deslizante de 1 s y actualiza state.fps.
        Emite WARNING si el FPS cae bajo _FPS_WARN_THRESH durante > 2 s.
        """
        now = time.monotonic()
        self._fps_ts.append(now)

        # Descartar timestamps fuera de la ventana de 1 s
        cutoff = now - 1.0
        while self._fps_ts and self._fps_ts[0] < cutoff:
            self._fps_ts.pop(0)

        state.fps      = float(len(self._fps_ts))
        state.frame_id = self._state.frame_id + 1
        state.timestamp = now

        # Watchdog
        if state.fps < _FPS_WARN_THRESH:
            if self._low_fps_since == 0.0:
                self._low_fps_since = now
            elif now - self._low_fps_since > _FPS_WARN_DUR_S:
                log.warning(
                    "FPS bajo sostenido: %.1f (umbral %.0f FPS, durante %.1f s)",
                    state.fps, _FPS_WARN_THRESH, now - self._low_fps_since,
                )
                self._low_fps_since = now   # reiniciar para evitar spam
        else:
            self._low_fps_since = 0.0


# ─────────────────────────────────────────────────────────────────────────────
# Demo / prueba directa
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import sys

    logging.basicConfig(
        level=logging.DEBUG,
        format="%(asctime)s [%(levelname)8s] %(name)s — %(message)s",
        datefmt="%H:%M:%S",
    )

    print("=" * 60)
    print("  VisionModule TMR 2026 — Demo")
    print("  Ctrl+C para salir")
    print("=" * 60)

    vm = VisionModule(display_overlay=False)
    vm.start()

    try:
        while True:
            s = vm.get_state()

            parts = [
                f"FPS:{s.fps:4.1f}",
                f"#:{s.frame_id:6d}",
                f"FSM:{s.brake_state.name:<11}",
                f"vel:{s.brake_speed_pct:5.1f}%",
            ]
            if s.stop_detected:
                d = f"{s.stop_distance_mm:.0f}" if s.stop_distance_mm else "?"
                parts.append(f"STOP:{d}mm")
            if s.traffic_light_color != "unknown":
                parts.append(f"LUZ:{s.traffic_light_color.upper()}"
                             f"({s.traffic_light_conf:.0%})")
            if s.car_detected:
                tag = "[CARRIL]" if s.car_in_lane else "[lateral]"
                parts.append(f"AUTO{tag}")
            if s.person_detected:
                parts.append("PERSONA")

            print("\r" + "  ".join(parts) + "   ", end="", flush=True)
            time.sleep(0.10)

    except KeyboardInterrupt:
        print("\nDeteniendo...")
    finally:
        vm.stop()
        print("Listo.")

# ─────────────────────────────────────────────────────────────────────────────
# Dependencias
# ─────────────────────────────────────────────────────────────────────────────
# sudo apt install imx500-all python3-picamera2
# pip install opencv-python-headless numpy
