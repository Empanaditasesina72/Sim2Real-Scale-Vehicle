# -*- coding: utf-8 -*-
"""
camera_stream.py — Hilo de captura Pi AI Camera (Picamera2) para TMR 2026.

Características:
  • Formato RGB888 → conversión obligatoria cv2.COLOR_RGB2BGR para OpenCV.
  • Bloqueo de AE/AWB tras período de estabilización (elimina parpadeo).
  • Hilo demonio: main loop nunca espera al frame anterior.
  • Resolución y FPS configurables.
"""

import threading
import time
from typing import Optional

import cv2
import numpy as np


class CameraStream:
    """
    Captura frames de la Pi AI Camera en un hilo separado.

    Uso::

        cam = CameraStream(width=640, height=480, fps=30)
        cam.start()
        frame = cam.get_frame()   # BGR, listo para OpenCV
        cam.stop()
    """

    def __init__(
        self,
        width:        int   = 640,
        height:       int   = 480,
        fps:          int   = 30,
        awb_warmup_s: float = 2.0,
    ):
        self._w          = width
        self._h          = height
        self._fps        = fps
        self._warmup_s   = awb_warmup_s

        self._frame: Optional[np.ndarray] = None
        self._lock  = threading.Lock()
        self._stop  = threading.Event()
        self._ready = threading.Event()   # se activa tras el primer frame

        # Picamera2 — importar aquí para no fallar en PC de desarrollo
        from picamera2 import Picamera2
        self._picam2 = Picamera2()

        cfg = self._picam2.create_preview_configuration(
            main={
                "format": "RGB888",          # RGB — se convierte a BGR en captura
                "size":   (width, height),
            },
            controls={
                "FrameDurationLimits": (1_000_000 // fps, 1_000_000 // fps),
                "AeEnable":            True,
                "AwbEnable":           True,
                "AwbMode":             4,    # Indoor — corrige tono azulado
                "Contrast":            1.4,
                "Saturation":          1.6,
                "Sharpness":           3.0,
                "NoiseReductionMode":  2,    # CDN_Fast
            },
        )
        self._picam2.configure(cfg)

    # ─── Ciclo de vida ────────────────────────────────────────────────────────

    def start(self) -> None:
        """Arranca la cámara, espera estabilización AE/AWB y lanza el hilo."""
        self._picam2.start()
        print(f"[CAM] Estabilizando AE/AWB ({self._warmup_s:.1f} s)...")
        time.sleep(self._warmup_s)
        self._lock_ae_awb()
        self._stop.clear()
        threading.Thread(
            target=self._capture_loop,
            name="CameraStream",
            daemon=True,
        ).start()
        # Esperar el primer frame antes de regresar
        self._ready.wait(timeout=5.0)
        print("[CAM] Lista.")

    def stop(self) -> None:
        self._stop.set()
        time.sleep(0.1)
        self._picam2.stop()

    # ─── API pública ──────────────────────────────────────────────────────────

    def get_frame(self) -> Optional[np.ndarray]:
        """
        Retorna el frame BGR más reciente.  Nunca bloquea.
        Retorna None si la cámara aún no ha capturado ningún frame.
        """
        with self._lock:
            return self._frame.copy() if self._frame is not None else None

    def recalibrate(self) -> None:
        """Re-calibra AE/AWB (usar en cambios de iluminación)."""
        self._picam2.set_controls({"AeEnable": True, "AwbEnable": True})
        time.sleep(self._warmup_s)
        self._lock_ae_awb()

    # ─── Bloqueo AE/AWB ───────────────────────────────────────────────────────

    def _lock_ae_awb(self) -> None:
        """Lee exposición y balance de blancos actuales y los fija."""
        try:
            meta   = self._picam2.capture_metadata()
            exp    = meta.get("ExposureTime")
            gain   = meta.get("AnalogueGain")
            cgains = meta.get("ColourGains")

            ctrl: dict = {"AeEnable": False}
            if exp    is not None: ctrl["ExposureTime"] = exp
            if gain   is not None: ctrl["AnalogueGain"] = gain
            if cgains is not None:
                ctrl["AwbEnable"]   = False
                ctrl["ColourGains"] = tuple(cgains)

            self._picam2.set_controls(ctrl)
            print(f"[CAM] AE/AWB bloqueados — exp={exp} µs  gain={gain:.2f}")
        except Exception as e:
            print(f"[CAM] No se pudo bloquear AE/AWB: {e}")

    # ─── Hilo de captura ──────────────────────────────────────────────────────

    def _capture_loop(self) -> None:
        while not self._stop.is_set():
            # capture_array() retorna RGB888
            rgb = self._picam2.capture_array()

            # REGLA DE ORO: convertir RGB → BGR para todos los módulos OpenCV
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

            with self._lock:
                self._frame = bgr

            self._ready.set()
