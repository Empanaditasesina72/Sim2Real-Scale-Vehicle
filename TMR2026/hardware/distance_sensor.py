# -*- coding: utf-8 -*-
"""
distance_sensor.py — Dos VL53L0X opcionales en I²C bus 4 (GPIO 22/23).

Si los sensores no están conectados el sistema sigue funcionando
y todas las lecturas devuelven None. El modo autónomo usa entonces
solo la cámara para estimar distancias.

Cableado:
  SDA        → GPIO 23 (Pin 16)
  SCL        → GPIO 22 (Pin 15)
  XSHUT front → GPIO 17 (Pin 11)
  XSHUT rear  → GPIO 27 (Pin 13)
"""

import threading
import time

import lgpio

from config import (
    PIN_TOF_XSHUT_FRONT, PIN_TOF_XSHUT_REAR,
    TOF_ADDR_FRONT, TOF_ADDR_REAR,
    TOF_TIMING_BUDGET_US, TOF_MAX_RANGE_MM, TOF_POLL_INTERVAL_S,
)

_CHIP = 4   # GPIO chip Pi 5


class DistanceSensor:
    """
    Gestiona dos VL53L0X. Si no están disponibles, retorna None siempre.
    """

    def __init__(self):
        self._available = False
        self._front = None
        self._rear  = None

        # Abrir chip GPIO para los pines XSHUT
        self._h = lgpio.gpiochip_open(_CHIP)
        lgpio.gpio_claim_output(self._h, PIN_TOF_XSHUT_FRONT, 0, 0)  # LOW
        lgpio.gpio_claim_output(self._h, PIN_TOF_XSHUT_REAR,  0, 0)  # LOW

        try:
            import adafruit_vl53l0x
            from adafruit_extended_bus import ExtendedI2C

            i2c = ExtendedI2C(4)  # /dev/i2c-4 (GPIO 22=SCL, GPIO 23=SDA)

            # Sensor delantero → cambiar dirección a 0x30
            lgpio.gpio_write(self._h, PIN_TOF_XSHUT_FRONT, 1)
            time.sleep(0.1)
            front = adafruit_vl53l0x.VL53L0X(i2c)
            front.set_address(TOF_ADDR_FRONT)
            front.measurement_timing_budget = TOF_TIMING_BUDGET_US
            self._front = front

            # Sensor trasero → queda en 0x29
            lgpio.gpio_write(self._h, PIN_TOF_XSHUT_REAR, 1)
            time.sleep(0.1)
            rear = adafruit_vl53l0x.VL53L0X(i2c)
            rear.measurement_timing_budget = TOF_TIMING_BUDGET_US
            self._rear = rear

            self._available = True
            print("[TOF] Dos sensores VL53L0X inicializados OK")

        except Exception as e:
            # Liberar XSHUT para que no queden en LOW indefinidamente
            lgpio.gpio_write(self._h, PIN_TOF_XSHUT_FRONT, 1)
            lgpio.gpio_write(self._h, PIN_TOF_XSHUT_REAR,  1)
            print(f"[TOF] Sensores no disponibles ({e}) — continuando sin ToF")

        self._front_mm: float | None = None
        self._rear_mm:  float | None = None
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    # ----------------------------------------------------------
    def start(self):
        if not self._available:
            return
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._poll_loop, name="ToF", daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        lgpio.gpio_free(self._h, PIN_TOF_XSHUT_FRONT)
        lgpio.gpio_free(self._h, PIN_TOF_XSHUT_REAR)
        lgpio.gpiochip_close(self._h)

    # ----------------------------------------------------------
    @property
    def front_mm(self) -> float | None:
        with self._lock:
            return self._front_mm

    @property
    def rear_mm(self) -> float | None:
        with self._lock:
            return self._rear_mm

    @property
    def distance_mm(self) -> float | None:
        """Alias frontal para compatibilidad con el resto del código."""
        return self.front_mm

    def is_obstacle_near(self, threshold_mm: float) -> bool:
        d = self.front_mm
        return d is not None and d < threshold_mm

    # ----------------------------------------------------------
    def _poll_loop(self):
        while not self._stop_event.is_set():
            try:
                rf = self._front.range if self._front else None
                rr = self._rear.range  if self._rear  else None
                front = rf if (rf and rf < TOF_MAX_RANGE_MM) else None
                rear  = rr if (rr and rr < TOF_MAX_RANGE_MM) else None
            except Exception:
                front = rear = None

            with self._lock:
                self._front_mm = front
                self._rear_mm  = rear

            time.sleep(TOF_POLL_INTERVAL_S)
