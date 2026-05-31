# -*- coding: utf-8 -*-
"""
parking_fsm.py — Maniobra de ESTACIONAMIENTO EN BATERÍA (perpendicular).

Sub-máquina de estados para la Prueba 3 del PDF "Validación Sim2Real":
   VISION → AUTONOMOUS → PARKING_SEARCH → PARKING_MANEUVER → PARKED

Diseño:
  • PARKING_SEARCH: avanza despacio por el carril buscando el hueco. Lo
    detecta con el ToF lateral/frontal (o por tiempo si no hay sensor).
  • PARKING_MANEUVER: maniobra en lazo abierto por TIEMPO (igual que el
    estacionamiento real del coche): gira a la derecha y avanza para entrar
    perpendicular al cajón, luego endereza.
  • PARKED: motor a 0, estacionado.

Garantías (como el resto del proyecto):
  • Usa time.monotonic(), NUNCA time.sleep() — el bucle nunca se bloquea.
  • brake() es corte inmediato; no se modifica.

Tiempos calibrables desde config.py (PARK_*). Si no existen, usa defaults.
"""

import time
from enum import Enum, auto

try:
    from config import (
        PARK_SEARCH_SPEED, PARK_MANEUVER_SPEED,
        PARK_REVERSE_LOCK_SEC, PARK_REVERSE_STRAIGHT_SEC,
        SERVO_CENTER_ANGLE, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE,
    )
except ImportError:
    PARK_SEARCH_SPEED = 15
    PARK_MANEUVER_SPEED = 12
    PARK_REVERSE_LOCK_SEC = 2.5
    PARK_REVERSE_STRAIGHT_SEC = 1.0
    SERVO_CENTER_ANGLE = 90.0
    SERVO_MIN_ANGLE = 58.0
    SERVO_MAX_ANGLE = 122.0


class ParkingState(Enum):
    PARKING_SEARCH   = auto()   # busca el hueco
    PARKING_MANEUVER = auto()   # maniobra de entrada
    PARKED           = auto()   # estacionado


class ParkingFSM:
    """
    Estacionamiento en batería. Uso::

        pk = ParkingFSM(motor, steering)
        pk.activate()
        while running:
            pk.lidar_mm = sensor.front_mm   # opcional (detección de hueco)
            pk.update(dt)                    # 50 Hz, no bloquea
            if pk.state == ParkingState.PARKED: break
    """

    # ── Tiempos de la maniobra (s) — calibrables ──────────────────────────────
    SEARCH_MIN_S   = 1.5     # mínimo buscando antes de aceptar el hueco
    SEARCH_MAX_S   = 4.0     # tope de búsqueda antes de maniobrar de todos modos
    TURN_IN_S      = 2.2     # girar+avanzar para entrar perpendicular
    STRAIGHTEN_S   = 1.2     # enderezar dentro del cajón
    # Umbral de hueco: el ToF frontal sube por encima de esto al pasar el hueco
    GAP_FRONT_MM   = 600

    SEARCH_SPEED   = float(PARK_SEARCH_SPEED)
    MANEUVER_SPEED = float(PARK_MANEUVER_SPEED)

    def __init__(self, motor, steering):
        self.motor = motor
        self.steering = steering

        self.lidar_mm = None         # distancia frontal (opcional)
        self._state = ParkingState.PARKING_SEARCH
        self._t_state = 0.0          # momento en que se entró al estado
        self._active = False
        # Sub-fase de la maniobra: 0 = girar+entrar, 1 = enderezar
        self._man_phase = 0

    # ─── Ciclo de vida ──────────────────────────────────────────────────────────
    def activate(self):
        self._state = ParkingState.PARKING_SEARCH
        self._t_state = time.monotonic()
        self._man_phase = 0
        self._active = True
        print("[PARK] Estacionamiento ACTIVADO → PARKING_SEARCH")

    def deactivate(self):
        self._active = False
        self.motor.brake()
        self.steering.set_angle(SERVO_CENTER_ANGLE)

    @property
    def state(self) -> ParkingState:
        return self._state

    @property
    def done(self) -> bool:
        return self._state == ParkingState.PARKED

    def _elapsed(self) -> float:
        return time.monotonic() - self._t_state

    def _go(self, new_state: ParkingState):
        print(f"[PARK] {self._state.name} → {new_state.name}")
        self._state = new_state
        self._t_state = time.monotonic()

    # ─── Tick (50 Hz) ─────────────────────────────────────────────────────────────
    def update(self, dt: float):
        if not self._active:
            return

        if self._state == ParkingState.PARKING_SEARCH:
            self._do_search()
        elif self._state == ParkingState.PARKING_MANEUVER:
            self._do_maneuver()
        elif self._state == ParkingState.PARKED:
            self.motor.brake()
            self.steering.set_angle(SERVO_CENTER_ANGLE)

    def _do_search(self):
        # Avanza recto y despacio buscando el hueco
        self.steering.set_angle(SERVO_CENTER_ANGLE)
        self.motor.set_speed(self.SEARCH_SPEED)

        # Buscar al menos SEARCH_MIN_S antes de aceptar el hueco (búsqueda real)
        hueco = (self._elapsed() >= self.SEARCH_MIN_S
                 and self.lidar_mm is not None
                 and self.lidar_mm >= self.GAP_FRONT_MM)
        # Maniobra cuando detecta hueco O cuando agota el tiempo de búsqueda
        if hueco or self._elapsed() >= self.SEARCH_MAX_S:
            self._man_phase = 0
            self._go(ParkingState.PARKING_MANEUVER)

    def _do_maneuver(self):
        # Fase 0: girar ruedas a la DERECHA y avanzar → entra perpendicular
        if self._man_phase == 0:
            self.steering.set_angle(SERVO_MAX_ANGLE)      # derecha máxima
            self.motor.set_speed(self.MANEUVER_SPEED)
            if self._elapsed() >= self.TURN_IN_S:
                self._man_phase = 1
                self._t_state = time.monotonic()
        # Fase 1: enderezar y entrar al fondo del cajón
        elif self._man_phase == 1:
            self.steering.set_angle(SERVO_CENTER_ANGLE)
            self.motor.set_speed(self.MANEUVER_SPEED * 0.7)
            if self._elapsed() >= self.STRAIGHTEN_S:
                self.motor.brake()
                self.steering.set_angle(SERVO_CENTER_ANGLE)
                self._go(ParkingState.PARKED)
                print("[PARK] ✔ Estacionado en batería (PARKED)")
