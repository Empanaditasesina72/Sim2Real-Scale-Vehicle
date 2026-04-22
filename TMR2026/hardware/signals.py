# -*- coding: utf-8 -*-
"""
signals.py — Direccionales y hazards para TMR 2026.

Un LED por lado (izquierdo / derecho). Hazard = ambos parpadeando a la vez.
Parpadeo controlado por `time.monotonic()` — NUNCA `sleep()`, NUNCA un hilo.
El main loop llama a `tick()` a 50 Hz; el parpadeo ocurre entre llamadas.

Modos:
  OFF    → ambos LEDs apagados
  LEFT   → parpadea solo el izquierdo
  RIGHT  → parpadea solo el derecho
  HAZARD → parpadean ambos en fase

Pines por defecto (BCM): 19 (izq) / 20 (der). Coinciden con
`vision_config.yaml:gpio.led_turn_left / led_turn_right` para reutilizar
el cableado del script de prueba `vision_module.py`.

Backend: intenta `lgpio` (chip 4, Pi 5) y cae a `RPi.GPIO` si no está.
Si ningún backend está disponible, el módulo queda en modo no-op —
el resto del programa sigue funcionando sin direccionales.
"""

from enum import Enum, auto
import time
from typing import Optional


class SignalMode(Enum):
    OFF    = auto()
    LEFT   = auto()
    RIGHT  = auto()
    HAZARD = auto()


class TurnSignals:
    """
    Controlador de direccionales / hazards con parpadeo por software.

    Uso::

        signals = TurnSignals(pin_left=19, pin_right=20, blink_hz=2.0)
        signals.set_mode(SignalMode.LEFT)
        while running:
            signals.tick()      # cada iteración del loop principal
        signals.close()
    """

    def __init__(
        self,
        pin_left:  int   = 19,
        pin_right: int   = 20,
        blink_hz:  float = 2.0,
    ):
        self._pin_l = pin_left
        self._pin_r = pin_right
        # 2 Hz → periodo 0.5 s → semi-periodo 0.25 s (0.25 on / 0.25 off)
        self._half_period = 0.5 / blink_hz if blink_hz > 0 else 0.25

        self._mode        = SignalMode.OFF
        self._last_toggle = time.monotonic()
        self._blink_on    = False   # fase actual del parpadeo

        self._backend: Optional[str] = None
        self._handle:  Optional[int] = None
        self._setup_gpio()

    # ─── Backend GPIO (lgpio → RPi.GPIO fallback) ────────────────────────────
    def _setup_gpio(self) -> None:
        try:
            import lgpio
            self._lgpio  = lgpio
            self._handle = lgpio.gpiochip_open(4)   # Pi 5 chip 4
            lgpio.gpio_claim_output(self._handle, self._pin_l, 0)
            lgpio.gpio_claim_output(self._handle, self._pin_r, 0)
            self._backend = "lgpio"
            print(f"[SIGNALS] lgpio OK — izq={self._pin_l}  der={self._pin_r}")
            return
        except Exception as e:
            last_err = e

        try:
            import RPi.GPIO as GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            GPIO.setup(self._pin_l, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self._pin_r, GPIO.OUT, initial=GPIO.LOW)
            self._GPIO    = GPIO
            self._backend = "RPi.GPIO"
            print(f"[SIGNALS] RPi.GPIO OK — izq={self._pin_l}  der={self._pin_r}")
            return
        except Exception as e:
            last_err = e

        print(f"[SIGNALS] Sin GPIO — direccionales deshabilitadas ({last_err})")
        self._backend = None

    def _write(self, pin: int, value: int) -> None:
        if self._backend == "lgpio":
            self._lgpio.gpio_write(self._handle, pin, value)
        elif self._backend == "RPi.GPIO":
            self._GPIO.output(pin, value)

    # ─── API pública ─────────────────────────────────────────────────────────
    def set_mode(self, mode: SignalMode) -> None:
        """Cambia el modo. Si es el mismo no hace nada."""
        if mode == self._mode:
            return
        self._mode        = mode
        self._last_toggle = time.monotonic()
        self._blink_on    = True          # primer flanco encendido inmediato
        self._apply()

    @property
    def mode(self) -> SignalMode:
        return self._mode

    def tick(self) -> None:
        """
        Avanza el parpadeo según `time.monotonic()`.
        Llamar en cada iteración del loop principal (50 Hz recomendado).
        No bloquea.
        """
        if self._mode == SignalMode.OFF:
            if self._blink_on:
                self._blink_on = False
                self._apply()
            return

        now = time.monotonic()
        if now - self._last_toggle >= self._half_period:
            self._blink_on    = not self._blink_on
            self._last_toggle = now
            self._apply()

    def _apply(self) -> None:
        """Escribe el estado actual a los dos LEDs según mode + blink_on."""
        left  = 0
        right = 0
        if self._blink_on:
            if self._mode in (SignalMode.LEFT,  SignalMode.HAZARD):
                left  = 1
            if self._mode in (SignalMode.RIGHT, SignalMode.HAZARD):
                right = 1
        self._write(self._pin_l, left)
        self._write(self._pin_r, right)

    def close(self) -> None:
        """Apaga LEDs y libera GPIO."""
        try:
            if self._backend == "lgpio" and self._handle is not None:
                self._lgpio.gpio_write(self._handle, self._pin_l, 0)
                self._lgpio.gpio_write(self._handle, self._pin_r, 0)
                self._lgpio.gpiochip_close(self._handle)
            elif self._backend == "RPi.GPIO":
                self._GPIO.output(self._pin_l, 0)
                self._GPIO.output(self._pin_r, 0)
        except Exception:
            pass
        self._backend = None
