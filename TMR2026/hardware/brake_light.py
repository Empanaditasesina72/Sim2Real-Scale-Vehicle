# -*- coding: utf-8 -*-
"""
brake_light.py — LED de freno para TMR 2026.

Un solo LED que se enciende mientras el coche está frenando o detenido
(estados FRENADO / ESPERA de la FSM). NO parpadea — encendido continuo.

Pin por defecto (BCM): 16.
Backend: `lgpio` (chip 4, Pi 5) con fallback a `RPi.GPIO`. Si no hay
ningún backend, queda en modo no-op sin romper al resto del sistema.
"""

from typing import Optional


class BrakeLight:
    """
    Controlador del LED de freno.

    Uso::

        brake = BrakeLight(pin=16)
        brake.on()    # al entrar a FRENADO o ESPERA
        brake.off()   # al salir
        brake.close()
    """

    def __init__(self, pin: int = 16):
        self._pin     = pin
        self._is_on   = False
        self._backend: Optional[str] = None
        self._handle:  Optional[int] = None
        self._setup_gpio()

    def _setup_gpio(self) -> None:
        try:
            import lgpio
            self._lgpio  = lgpio
            self._handle = lgpio.gpiochip_open(4)
            lgpio.gpio_claim_output(self._handle, self._pin, 0)
            self._backend = "lgpio"
            print(f"[BRAKE] lgpio OK — pin={self._pin}")
            return
        except Exception as e:
            last_err = e

        try:
            import RPi.GPIO as GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            GPIO.setup(self._pin, GPIO.OUT, initial=GPIO.LOW)
            self._GPIO    = GPIO
            self._backend = "RPi.GPIO"
            print(f"[BRAKE] RPi.GPIO OK — pin={self._pin}")
            return
        except Exception as e:
            last_err = e

        print(f"[BRAKE] Sin GPIO — luz de freno deshabilitada ({last_err})")
        self._backend = None

    def _write(self, value: int) -> None:
        if self._backend == "lgpio":
            self._lgpio.gpio_write(self._handle, self._pin, value)
        elif self._backend == "RPi.GPIO":
            self._GPIO.output(self._pin, value)

    def on(self) -> None:
        if not self._is_on:
            self._write(1)
            self._is_on = True

    def off(self) -> None:
        if self._is_on:
            self._write(0)
            self._is_on = False

    @property
    def is_on(self) -> bool:
        return self._is_on

    def close(self) -> None:
        try:
            if self._backend == "lgpio" and self._handle is not None:
                self._lgpio.gpio_write(self._handle, self._pin, 0)
                self._lgpio.gpiochip_close(self._handle)
            elif self._backend == "RPi.GPIO":
                self._GPIO.output(self._pin, 0)
        except Exception:
            pass
        self._is_on   = False
        self._backend = None
