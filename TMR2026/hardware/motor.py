# -*- coding: utf-8 -*-
"""
motor.py — Control IBT-2 con Soft-Start anti voltage-sag (TMR 2026).

NOTA DE COMPATIBILIDAD Pi 5:
  RPi.GPIO no soporta hardware PWM en Pi 5 (kernel 6.1+).
  Este módulo intenta RPi.GPIO (software PWM) primero.
  Si falla, usa lgpio (hardware PWM nativo Pi 5) automáticamente.
  En Pi 4 / Pi 3, RPi.GPIO funciona normalmente.

Cableado IBT-2:
  RPWM → GPIO 18  (avance)
  LPWM → GPIO 13  (reversa)
  R_EN + L_EN → 3.3V físico (siempre habilitado, sin control por GPIO)

Soft-Start:
  Un hilo interno (50 Hz) sube el duty a máx. _SLEW_UP % por tick.
  La bajada es 4× más rápida (seguridad).
  brake() corta INSTANTÁNEAMENTE a 0 sin pasar por la rampa.
"""

import threading
import time
from typing import Optional

# ── Selección de backend ──────────────────────────────────────────────────────
try:
    import RPi.GPIO as _GPIO
    _GPIO.setmode(_GPIO.BCM)
    _GPIO.setwarnings(False)
    _BACKEND = "RPi.GPIO"
except (ImportError, RuntimeError):
    try:
        import lgpio as _lgpio
        _BACKEND = "lgpio"
    except ImportError:
        _BACKEND = "mock"

_PWM_FREQ = 1_000   # Hz — frecuencia PWM para el IBT-2


class MotorDriver:
    """
    Interfaz de alto nivel para el puente H IBT-2.

    Uso::

        m = MotorDriver()
        m.set_speed(35.0)   # 35 % de potencia hacia adelante
        m.brake()           # corte inmediato a 0
        m.cleanup()
    """

    MAX_DUTY   = 100.0
    _SLEW_UP   = 2.0    # % por tick al subir   (50 Hz → ~1 s de 0 → 40 %)
    _SLEW_DOWN = 8.0    # % por tick al bajar   (frenado más rápido)
    _TICK_S    = 0.02   # 50 Hz hilo interno

    def __init__(self, pin_rpwm: int = 18, pin_lpwm: int = 13):
        self._pin_r = pin_rpwm
        self._pin_l = pin_lpwm
        self._current = 0.0
        self._target  = 0.0
        self._lock    = threading.Lock()
        self._running = True

        self._init_hw()

        self._thread = threading.Thread(
            target=self._ramp_loop, name="MotorRamp", daemon=True
        )
        self._thread.start()
        print(f"[MOTOR] Backend: {_BACKEND}  RPWM=GPIO{pin_rpwm}  LPWM=GPIO{pin_lpwm}")

    # ─── API pública ──────────────────────────────────────────────────────────

    def set_speed(self, duty: float) -> None:
        """
        Establece la velocidad objetivo.  La rampa interna la alcanzará
        gradualmente (soft-start).  duty ∈ [-MAX_DUTY, +MAX_DUTY];
        positivo = avance, negativo = reversa.
        """
        duty = max(-self.MAX_DUTY, min(self.MAX_DUTY, float(duty)))
        with self._lock:
            self._target = duty

    def brake(self) -> None:
        """
        Freno inmediato: corta el PWM a EXACTAMENTE 0.
        Omite la rampa.  Llama esto en FRENADO/ESPERA.
        """
        with self._lock:
            self._target  = 0.0
            self._current = 0.0
        self._apply_hw(0.0)

    @property
    def current_duty(self) -> float:
        """Duty cycle actual (el que se está aplicando al puente H)."""
        with self._lock:
            return self._current

    def cleanup(self) -> None:
        """Libera GPIO. Llamar en shutdown."""
        self._running = False
        self.brake()
        time.sleep(self._TICK_S * 2)
        if _BACKEND == "RPi.GPIO":
            try:
                self._pwm_r.stop()
                self._pwm_l.stop()
                _GPIO.cleanup([self._pin_r, self._pin_l])
            except Exception:
                pass
        elif _BACKEND == "lgpio":
            try:
                _lgpio.gpiochip_close(self._h)
            except Exception:
                pass

    # ─── Inicialización de hardware ───────────────────────────────────────────

    def _init_hw(self) -> None:
        if _BACKEND == "RPi.GPIO":
            _GPIO.setup(self._pin_r, _GPIO.OUT)
            _GPIO.setup(self._pin_l, _GPIO.OUT)
            self._pwm_r = _GPIO.PWM(self._pin_r, _PWM_FREQ)
            self._pwm_l = _GPIO.PWM(self._pin_l, _PWM_FREQ)
            self._pwm_r.start(0)
            self._pwm_l.start(0)

        elif _BACKEND == "lgpio":
            self._h = _lgpio.gpiochip_open(4)   # chip 4 en Pi 5
            _lgpio.gpio_claim_output(self._h, self._pin_r)
            _lgpio.gpio_claim_output(self._h, self._pin_l)
            _lgpio.tx_pwm(self._h, self._pin_r, _PWM_FREQ, 0)
            _lgpio.tx_pwm(self._h, self._pin_l, _PWM_FREQ, 0)

        # mock: no hace nada (útil para pruebas en PC)

    # ─── Hilo de rampa (50 Hz) ────────────────────────────────────────────────

    def _ramp_loop(self) -> None:
        """Mueve _current hacia _target con límite de rampa."""
        while self._running:
            with self._lock:
                target  = self._target
                current = self._current

            diff = target - current
            if diff > 0:
                step = min(diff,  self._SLEW_UP)
            elif diff < 0:
                step = max(diff, -self._SLEW_DOWN)
            else:
                step = 0.0

            if step != 0.0:
                new_duty = current + step
                with self._lock:
                    self._current = new_duty
                self._apply_hw(new_duty)

            time.sleep(self._TICK_S)

    # ─── Aplicación al hardware ───────────────────────────────────────────────

    def _apply_hw(self, duty: float) -> None:
        """Escribe el duty cycle en el puente H. Sin rampa."""
        r = max(0.0, min(100.0,  duty))
        l = max(0.0, min(100.0, -duty))

        if _BACKEND == "RPi.GPIO":
            self._pwm_r.ChangeDutyCycle(r)
            self._pwm_l.ChangeDutyCycle(l)

        elif _BACKEND == "lgpio":
            _lgpio.tx_pwm(self._h, self._pin_r, _PWM_FREQ, r)
            _lgpio.tx_pwm(self._h, self._pin_l, _PWM_FREQ, l)
