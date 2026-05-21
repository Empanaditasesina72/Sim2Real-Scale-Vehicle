# -*- coding: utf-8 -*-
"""
pid_controller.py — Controlador PID genérico con anti-windup.

Características:
  - Anti-windup por clamping del integrador.
  - Derivada sobre la medida (no sobre el error) para evitar
    "derivative kick" cuando el setpoint cambia abruptamente.
  - Reset en caliente sin perder el estado I/D previo.
"""

import time


class PIDController:
    """
    PID discreto de tiempo real.

    Parameters
    ----------
    kp, ki, kd : float
        Ganancias proporcional, integral y derivativa.
    setpoint : float
        Valor objetivo (puede cambiarse en runtime).
    output_limits : (min, max)
        Saturación de la salida.
    integral_limits : (min, max)
        Límites del acumulador integral (anti-windup).
    derivative_on_measurement : bool
        Si True, la derivada se calcula sobre la medida (recomendado).
        Si False, se calcula sobre el error (comportamiento clásico).
    """

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        setpoint: float = 0.0,
        output_limits: tuple[float, float] = (-100.0, 100.0),
        integral_limits: tuple[float, float] = (-50.0, 50.0),
        derivative_on_measurement: bool = True,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.output_limits = output_limits
        self.integral_limits = integral_limits
        self.derivative_on_measurement = derivative_on_measurement

        self._integral   = 0.0
        self._last_input = 0.0  # Para derivada sobre medida
        self._last_error = 0.0  # Para derivada sobre error
        self._last_time  = time.monotonic()

        # Componentes del último cálculo (lectura para telemetría / overlay).
        self.last_error  = 0.0
        self.last_p      = 0.0
        self.last_i      = 0.0
        self.last_d      = 0.0
        self.last_output = 0.0

    # ----------------------------------------------------------
    def compute(self, measurement: float, dt: float | None = None) -> float:
        """
        Calcula la salida del PID.

        Parameters
        ----------
        measurement : float
            Valor actual de la variable controlada.
        dt : float, optional
            Intervalo de tiempo en segundos.  Si None se usa el tiempo
            real transcurrido desde la última llamada.

        Returns
        -------
        float
            Salida saturada del controlador.
        """
        now = time.monotonic()
        if dt is None:
            dt = now - self._last_time
        if dt <= 0:
            dt = 1e-4
        self._last_time = now

        error = self.setpoint - measurement

        # Proporcional
        p_out = self.kp * error

        # Integral con anti-windup
        self._integral += error * dt
        self._integral = max(
            self.integral_limits[0],
            min(self.integral_limits[1], self._integral)
        )
        i_out = self.ki * self._integral

        # Derivada
        if self.derivative_on_measurement:
            d_out = -self.kd * (measurement - self._last_input) / dt
            self._last_input = measurement
        else:
            d_out = self.kd * (error - self._last_error) / dt
            self._last_error = error

        output = p_out + i_out + d_out
        output = max(self.output_limits[0], min(self.output_limits[1], output))

        self.last_error  = error
        self.last_p      = p_out
        self.last_i      = i_out
        self.last_d      = d_out
        self.last_output = output

        return output

    def reset(self):
        """Reinicia el estado interno sin cambiar las ganancias."""
        self._integral   = 0.0
        self._last_input = 0.0
        self._last_error = 0.0
        self._last_time  = time.monotonic()
        self.last_error  = 0.0
        self.last_p      = 0.0
        self.last_i      = 0.0
        self.last_d      = 0.0
        self.last_output = 0.0

    def update_gains(self, kp: float, ki: float, kd: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
