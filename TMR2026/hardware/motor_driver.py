"""IBT-2 H-bridge control via lgpio (native Pi 5).

Real car wiring:
  RPWM = GPIO 18 (Pin 12)  -- forward PWM
  LPWM = GPIO 13 (Pin 33)  -- reverse PWM
  R_EN + L_EN -> fixed 3.3V -- always enabled (no enable GPIO)

With R_EN and L_EN at 3.3V the bridge is always active.
Direction and speed control use only RPWM and LPWM:
  Forward -> RPWM=%duty, LPWM=0
  Reverse -> RPWM=0,     LPWM=%duty
  Brake   -> RPWM=100,   LPWM=100  (electrical brake)
  Stop    -> RPWM=0,     LPWM=0    (freewheel)
"""

import lgpio
from config import PIN_MOTOR_RPWM, PIN_MOTOR_LPWM, MOTOR_PWM_FREQ

_CHIP = 4

_SLEW_STEP = 3.0


class MotorDriver:
    """Interfaz de alto nivel para el IBT-2 con enable permanente."""

    def __init__(self):
        self._h = lgpio.gpiochip_open(_CHIP)
        lgpio.gpio_claim_output(self._h, PIN_MOTOR_RPWM)
        lgpio.gpio_claim_output(self._h, PIN_MOTOR_LPWM)
        lgpio.tx_pwm(self._h, PIN_MOTOR_RPWM, MOTOR_PWM_FREQ, 0)
        lgpio.tx_pwm(self._h, PIN_MOTOR_LPWM, MOTOR_PWM_FREQ, 0)
        self._current_duty = 0.0

    def enable(self):
        """No-op -- enable is in hardware (fixed 3.3V)."""
        pass

    def disable(self):
        """Freewheel -- cut both PWMs."""
        lgpio.tx_pwm(self._h, PIN_MOTOR_RPWM, MOTOR_PWM_FREQ, 0)
        lgpio.tx_pwm(self._h, PIN_MOTOR_LPWM, MOTOR_PWM_FREQ, 0)
        self._current_duty = 0.0

    def set_throttle(self, duty: float):
        """
        Apply power to the motor with a ramp-up (anti-inrush).

        Parameters
        ----------
        duty : float
            [-100, 100]  >0 = forward, <0 = reverse, 0 = brake

        Power decreases (reducing or braking) are instantaneous.
        Increases are limited to _SLEW_STEP % per call to avoid current
        spikes that shut down the battery.
        """
        duty = max(-100.0, min(100.0, duty))

        if duty > self._current_duty:
            diff = duty - self._current_duty
            if diff > _SLEW_STEP:
                duty = self._current_duty + _SLEW_STEP

        self._current_duty = duty

        if duty > 0:
            lgpio.tx_pwm(self._h, PIN_MOTOR_RPWM, MOTOR_PWM_FREQ, duty)
            lgpio.tx_pwm(self._h, PIN_MOTOR_LPWM, MOTOR_PWM_FREQ, 0)
        elif duty < 0:
            lgpio.tx_pwm(self._h, PIN_MOTOR_RPWM, MOTOR_PWM_FREQ, 0)
            lgpio.tx_pwm(self._h, PIN_MOTOR_LPWM, MOTOR_PWM_FREQ, -duty)
        else:
            self.brake()

    def brake(self):
        """Electrical brake -- both inputs at 100 %."""
        lgpio.tx_pwm(self._h, PIN_MOTOR_RPWM, MOTOR_PWM_FREQ, 100)
        lgpio.tx_pwm(self._h, PIN_MOTOR_LPWM, MOTOR_PWM_FREQ, 100)
        self._current_duty = 0.0

    def stop(self):
        """Alias de brake."""
        self.brake()

    @property
    def duty(self) -> float:
        return self._current_duty

    def cleanup(self):
        self.disable()
        lgpio.gpio_free(self._h, PIN_MOTOR_RPWM)
        lgpio.gpio_free(self._h, PIN_MOTOR_LPWM)
        lgpio.gpiochip_close(self._h)
