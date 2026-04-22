class PWMController:

    def __init__(self, step=3):

        # porcentaje actual
        self.current_pwm = 0

        # máximo cambio por ciclo
        self.step = step


    def update(self, target_pwm):

        # limitar rango
        target_pwm = max(0, min(100, target_pwm))

        difference = target_pwm - self.current_pwm

        # limitar cambio brusco
        if abs(difference) > self.step:
            difference = self.step if difference > 0 else -self.step

        self.current_pwm += difference

        return self.current_pwm
