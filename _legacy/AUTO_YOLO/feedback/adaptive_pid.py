class AdaptivePID:

    def __init__(self):
        # ganancias iniciales
        self.kp = 0.6
        self.ki = 0.2
        self.kd = 0.05

    def actualizar(self, error):

        e = abs(error)

        # ===== REGLAS ADAPTATIVAS =====

        # proporcional: responde al error actual
        self.kp = 0.4 + 0.02 * e

        # integral: pequeña para evitar sobreacumulación
        self.ki = 0.15 + 0.005 * e

        # derivativa: amortiguación
        self.kd = 0.03 + 0.01 * e

        # ===== LIMITES =====
        self.kp = min(self.kp, 3.0)
        self.ki = min(self.ki, 1.0)
        self.kd = min(self.kd, 1.0)

        return self.kp, self.ki, self.kd
#pid para velocidad,error de linea 

def lane_control(frame, lines, desviacion):
    # Control proporcional para corrección de desviación
    kp_lane = 0.5
    control = kp_lane * desviacion

    # Limitar el control a un rango razonable
    control = max(-30, min(30, control))
    pwm = 50 + control  # Base PWM de 50, ajustada por el control
    return control
