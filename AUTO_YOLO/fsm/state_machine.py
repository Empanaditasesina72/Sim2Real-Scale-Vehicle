# ===== ESTADOS =====
RUTA_LIBRE = 0
STOP_LEJOS = 1
STOP_CERCA = 2
EMERGENCIA = 3
SEMAFORO_ROJO = 4
SEMAFORO_AMARILLO = 5


class StateMachine:

    def __init__(self):
        self.estado = RUTA_LIBRE

    def evaluar(self, dist_stop, semaforo):

        # ===== PRIORIDAD SEMAFORO =====
        if semaforo == "red":
            self.estado = SEMAFORO_ROJO
            return self.estado

        if semaforo == "yellow":
            self.estado = SEMAFORO_AMARILLO
            return self.estado

        # ===== LOGICA STOP =====
        if dist_stop is None:
            self.estado = RUTA_LIBRE
            return self.estado

        if dist_stop < 10:
            self.estado = EMERGENCIA
        elif dist_stop < 25:
            self.estado = STOP_CERCA
        elif dist_stop < 50:
            self.estado = STOP_LEJOS
        else:
            self.estado = RUTA_LIBRE

        return self.estado

    def accion(self):

        match self.estado:  # ← IMPORTANTE: con .estado y :

            case 0:
                return 90, "RUTA LIBRE"

            case 1:
                return 50, "STOP LEJOS"

            case 2:
                return 20, "STOP CERCA"

            case 3:
                return 0, "EMERGENCIA"

            case 4:
                return 0, "SEMAFORO ROJO"

            case 5:
                return 40, "SEMAFORO AMARILLO"

            case _:
                return 90, "RUTA LIBRE"
