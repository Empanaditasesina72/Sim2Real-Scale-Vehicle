# ===== ESTADOS SEMAFORO =====
SEMAFORO_VERDE = 0
SEMAFORO_AMARILLO = 1
SEMAFORO_ROJO = 2


class SemaforoFSM:

    def __init__(self):
        self.estado = SEMAFORO_VERDE

    def evaluar(self, semaforo):

        if semaforo == "red":
            self.estado = SEMAFORO_ROJO

        elif semaforo == "yellow":
            self.estado = SEMAFORO_AMARILLO

        else:
            self.estado = SEMAFORO_VERDE

        return self.estado

    def accion(self):

        match self.estado:

            case 0:
                return None, "VERDE"

            case 1:
                return 40, "SEMAFORO AMARILLO"

            case 2:
                return 0, "SEMAFORO ROJO"

            case _:
                return None, "VERDE"
