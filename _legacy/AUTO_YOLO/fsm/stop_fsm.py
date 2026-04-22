# ===== ESTADOS STOP =====
RUTA_LIBRE = 0
STOP_LEJOS = 1
STOP_CERCA = 2
EMERGENCIA = 3


class StopFSM:

    def __init__(self):
        self.estado = RUTA_LIBRE

    def evaluar(self, dist_stop):

        if dist_stop is None:
            self.estado = RUTA_LIBRE
            return self.estado

        if dist_stop < 50:   # 🔥 paro total
            self.estado = EMERGENCIA

        elif dist_stop < 70:
            self.estado = STOP_CERCA

        elif dist_stop < 100:
            self.estado = STOP_LEJOS

        else:
            self.estado = RUTA_LIBRE

        return self.estado

    def accion(self):

        match self.estado:

            case 0:
                return 90, "RUTA LIBRE"

            case 1:
                return 60, "STOP LEJOS"

            case 2:
                return 30, "STOP CERCA"

            case 3:
                return 0, "EMERGENCIA"

            case _:
                return 90, "RUTA LIBRE"
