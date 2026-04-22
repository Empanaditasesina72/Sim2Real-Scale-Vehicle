# ===== ESTADOS SENSOR =====

SENSOR_LIBRE = 0
FRENO_SUAVE = 1
FRENO_FUERTE = 2
STOP_SENSOR = 3


class SensorFSM:

    def __init__(self):
        self.estado = SENSOR_LIBRE

    def evaluar(self, distancia):

        if distancia is None:
            self.estado = SENSOR_LIBRE
            return self.estado

        # VL53L0X devuelve milimetros

        if distancia < 50:
            self.estado = STOP_SENSOR

        elif distancia < 100:
            self.estado = FRENO_FUERTE

        elif distancia < 200:
            self.estado = FRENO_SUAVE

        else:
            self.estado = SENSOR_LIBRE

        return self.estado

    def accion(self):

        match self.estado:

            case 0:
                return None, "SENSOR LIBRE"

            case 1:
                return 40, "FRENO SUAVE"

            case 2:
                return 20, "FRENO FUERTE"

            case 3:
                return 0, "STOP SENSOR"

            case _:
                return None, "SENSOR LIBRE"
