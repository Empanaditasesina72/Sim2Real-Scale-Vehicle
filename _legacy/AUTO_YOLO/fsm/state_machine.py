from fsm.semaforo_fsm import SemaforoFSM
from fsm.stop_fsm import StopFSM
from fsm.sensor_fsm import SensorFSM


class StateMachine:

    def __init__(self):

        self.semaforo_fsm = SemaforoFSM()
        self.stop_fsm = StopFSM()
        self.sensor_fsm = SensorFSM()

        self.setpoint = 90
        self.estado_txt = "RUTA LIBRE"

    def evaluar(self, dist_stop, semaforo, dist_sensor):

        # ===== EVALUAR SUBMAQUINAS =====

        self.semaforo_fsm.evaluar(semaforo)
        self.stop_fsm.evaluar(dist_stop)
        self.sensor_fsm.evaluar(dist_sensor)

        sp_sem, txt_sem = self.semaforo_fsm.accion()
        sp_stop, txt_stop = self.stop_fsm.accion()
        sp_sensor, txt_sensor = self.sensor_fsm.accion()

        # ===== PRIORIDAD JERARQUICA =====

        # 1 SENSOR (seguridad maxima)
        if sp_sensor is not None:
            self.setpoint = sp_sensor
            self.estado_txt = txt_sensor
            return

        # 2 SEMAFORO
        if sp_sem is not None:
            self.setpoint = sp_sem
            self.estado_txt = txt_sem
            return

        # 3 STOP
        self.setpoint = sp_stop
        self.estado_txt = txt_stop

    def accion(self):

        return self.setpoint, self.estado_txt
