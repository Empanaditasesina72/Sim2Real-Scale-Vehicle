# ===== ESTADOS DEL VEHÍCULO AUTÓNOMO =====
RUTA_LIBRE = 0
STOP_LEJOS = 1
STOP_CERCA = 2
EMERGENCIA = 3
SEMAFORO_ROJO = 4
SEMAFORO_AMARILLO = 5
PARKING_SEARCH = 6
PARKING_MANEUVER = 7
INTERSECTION_STOP = 8
LANE_CHANGE_LEFT = 9
LANE_CHANGE_RIGHT = 10
OBSTACLE_AVOIDANCE = 11

class StateMachine:
    def __init__(self):
        self.estado = RUTA_LIBRE
        self.prev_estado = RUTA_LIBRE
        self.parking_attempts = 0
        self.lane_change_progress = 0

    def evaluar(self, perception_output):
        """
        Evalúa el estado basado en la salida de percepción
        """
        dist_obj = perception_output['distancia_obj']
        tipo_obj = perception_output['tipo_obj']
        semaforo = perception_output['semaforo']
        interseccion = perception_output['interseccion']
        lane_error = perception_output['lane_error']

        # Guardar estado anterior
        self.prev_estado = self.estado

        # ===== PRIORIDAD MÁXIMA: SEGURIDAD =====
        if dist_obj is not None and dist_obj < 15:
            self.estado = EMERGENCIA
            return self.estado

        # ===== PRIORIDAD ALTA: SEMÁFOROS =====
        if semaforo == "red":
            self.estado = SEMAFORO_ROJO
            return self.estado

        if semaforo == "yellow":
            self.estado = SEMAFORO_AMARILLO
            return self.estado

        # ===== INTERSECCIONES =====
        if interseccion and tipo_obj in ["stop", "yield"]:
            self.estado = INTERSECTION_STOP
            return self.estado

        # ===== OBJETOS EN CARRIL =====
        if dist_obj is not None:
            if dist_obj < 25:
                self.estado = STOP_CERCA
            elif dist_obj < 50:
                self.estado = STOP_LEJOS
            else:
                # Verificar si necesitamos cambiar de carril
                if abs(lane_error) > 50:  # Error de carril significativo
                    if lane_error > 0:  # Desviado a la derecha
                        self.estado = LANE_CHANGE_LEFT
                    else:  # Desviado a la izquierda
                        self.estado = LANE_CHANGE_RIGHT
                else:
                    self.estado = RUTA_LIBRE
        else:
            self.estado = RUTA_LIBRE

        return self.estado

    def accion(self):
        """
        Retorna la acción correspondiente al estado actual
        """
        match self.estado:
            case 0:  # RUTA_LIBRE
                return 90, "RUTA LIBRE"

            case 1:  # STOP_LEJOS
                return 50, "STOP LEJOS"

            case 2:  # STOP_CERCA
                return 20, "STOP CERCA"

            case 3:  # EMERGENCIA
                return 0, "EMERGENCIA"

            case 4:  # SEMAFORO_ROJO
                return 0, "SEMAFORO ROJO"

            case 5:  # SEMAFORO_AMARILLO
                return 40, "SEMAFORO AMARILLO"

            case 6:  # PARKING_SEARCH
                return 30, "BUSCANDO ESTACIONAMIENTO"

            case 7:  # PARKING_MANEUVER
                return 15, "MANIOBRA DE ESTACIONAMIENTO"

            case 8:  # INTERSECTION_STOP
                return 0, "STOP INTERSECCION"

            case 9:  # LANE_CHANGE_LEFT
                return 60, "CAMBIO CARRIL IZQUIERDA"

            case 10:  # LANE_CHANGE_RIGHT
                return 60, "CAMBIO CARRIL DERECHA"

            case 11:  # OBSTACLE_AVOIDANCE
                return 25, "EVITANDO OBSTACULO"

            case _:
                return 90, "RUTA LIBRE"

    def iniciar_estacionamiento(self, vehicle_controller):
        """
        Inicia el proceso de estacionamiento usando el controlador del vehículo
        """
        if self.estado == RUTA_LIBRE:
            self.estado = PARKING_SEARCH
            vehicle_controller.iniciar_estacionamiento_paralelo()
            self.parking_attempts = 0

    def cancelar_estacionamiento(self, vehicle_controller):
        """
        Cancela el proceso de estacionamiento
        """
        if self.estado in [PARKING_SEARCH, PARKING_MANEUVER]:
            self.estado = RUTA_LIBRE
            vehicle_controller.finalizar_estacionamiento()

    def actualizar_parking(self, vehicle_controller, perception_output):
        """
        Actualiza el estado del parking basado en el progreso
        """
        if self.estado in [PARKING_SEARCH, PARKING_MANEUVER]:
            parking_status = vehicle_controller.get_estado_estacionamiento()

            if parking_status['complete']:
                self.estado = RUTA_LIBRE
                vehicle_controller.finalizar_estacionamiento()
            elif parking_status['phase'] == 'backup':
                self.estado = PARKING_MANEUVER

    def get_steering_ackerman(self, lane_error, velocidad):
        """
        Calcula el ángulo de dirección usando ecuación de Ackerman
        """
        # Parámetros del vehículo
        wheelbase = 0.25  # Distancia entre ejes (metros)
        track_width = 0.18  # Ancho de vía (metros)

        # Error de carril normalizado
        error_norm = lane_error / 320  # Asumiendo frame de 640px ancho

        # Control proporcional
        kp_steer = 0.5
        steering_angle = kp_steer * error_norm

        # Limitar ángulo de dirección
        max_steering_angle = np.radians(30)  # 30 grados máximo
        steering_angle = np.clip(steering_angle, -max_steering_angle, max_steering_angle)

        # Ackerman steering: ajustar ángulo basado en velocidad
        if velocidad > 0:
            # Ángulo exterior (ruedas delanteras)
            steering_angle_outer = np.arctan2(wheelbase, wheelbase/np.tan(steering_angle) + track_width/2)
            # Ángulo interior
            steering_angle_inner = np.arctan2(wheelbase, wheelbase/np.tan(steering_angle) - track_width/2)

            return steering_angle_outer, steering_angle_inner

        return 0, 0

    def get_estado_string(self):
        """
        Retorna descripción del estado actual
        """
        estados_str = {
            0: "RUTA_LIBRE",
            1: "STOP_LEJOS",
            2: "STOP_CERCA",
            3: "EMERGENCIA",
            4: "SEMAFORO_ROJO",
            5: "SEMAFORO_AMARILLO",
            6: "PARKING_SEARCH",
            7: "PARKING_MANEUVER",
            8: "INTERSECTION_STOP",
            9: "LANE_CHANGE_LEFT",
            10: "LANE_CHANGE_RIGHT",
            11: "OBSTACLE_AVOIDANCE"
        }
        return estados_str.get(self.estado, "UNKNOWN")