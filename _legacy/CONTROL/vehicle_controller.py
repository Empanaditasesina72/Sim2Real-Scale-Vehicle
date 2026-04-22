import numpy as np
import math

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.prev_error = 0
        self.integral_limit = 100  # Límite del integral

    def compute(self, error, dt):
        # Integral con windup protection
        self.integral += error * dt
        self.integral = np.clip(self.integral, -self.integral_limit, self.integral_limit)

        # Derivativo
        derivative = (error - self.prev_error) / dt if dt > 0 else 0

        # Output
        output = (
            self.kp * error +
            self.ki * self.integral +
            self.kd * derivative
        )

        self.prev_error = error
        return output

    def update_gains(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def reset(self):
        self.integral = 0
        self.prev_error = 0

class SimpleSteeringController:
    """
    Controlador de dirección simplificado para sistemas donde ambas ruedas
    delanteras giran al mismo ángulo (sin Ackerman independiente)
    """
    def __init__(self, wheelbase=0.25, max_steering_angle=np.radians(30)):
        """
        wheelbase: distancia entre ejes (metros)
        max_steering_angle: ángulo máximo de dirección (radianes)
        """
        self.wheelbase = wheelbase
        self.max_steering_angle = max_steering_angle

    def calculate_steering_angle(self, lane_error, frame_width=640):
        """
        Calcula el ángulo de dirección basado en el error de carril
        Para sistemas simples: ambas ruedas giran al mismo ángulo
        """
        # Normalizar error de carril (-1 a 1)
        error_norm = lane_error / (frame_width / 2)

        # Control proporcional simple
        kp_steer = 0.5  # Ganancia proporcional
        steering_angle = kp_steer * error_norm

        # Limitar ángulo máximo
        steering_angle = np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle)

        return steering_angle

    def get_turning_radius(self, steering_angle):
        """
        Calcula el radio de giro aproximado
        R = L / tan(δ) donde L es wheelbase, δ es steering_angle
        """
        if abs(steering_angle) < 1e-6:
            return float('inf')  # Recto

        return self.wheelbase / np.tan(steering_angle)

    def predict_trajectory(self, steering_angle, velocity, dt=0.1, steps=10):
        """
        Predice la trayectoria del vehículo (útil para parking)
        """
        trajectory = []
        x, y, theta = 0, 0, 0  # Posición inicial

        for _ in range(steps):
            if abs(steering_angle) < 1e-6:
                # Movimiento recto
                x += velocity * dt * np.cos(theta)
                y += velocity * dt * np.sin(theta)
            else:
                # Movimiento circular
                radius = self.get_turning_radius(steering_angle)
                omega = velocity / radius  # Velocidad angular

                x += radius * np.sin(theta + omega * dt) - radius * np.sin(theta)
                y -= radius * np.cos(theta + omega * dt) + radius * np.cos(theta)
                theta += omega * dt

            trajectory.append((x, y, theta))

        return trajectory

class ParkingController:
    """
    Controlador específico para maniobras de estacionamiento
    Algoritmos especializados sin Ackerman
    """
    def __init__(self, vehicle_width=0.18, parking_space_length=0.6):
        self.vehicle_width = vehicle_width
        self.parking_space_length = parking_space_length

        # Estados del parking
        self.parking_step = 0
        self.parking_phase = "search"  # "search", "align", "backup", "adjust", "complete"

    def parallel_parking_maneuver(self, lateral_distance, current_angle, vehicle_speed=0):
        """
        Algoritmo completo de estacionamiento paralelo en 3 fases
        """
        target_distance = self.vehicle_width + 0.1  # 10cm del bordillo

        if self.parking_phase == "search":
            # Fase 1: Buscar espacio paralelo
            if lateral_distance > target_distance + 0.2:  # Espacio suficiente
                self.parking_phase = "align"
                return np.radians(15)  # Giro suave para alinearse
            return 0  # Continuar recto

        elif self.parking_phase == "align":
            # Fase 2: Alinearse con el espacio
            error_distance = lateral_distance - target_distance
            kp_align = 0.4
            steering_correction = kp_align * error_distance

            if abs(error_distance) < 0.05:  # Alineado
                self.parking_phase = "backup"
                self.parking_step = 0

            return np.clip(steering_correction, -np.radians(25), np.radians(25))

        elif self.parking_phase == "backup":
            # Fase 3: Retroceder en maniobra de 3 puntos
            if self.parking_step == 0:
                # Primer giro hacia atrás
                if current_angle < np.radians(30):
                    return np.radians(35)  # Girar hacia atrás
                else:
                    self.parking_step = 1
                    return np.radians(35)  # Mantener giro

            elif self.parking_step == 1:
                # Enderezar
                if abs(current_angle) > np.radians(5):
                    return -np.radians(20)  # Contragiro
                else:
                    self.parking_step = 2
                    return 0

            elif self.parking_step == 2:
                # Segundo giro para alinear
                if current_angle > -np.radians(25):
                    return -np.radians(30)  # Girar hacia adelante
                else:
                    self.parking_phase = "adjust"
                    return -np.radians(30)

        elif self.parking_phase == "adjust":
            # Fase 4: Ajustes finales
            error_distance = lateral_distance - target_distance
            kp_adjust = 0.2
            steering_correction = kp_adjust * error_distance

            if abs(error_distance) < 0.02:  # Perfectamente alineado
                self.parking_phase = "complete"

            return np.clip(steering_correction, -np.radians(15), np.radians(15))

        return 0  # Parking completo o error

    def perpendicular_parking(self, distance_to_space, angle_to_space):
        """
        Estacionamiento perpendicular simplificado
        """
        # Control simple para entrar en espacio perpendicular
        kp_distance = 0.4
        kp_angle = 0.3

        steering_angle = kp_distance * distance_to_space + kp_angle * angle_to_space
        steering_angle = np.clip(steering_angle, -np.radians(30), np.radians(30))

        return steering_angle

    def reverse_parking_assist(self, obstacle_distance, target_distance=0.2):
        """
        Asistente de estacionamiento en reversa con sensores
        """
        error = obstacle_distance - target_distance
        kp_reverse = 0.5

        # Control proporcional para distancia
        steering_correction = kp_reverse * error

        # Limitar ángulo para parking
        return np.clip(steering_correction, -np.radians(40), np.radians(40))

    def reset_parking(self):
        """
        Reinicia el proceso de estacionamiento
        """
        self.parking_step = 0
        self.parking_phase = "search"

    def get_parking_status(self):
        """
        Retorna el estado actual del parking
        """
        return {
            'phase': self.parking_phase,
            'step': self.parking_step,
            'complete': self.parking_phase == "complete"
        }

class VehicleController:
    def __init__(self):
        # Controladores PID
        self.pid_velocity = PIDController(0.6, 0.2, 0.05)
        self.pid_steering = PIDController(0.8, 0.1, 0.2)

        # Controladores simplificados (sin Ackerman)
        self.simple_steering = SimpleSteeringController()
        self.parking_controller = ParkingController()

        # Estado del vehículo
        self.velocidad_actual = 0
        self.velocidad_max = 100
        self.velocidad_min = 0

        # Parámetros de control
        self.dt = 0.1

        # Modo de control
        self.control_mode = "normal"  # "normal", "parking_parallel", "parking_perp"

    def controlar_velocidad(self, setpoint_velocidad):
        """
        Controla la velocidad del vehículo usando PID
        """
        error = setpoint_velocidad - self.velocidad_actual
        control = self.pid_velocity.compute(error, self.dt)

        # Aplicar control
        self.velocidad_actual += control * self.dt
        self.velocidad_actual = np.clip(self.velocidad_actual,
                                      self.velocidad_min,
                                      self.velocidad_max)

        return self.velocidad_actual

    def controlar_direccion_simple(self, lane_error, frame_width=640):
        """
        Control de dirección simplificado para navegación normal
        """
        if self.control_mode == "parking_parallel":
            # Modo parking paralelo - usar algoritmo específico
            # Nota: necesitaríamos sensores laterales para distancia real
            steering_angle = self.simple_steering.calculate_steering_angle(lane_error, frame_width)
        elif self.control_mode == "parking_perp":
            # Modo parking perpendicular
            steering_angle = self.simple_steering.calculate_steering_angle(lane_error, frame_width)
        else:
            # Modo normal - lane keeping
            steering_angle = self.simple_steering.calculate_steering_angle(lane_error, frame_width)

        return steering_angle

    def iniciar_estacionamiento_paralelo(self):
        """
        Inicia maniobra de estacionamiento paralelo
        """
        self.control_mode = "parking_parallel"
        self.parking_controller.reset_parking()

    def maniobra_estacionamiento_paralelo(self, distancia_lateral, angulo_actual, velocidad=0):
        """
        Ejecuta maniobra de estacionamiento paralelo
        """
        self.control_mode = "parking_parallel"
        steering_angle = self.parking_controller.parallel_parking_maneuver(
            distancia_lateral, angulo_actual, velocidad
        )
        return steering_angle

    def finalizar_estacionamiento(self):
        """
        Regresa al modo normal después del parking
        """
        self.control_mode = "normal"
        self.parking_controller.reset_parking()
        self.pid_steering.reset()

    def get_estado_estacionamiento(self):
        """
        Retorna el estado del proceso de estacionamiento
        """
        return self.parking_controller.get_parking_status()

    def predecir_trayectoria(self, steering_angle, velocity, steps=10):
        """
        Predice la trayectoria del vehículo (útil para planning)
        """
        return self.simple_steering.predict_trajectory(steering_angle, velocity, self.dt, steps)

    def reset_controladores(self):
        """
        Reinicia los controladores PID
        """
        self.pid_velocity.reset()
        self.pid_steering.reset()

    def actualizar_parametros(self, velocidad_max=None, dt=None):
        """
        Actualiza parámetros del controlador
        """
        if velocidad_max is not None:
            self.velocidad_max = velocidad_max
        if dt is not None:
            self.dt = dt

    def get_estado_control(self):
        """
        Retorna el estado actual del control
        """
        return {
            'velocidad': self.velocidad_actual,
            'modo_control': self.control_mode,
            'parking_status': self.get_estado_estacionamiento(),
            'velocity_pid_gains': (self.pid_velocity.kp, self.pid_velocity.ki, self.pid_velocity.kd),
            'steering_pid_gains': (self.pid_steering.kp, self.pid_steering.ki, self.pid_steering.kd)
        }