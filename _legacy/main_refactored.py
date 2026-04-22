# -*- coding: utf-8 -*-
import time
import matplotlib.pyplot as plt
import cv2
import numpy as np
import RPi.GPIO as GPIO
import serial

# =====================================
# IMPORTACIÓN DE MÓDULOS REFACTORIZADOS
# =====================================
from CAMARA.YOLO.objeto.objeto_detector import ObjetoDetector
from CAMARA.YOLO.semaforo.semaforo_detector import SemaforoDetector
from CAMARA.YOLO.señal.senal_detector import SenalDetector
from CAMARA.lane_detection.lane_center.lane_center_detector import LaneCenterDetector
from CAMARA.lane_detection.lane_curvature.lane_curvature_detector import LaneCurvatureDetector
from PERCEPTION_OUTPUT.perception_output import PerceptionOutput
from STATE_MACHINE.state_machine import StateMachine
from CONTROL.vehicle_controller import VehicleController

# =====================================
# INICIALIZACIÓN DE COMPONENTES
# =====================================
# Detectores YOLO
objeto_detector = ObjetoDetector()
semaforo_detector = SemaforoDetector()
senal_detector = SenalDetector()

# Detectores de carril
lane_center_detector = LaneCenterDetector()
lane_curvature_detector = LaneCurvatureDetector()

# Salida de percepción
perception_output = PerceptionOutput()

# Máquina de estados
fsm = StateMachine()

# Controlador del vehículo
vehicle_controller = VehicleController()

# =====================================
# PICAMERA2 (Raspberry Pi CSI)
# =====================================
from picamera2 import Picamera2

picam2 = Picamera2()
config = picam2.create_preview_configuration(
    main={"format": "BGR888", "size": (640, 480)}
)
picam2.configure(config)
picam2.start()

print("Camara iniciada con Picamera2")
print("Presiona Q para salir")

cv2.namedWindow("AUTO YOLO REFACTORIZADO", cv2.WINDOW_NORMAL)

# =====================================
# GPIO LEDs
# =====================================
GPIO.setmode(GPIO.BCM)

LED_ROJO = 17
LED_AMARILLO = 27
LED_VERDE = 22

GPIO.setup(LED_ROJO, GPIO.OUT)
GPIO.setup(LED_AMARILLO, GPIO.OUT)
GPIO.setup(LED_VERDE, GPIO.OUT)

# =====================================
# SERIAL VL53L0X
# =====================================
try:
    ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=0.01)
    print("Serial sensores conectado")
except:
    ser = None
    print("WARNING: No se pudo abrir serial")

# Variables globales ToF
dist_delante = None
dist_atras = None

# =====================================
# VARIABLES DEL SISTEMA
# =====================================
dt = 0.1
t = 0

hist_v = []
hist_s = []
hist_t = []

# =====================================
# LOOP PRINCIPAL
# =====================================
try:
    while True:
        # ===== CAPTURA DE FRAME =====
        frame = picam2.capture_array()

        if frame is None:
            print("Frame vacio")
            time.sleep(0.05)
            continue

        # Fix canales
        if len(frame.shape) == 3 and frame.shape[2] == 4:
            frame = frame[:, :, :3]

        # Corrección color
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # =====================================
        # LECTURA SERIAL SENSORES ToF
        # =====================================
        if ser and ser.in_waiting:
            try:
                linea = ser.readline().decode().strip()
                if linea:
                    d1, d2 = linea.split(",")
                    dist_delante = int(d1)
                    dist_atras = int(d2)
            except:
                pass

        # =====================================
        # PERCEPCIÓN - CAMARA
        # =====================================

        # 1. Detección de objetos
        objetos = objeto_detector.detectar_objetos(frame)

        # 2. Detección de semáforos
        # Nota: Para semáforos necesitaríamos clasificar primero las detecciones YOLO
        # Por ahora asumimos que el semáforo viene de objetos
        semaforo_info = None
        for obj in objetos:
            if obj['tipo'] in ['red', 'yellow', 'green', 'traffic_light']:
                semaforo_info = semaforo_detector.procesar_semaforo(frame, obj['bbox'])
                break

        # 3. Detección de señales
        senales = senal_detector.detectar_senales(frame)

        # 4. Detección de carriles
        lines = lane_center_detector.detectar_lineas(frame)
        lane_center = lane_center_detector.calcular_centro_carril(frame, lines)
        lane_curvature = lane_curvature_detector.calcular_curvatura(frame, lines)

        # =====================================
        # ACTUALIZACIÓN DE PERCEPCIÓN
        # =====================================
        perception_output.actualizar_objetos(objetos)
        perception_output.actualizar_semaforo(semaforo_info)
        perception_output.actualizar_lane_error(lane_center, frame.shape[1])
        perception_output.detectar_interseccion(senales)

        # Agregar distancia ToF si está disponible
        if dist_delante is not None and perception_output.distancia_obj is None:
            perception_output.distancia_obj = dist_delante / 10  # Convertir mm a cm

        # =====================================
        # MÁQUINA DE ESTADOS
        # =====================================
        estado_actual = fsm.evaluar(perception_output.obtener_estado_percepcion())
        setpoint_velocidad, estado_txt = fsm.accion()

        # Actualizar estado del parking si está activo
        fsm.actualizar_parking(vehicle_controller, perception_output.obtener_estado_percepcion())

        # =====================================
        # CONTROL DE VELOCIDAD
        # =====================================
        velocidad_actual = vehicle_controller.controlar_velocidad(setpoint_velocidad)

        # =====================================
        # ACTIVACIÓN DE PARKING (automática o manual)
        # =====================================
        # Activar parking si se detecta un espacio amplio (lógica simplificada)
        parking_status = vehicle_controller.get_estado_estacionamiento()
        if (estado_actual == 0 and  # RUTA_LIBRE
            perception_output.lane_error < 20 and  # Bien centrado en carril
            not parking_status['complete']):  # No está completando parking

            # Simular detección de espacio de parking (podrías usar sensores laterales)
            espacio_parking_detectado = True  # Simulado

            if espacio_parking_detectado and vehicle_controller.control_mode == "normal":
                print("¡Espacio de estacionamiento detectado! Iniciando maniobra...")
                fsm.iniciar_estacionamiento(vehicle_controller)

        # =====================================
        # CONTROL DE DIRECCIÓN (SIMPLIFICADO - Sin Ackerman)
        # =====================================
        if vehicle_controller.control_mode == "parking_parallel":
            # Modo estacionamiento paralelo
            steering_angle = vehicle_controller.maniobra_estacionamiento_paralelo(
                distancia_lateral=0.3,  # Simulado - necesitarías sensor lateral real
                angulo_actual=0,       # Simulado - necesitarías IMU
                velocidad=velocidad_actual if velocidad_actual < 0 else -10  # Retroceder lentamente
            )
        else:
            # Modo normal
            steering_angle = vehicle_controller.controlar_direccion_simple(
                perception_output.lane_error, frame.shape[1]
            )

        # =====================================
        # CONTROL LEDs POR ESTADO
        # =====================================
        estado_percepcion = perception_output.obtener_estado_percepcion()

        if estado_percepcion['distancia_obj'] is not None and estado_percepcion['distancia_obj'] < 25:
            GPIO.output(LED_VERDE, GPIO.LOW)
            GPIO.output(LED_AMARILLO, GPIO.LOW)
            GPIO.output(LED_ROJO, GPIO.HIGH)
        elif estado_percepcion['semaforo'] == "red":
            GPIO.output(LED_VERDE, GPIO.LOW)
            GPIO.output(LED_AMARILLO, GPIO.LOW)
            GPIO.output(LED_ROJO, GPIO.HIGH)
        elif estado_percepcion['semaforo'] == "yellow":
            GPIO.output(LED_VERDE, GPIO.LOW)
            GPIO.output(LED_AMARILLO, GPIO.HIGH)
            GPIO.output(LED_ROJO, GPIO.LOW)
        else:
            GPIO.output(LED_VERDE, GPIO.HIGH)
            GPIO.output(LED_AMARILLO, GPIO.LOW)
            GPIO.output(LED_ROJO, GPIO.LOW)

        # =====================================
        # VISUALIZACIÓN
        # =====================================

        # Dibujar detecciones
        frame = objeto_detector.dibujar_detecciones(frame, objetos)
        if semaforo_info:
            frame = semaforo_detector.dibujar_semaforo(frame, semaforo_info)
        frame = lane_center_detector.dibujar_centro_carril(frame, lane_center)
        frame = lane_curvature_detector.dibujar_curvatura(frame, lane_curvature)

        # Información de texto
        cv2.putText(frame, f"Estado: {estado_txt}", (30, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

        cv2.putText(frame, f"Vel: {velocidad_actual:.1f}", (30, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv2.putText(frame, f"Steering: {np.degrees(steering_angle):.1f}°", (30, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

        cv2.putText(frame, perception_output.debug_info(), (30, 160),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        # Mostrar estado del parking
        parking_info = f"Parking: {parking_status['phase']} ({parking_status['step']})"
        cv2.putText(frame, parking_info, (30, 200),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 255), 2)

        cv2.putText(frame,
                    f"TOF D:{dist_delante} A:{dist_atras}",
                    (30, 240),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)

        cv2.imshow("AUTO YOLO REFACTORIZADO", frame)

        # ===== CONTROLES MANUALES =====
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("p"):  # Tecla P para activar parking
            if vehicle_controller.control_mode == "normal":
                print("Activando estacionamiento manualmente...")
                fsm.iniciar_estacionamiento(vehicle_controller)
            else:
                print("Cancelando estacionamiento...")
                fsm.cancelar_estacionamiento(vehicle_controller)
        elif key == ord("r"):  # Tecla R para reset
            vehicle_controller.finalizar_estacionamiento()
            fsm.estado = 0  # RUTA_LIBRE

        time.sleep(dt)

        # ===== HISTORIAL =====
        hist_v.append(velocidad_actual)
        hist_s.append(setpoint_velocidad)
        hist_t.append(t)
        t += dt

finally:
    picam2.stop()
    cv2.destroyAllWindows()
    GPIO.cleanup()

    plt.plot(hist_t, hist_v, label="Velocidad")
    plt.plot(hist_t, hist_s, "--", label="Setpoint")
    plt.legend()
    plt.show()