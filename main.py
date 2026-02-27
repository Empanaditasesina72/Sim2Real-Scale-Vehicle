# -*- coding: utf-8 -*-
import time
import matplotlib.pyplot as plt
import cv2
import numpy as np
import RPi.GPIO as GPIO
import serial

from pid.pid_controller import PID
from detector.detector import obtener_distancia
from fsm.state_machine import StateMachine
from feedback.adaptive_pid import AdaptivePID

# =====================================
# CONTROLADORES
# =====================================
pid = PID(0.6, 0.2, 0.05)
adaptativo = AdaptivePID()
fsm = StateMachine()

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

cv2.namedWindow("AUTO YOLO PID", cv2.WINDOW_NORMAL)

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
# UMBRALES
# =====================================
STOP_CERCA = 40
STOP_LEJOS = 120

TOF_FRENO_DELANTE = 300  # mm
TOF_ALERTA_ATRAS = 150   # mm

# =====================================
# VARIABLES DEL SISTEMA
# =====================================
velocidad = 80
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

        # DEBUG opcional
        # print("TOF:", dist_delante, dist_atras)

        # ===== YOLO =====
        dist_stop, semaforo, zona, cx = obtener_distancia(frame)

        # =====================================
        # CONTROL LEDs POR STOP VISUAL
        # =====================================
        if dist_stop is None:
            GPIO.output(LED_VERDE, GPIO.HIGH)
            GPIO.output(LED_AMARILLO, GPIO.LOW)
            GPIO.output(LED_ROJO, GPIO.LOW)

        elif dist_stop < STOP_CERCA:
            GPIO.output(LED_VERDE, GPIO.LOW)
            GPIO.output(LED_AMARILLO, GPIO.LOW)
            GPIO.output(LED_ROJO, GPIO.HIGH)

        elif dist_stop < STOP_LEJOS:
            GPIO.output(LED_VERDE, GPIO.LOW)
            GPIO.output(LED_AMARILLO, GPIO.HIGH)
            GPIO.output(LED_ROJO, GPIO.LOW)

        else:
            GPIO.output(LED_VERDE, GPIO.HIGH)
            GPIO.output(LED_AMARILLO, GPIO.LOW)
            GPIO.output(LED_ROJO, GPIO.LOW)

        # ===== FSM =====
        fsm.evaluar(dist_stop, semaforo)
        setpoint, estado_txt = fsm.accion()

        # ===== PRIORIDAD SEMAFORO =====
        if semaforo == "red":
            setpoint = 0
            estado_txt += " | ROJO"
        elif semaforo == "yellow":
            setpoint = min(setpoint, 30)
            estado_txt += " | AMARILLO"
        elif semaforo == "green":
            estado_txt += " | VERDE"

        # =====================================
        # SEGURIDAD ToF (ANTI-CHOQUE REAL)
        # =====================================
        if dist_delante is not None and dist_delante < TOF_FRENO_DELANTE:
            setpoint = 0
            estado_txt += " | STOP TOF DELANTE"

        if dist_atras is not None and dist_atras < TOF_ALERTA_ATRAS:
            estado_txt += " | OBSTACULO ATRAS"

        # ===== ERROR =====
        error = setpoint - velocidad

        # ===== PID ADAPTATIVO =====
        kp, ki, kd = adaptativo.actualizar(error)
        pid.update_gains(kp, ki, kd)

        # ===== PID =====
        control = pid.compute(error, dt)

        # ===== PLANTA =====
        velocidad += control * dt
        velocidad = max(0, min(100, velocidad))

        # ===== HISTORIAL =====
        hist_v.append(velocidad)
        hist_s.append(setpoint)
        hist_t.append(t)
        t += dt

        # ===== VISUAL =====
        cv2.putText(frame, f"Estado: {estado_txt}", (30, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

        cv2.putText(frame, f"Vel: {velocidad:.1f}", (30, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv2.putText(frame, f"Semaforo: {semaforo}", (30, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

        cv2.putText(frame, f"Zona: {zona}", (30, 160),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)

        cv2.putText(frame,
                    f"TOF D:{dist_delante} A:{dist_atras}",
                    (30, 200),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)

        cv2.imshow("AUTO YOLO PID", frame)

        # ===== SALIR =====
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

        time.sleep(dt)

finally:
    picam2.stop()
    cv2.destroyAllWindows()
    GPIO.cleanup()

    plt.plot(hist_t, hist_v, label="Velocidad")
    plt.plot(hist_t, hist_s, "--", label="Setpoint")
    plt.legend()
    plt.show()
