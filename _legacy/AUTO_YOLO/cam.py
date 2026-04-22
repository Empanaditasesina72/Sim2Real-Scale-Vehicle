#-*- coding: utf-8 -*-

import time
import cv2
import numpy as np
import RPi.GPIO as GPIO

import board
import busio
import adafruit_vl53l0x

from pid.pid_controller import PID
from detector.detector import obtener_distancia
from fsm.state_machine import StateMachine
from feedback.adaptive_pid import AdaptivePID
from motor.pwm_controller import PWMController

# =====================================
# CONTROLADORES
# =====================================

pid = PID(0.6, 0.2, 0.05)
adaptativo = AdaptivePID()
fsm = StateMachine()

# =====================================
# CAMARA
# =====================================

usar_camara = False

try:
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

    if cap.isOpened():
        usar_camara = True
        print("✅ Webcam USB detectada")

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        cap.set(cv2.CAP_PROP_FPS, 30)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        cv2.namedWindow("AUTO YOLO PID", cv2.WINDOW_NORMAL)
    else:
        print("⚠️ Webcam NO detectada → simulacion")

except:
    print("⚠️ Error con la webcam → simulacion")

# =====================================
# GPIO
# =====================================

GPIO.setmode(GPIO.BCM)

PWM_PIN = 18
DIR_PIN = 23

GPIO.setup(DIR_PIN, GPIO.OUT)
GPIO.setup(PWM_PIN, GPIO.OUT)

pwmA = GPIO.PWM(PWM_PIN, 1000)
pwmA.start(0)

pwm_control = PWMController(step=3)

def motor(vel, direccion):

    if direccion == "adelante":
        GPIO.output(DIR_PIN, GPIO.HIGH)

    elif direccion == "atras":
        GPIO.output(DIR_PIN, GPIO.LOW)

    elif direccion == "stop":
        pwmA.ChangeDutyCycle(0)
        return

    pwmA.ChangeDutyCycle(vel)

# =====================================
# SENSOR
# =====================================

try:
    i2c = busio.I2C(board.SCL, board.SDA)
    sensor = adafruit_vl53l0x.VL53L0X(i2c)
    print("Sensor OK")
except:
    sensor = None
    print("⚠️ Sensor no detectado")

dist_delante = None

# =====================================
# PARAMETROS
# =====================================

VEL_MAX = 4
velocidad = 0
dt = 0.1

# =====================================
# MEMORIA DETECCION
# =====================================

ultima_deteccion_stop = None
ultima_zona = "none"
tiempo_ultima_deteccion = 0

TIEMPO_MEMORIA = 2.0

# =====================================
# VARIABLES
# =====================================

modo_arranque = False
tiempo_stop = 0

modo_ignore_stop = False
tiempo_ignore = 0

# =====================================
# FUNCION DE FRENADO
# =====================================

def calcular_freno(dist):

    if dist is None:
        return 0.2

    if dist > 200:
        return 0.1
    elif dist > 150:
        return 0.2
    elif dist > 120:
        return 0.4
    elif dist > 90:
        return 0.7
    elif dist > 70:
        return 1.0
    elif dist > 55:
        return 1.5
    else:
        return 2.0

# =====================================
# LOOP
# =====================================

try:

    while True:

        # SENSOR
        if sensor:
            try:
                dist_delante = sensor.range
            except:
                dist_delante = None

        # CAMARA
        if usar_camara:

            ret, frame = cap.read()
            if not ret:
                continue

            dist_stop, semaforo, zona_objeto, cx = obtener_distancia(frame)

            # 🔥 IGNORE TOTAL DE STOP
            if modo_ignore_stop:
                dist_stop = None

            # MEMORIA
            if dist_stop is not None and zona_objeto in ["izquierda", "derecha", "centro"]:
                ultima_deteccion_stop = dist_stop
                ultima_zona = zona_objeto
                tiempo_ultima_deteccion = time.time()
            else:
                if time.time() - tiempo_ultima_deteccion < TIEMPO_MEMORIA:
                    dist_stop = ultima_deteccion_stop
                    zona_objeto = ultima_zona

            cv2.imshow("AUTO YOLO PID", frame)
            cv2.waitKey(1)

        else:
            dist_stop = None
            semaforo = None
            zona_objeto = "none"

        # =====================================
        # VELOCIDAD DINAMICA
        # =====================================

        if dist_stop is None:
            VEL_MAX_DIN = 4
        elif dist_stop > 180:
            VEL_MAX_DIN = 3
        elif dist_stop > 120:
            VEL_MAX_DIN = 2
        elif dist_stop > 80:
            VEL_MAX_DIN = 1
        else:
            VEL_MAX_DIN = 0

        # FSM
        fsm.evaluar(dist_stop, semaforo, dist_delante)
        setpoint, estado_txt = fsm.accion()

        # =====================================
        # TIMER IGNORE
        # =====================================

        if modo_ignore_stop:
            if time.time() - tiempo_ignore >= 5:
                modo_ignore_stop = False
                print("✅ STOP reactivado")

        # =====================================
        # STOP TOTAL
        # =====================================

        if (not modo_ignore_stop and
            dist_stop is not None and
            dist_stop < 60 and
            zona_objeto in ["izquierda", "derecha","centro"]):

            motor(0, "stop")
            velocidad = 0

            if tiempo_stop == 0:
                tiempo_stop = time.time()
                print("🛑 STOP TOTAL")

            elif time.time() - tiempo_stop >= 5:
                modo_arranque = True
                modo_ignore_stop = True
                tiempo_ignore = time.time()
                tiempo_stop = 0
                print("🚀 REANUDANDO + IGNORE")

            time.sleep(dt)
            continue

        # =====================================
        # ARRANQUE SUAVE
        # =====================================

        if modo_arranque:
            velocidad += 0.3
            velocidad = min(VEL_MAX, velocidad)

            motor(velocidad, "adelante")

            if velocidad >= VEL_MAX:
                modo_arranque = False

            time.sleep(dt)
            continue

        # =====================================
        # FRENADO PROGRESIVO
        # =====================================

        if (not modo_ignore_stop and
            dist_stop is not None and
            zona_objeto in ["izquierda", "derecha", "centro"]):

            freno = calcular_freno(dist_stop)

            if velocidad > VEL_MAX_DIN:
                velocidad -= freno
            else:
                velocidad += 0.1

            velocidad = max(0, min(VEL_MAX_DIN, velocidad))

            motor(velocidad, "adelante")

            print(f"🚧 Dist={dist_stop:.1f} | Freno={freno:.2f} | Vel={velocidad:.2f}")

            time.sleep(dt)
            continue

        # =====================================
        # PID NORMAL
        # =====================================

        error = setpoint - velocidad

        kp, ki, kd = adaptativo.actualizar(error)
        pid.update_gains(kp, ki, kd)

        control = pid.compute(error, dt)

        velocidad += control * dt
        velocidad = max(0, min(VEL_MAX_DIN, velocidad))

        velocidad = pwm_control.update(velocidad)

        if velocidad > 0:
            motor(velocidad, "adelante")
        else:
            motor(0, "stop")

        print(f"Vel={velocidad:.2f}")

        time.sleep(dt)

finally:

    if usar_camara:
        cap.release()
        cv2.destroyAllWindows()

    pwmA.stop()
    GPIO.cleanup()
