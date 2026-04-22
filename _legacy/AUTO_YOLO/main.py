# -*- coding: utf-8 -*-

import time
import matplotlib.pyplot as plt
import cv2
import numpy as np
import RPi.GPIO as GPIO
import pygame

import board
import busio
import adafruit_vl53l0x

from pid.pid_controller import PID
from detector.detector import obtener_distancia
from fsm.state_machine import StateMachine
from feedback.adaptive_pid import AdaptivePID
from motor.pwm_controller import PWMController

# 🔥 NUEVO
from vision.lane_detector import detectar_carril

# =====================================
# CONTROLADORES
# =====================================

pid = PID(0.6, 0.2, 0.05)
adaptativo = AdaptivePID()
fsm = StateMachine()

# =====================================
# CAMARA USB
# =====================================

usar_camara = False

try:
    cap = cv2.VideoCapture(0)

    if cap.isOpened():
        usar_camara = True
        print("✅ Webcam USB detectada")

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

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

velocidad = 0
dt = 0.1

# =====================================
# CONTROL
# =====================================

pygame.init()
pygame.joystick.init()

joystick = None
if pygame.joystick.get_count() > 0:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print("🎮 Control conectado")

activo = False

# =====================================
# LOOP
# =====================================

try:
    while True:

        pygame.event.pump()

        if joystick:
            if joystick.get_button(2):
                activo = True

            if joystick.get_button(0):
                activo = False
                motor(0, "stop")
                continue

        if not activo:
            motor(0, "stop")
            time.sleep(dt)
            continue

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

            # YOLO / señales
            dist_stop, semaforo, _, cx_obj = obtener_distancia(frame)

            # 🔥 CARRIL
            cx_carril, frame = detectar_carril(frame)

            if cx_carril is not None:
                centro = frame.shape[1] // 2
                error_dir = cx_carril - centro
            else:
                error_dir = 0

            print(f"Error carril: {error_dir}")

            cv2.imshow("AUTO YOLO PID", frame)
            cv2.waitKey(1)

        else:
            dist_stop = None
            semaforo = None

        # FSM
        fsm.evaluar(dist_stop, semaforo, dist_delante)
        setpoint, _ = fsm.accion()

        # PID velocidad
        error = setpoint - velocidad

        kp, ki, kd = adaptativo.actualizar(error)
        pid.update_gains(kp, ki, kd)

        control = pid.compute(error, dt)

        velocidad += control * dt
        velocidad = max(0, min(100, velocidad))

        velocidad = pwm_control.update(velocidad)

        if velocidad > 0:
            motor(velocidad, "adelante")
        else:
            motor(0, "stop")

        print(f"Vel={velocidad:.1f}")

        time.sleep(dt)

finally:

    if usar_camara:
        cap.release()
        cv2.destroyAllWindows()

    pwmA.stop()
    GPIO.cleanup()
