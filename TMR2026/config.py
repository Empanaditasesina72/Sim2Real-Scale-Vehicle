# -*- coding: utf-8 -*-
"""
config.py — Parámetros globales del sistema TMR 2026.
Todos los valores físicos, pines y ganancias PID viven aquí.
NO importar hardware real desde este módulo.
"""

# ============================================================
# HARDWARE PINS  (RPi.GPIO, numeración BCM)
# ============================================================
# --- IBT-2 H-Bridge ---
# R_EN y L_EN conectados a 3.3V (siempre habilitado — sin GPIO de enable)
PIN_MOTOR_RPWM = 18   # GPIO 18, Pin 12 — PWM avance
PIN_MOTOR_LPWM = 13   # GPIO 13, Pin 33 — PWM reversa
MOTOR_PWM_FREQ = 1000 # Hz

# --- LEDs status (opcionales) ---
PIN_LED_STOP   = 25   # Parpadea durante parada en STOP
PIN_LED_STATUS = 26   # Estado general del sistema

# --- LEDs de señalización vehicular (direccionales + freno) -------------------
# Cableado físico actual del coche:
#   Direccional IZQ (ámbar front + rojo rear) → Pin 11 / GPIO 17
#   Direccional DER (ámbar front + rojo rear) → Pin 29 / GPIO 5
#   Luces de freno (2 LEDs rojos en paralelo) → Pin 31 / GPIO 6
# El PIN_TOF_XSHUT_FRONT se movió a GPIO 24 para liberar GPIO 17.
PIN_LED_TURN_LEFT  = 17
PIN_LED_TURN_RIGHT = 5
PIN_LED_BRAKE      = 6
SIGNAL_BLINK_HZ    = 2.0   # Parpadeo direccionales / hazard (reglamento TMR)

# ============================================================
# I²C BUS 3  (GPIO 0=SDA, GPIO 1=SCL) — PCA9685 servo
# dtoverlay=i2c-gpio,bus=3,i2c_gpio_sda=0,i2c_gpio_scl=1
# ============================================================

# ============================================================
# I²C BUS 4  (GPIO 23=SDA, GPIO 22=SCL) — VL53L0X sensores
# dtoverlay=i2c-gpio,bus=4,i2c_gpio_sda=23,i2c_gpio_scl=22
# ============================================================
# Dos sensores VL53L0X en el mismo bus — se diferencian por XSHUT.
# El frontal NO está cableado actualmente (GPIO 24 reservado para futuro);
# distance_sensor.py degrada con gracia a "rear-only" si el frontal no responde.
PIN_TOF_XSHUT_FRONT = 24   # GPIO 24, Pin 18 — libre, listo para futuro lidar frontal
PIN_TOF_XSHUT_REAR  = 27   # GPIO 27, Pin 13
TOF_ADDR_FRONT      = 0x30 # dirección cambiada al inicializar
TOF_ADDR_REAR       = 0x29 # dirección por defecto

# --- PCA9685 ---
PCA9685_I2C_ADDR   = 0x40   # Dirección por defecto
PCA9685_PWM_FREQ   = 50     # Hz (estándar servo analógico)
SERVO_CHANNEL      = 15     # Canal MG90s en la placa (verificado en Pi)

# --- Servo MG90s ---
SERVO_MIN_PULSE_US  = 500   # µs → ~0°
SERVO_MAX_PULSE_US  = 2500  # µs → ~180°
SERVO_CENTER_ANGLE  = 90.0  # grados — ruedas al frente
SERVO_MIN_ANGLE     = 58.0  # grados — giro máximo izquierda (reducido para no trabarse)
SERVO_MAX_ANGLE     = 122.0 # grados — giro máximo derecha  (reducido para no trabarse)

# El servo está montado al revés: con joystick a la izquierda el coche giraba a
# la derecha. STEERING_INVERTED corrige esto a nivel de driver — todos los modos
# (MANUAL, AUTONOMOUS, VISION) ven la misma orientación lógica:
# 90° = recto, <90° = izquierda, >90° = derecha.
STEERING_INVERTED   = True

# ============================================================
# PI AI CAMERA  (Sony IMX500 — aceleración NPU on-chip)
# ============================================================
IMX500_MODEL_PATH = "/usr/share/imx500-models/imx500_network_efficientdet_lite0_pp.rpk"

# ── NPU on-chip con el modelo PROPIO (tmr_signs) ─────────────
# Si USE_IMX500_NPU=True y existe el .rpk, main.py corre la detección de
# señales DENTRO de la cámara (vision/imx500_detector.py) y la CPU queda
# libre. Si el .rpk no existe o falla, cae automáticamente al camino CPU
# (CameraStream + SignDetector NCNN) sin romper nada.
# Generar el .rpk EN LA PI (o cualquier Linux): python tools/export_imx500.py
USE_IMX500_NPU     = True
IMX500_RPK_PATH    = "weights/tmr_signs_imx500.rpk"
IMX500_LABELS_PATH = "weights/tmr_signs_imx500_labels.txt"
# Confianza para el modelo INT8 del NPU. La cuantización puede mover las
# confianzas respecto al .pt — calibrar en pista (arranca igual que CPU).
IMX500_CONF        = 0.55

CAMERA_WIDTH  = 640
CAMERA_HEIGHT = 480
CAMERA_FPS    = 30

# ── Calibración de imagen ────────────────────────────────────
# AWB: 0=Auto 1=Incandescent 2=Tungsten 3=Fluorescent 4=Indoor 5=Daylight
# Para competencia en interior con luz artificial → Indoor(4) o Fluorescent(3)
CAMERA_AWB_MODE   = 4      # Indoor — corrige el tono azulado en interiores
CAMERA_CONTRAST   = 1.5    # [0–32]  más contraste para bordes definidos
CAMERA_SATURATION = 1.8    # [0–32]  más saturación para colores vivos (rojo del STOP)
CAMERA_SHARPNESS  = 4.0    # [0–16]  imagen más nítida para detección
CAMERA_DENOISE    = 2      # 0=Off 2=CDN_Fast 3=CDN_HQ
CAMERA_BUFFERS    = 6      # frames en buffer — Pi 5 16 GB tiene RAM de sobra

# ── Detección ────────────────────────────────────────────────
# Umbral de confianza para aceptar detecciones del NPU
DETECTION_CONFIDENCE = 0.28   # bajo para detectar señales impresas

# Filtro temporal: cuántos frames consecutivos necesita una detección
# para ser "confirmada" — elimina falsos positivos de un solo frame
DETECTION_MIN_FRAMES = 2

# Nombres de clases COCO que interesan al TMR
# (deben coincidir con el modelo cargado en el IMX500)
CLASSES_OF_INTEREST = {
    "stop sign"    : "STOP",
    "traffic light": "SEMAFORO",
    "person"       : "PERSONA",
    "car"          : "AUTO",
}

# ============================================================
# DIMENSIONES DEL VEHÍCULO  (metros, escala 1:10)
# ============================================================
WHEELBASE    = 0.258   # Distancia entre ejes
TRACK_WIDTH  = 0.172   # Ancho entre ruedas delanteras
CAR_LENGTH   = 0.350   # Largo total (medido: 35 cm)
CAR_WIDTH    = 0.200   # Ancho total (medido: 20 cm)
CAMERA_HEIGHT_M = 0.22 # Altura de la cámara desde el piso (medido: 22 cm)
                       # Usado para calibrar la transformación BEV en lane_pipeline.py

MAX_STEERING_ANGLE_DEG = 35.0  # Límite físico del servo en grados desde centro

# ============================================================
# DIMENSIONES DE LA PISTA  (metros)
# ============================================================
# Ancho del carril medido de línea a línea, INCLUYENDO el grosor de las líneas.
# Con CAR_WIDTH=0.20 m, el carro tiene 0.17 m de margen lateral por carril
# (8.5 cm a cada lado si va centrado).
LANE_WIDTH_M = 0.54

# ============================================================
# VL53L0X  (sensor ToF frontal)
# ============================================================
TOF_TIMING_BUDGET_US = 20_000  # 20 ms — balance velocidad/precisión
TOF_MAX_RANGE_MM     = 1_200   # mm — fuera de rango = None
TOF_POLL_INTERVAL_S  = 0.020   # 50 Hz lectura del sensor

# ============================================================
# GANANCIAS PID
# ============================================================
# Steering (error de carril → ángulo servo)
STEER_KP = 0.09
STEER_KI = 0.002
STEER_KD = 0.025

# Velocidad (aproximación a señal STOP)
VEL_STOP_KP = 0.035   # salida en % PWM por mm de error
VEL_STOP_KI = 0.001
VEL_STOP_KD = 0.008

# ============================================================
# MODO AUTÓNOMO — VELOCIDADES Y UMBRALES
# ============================================================
# Velocidades conservadoras — subir de 5 en 5 según pruebas en pista
SPEED_STRAIGHT   = 22   # % PWM en rectas
SPEED_CURVE      = 15   # % PWM en curvas
SPEED_APPROACH   = 10   # % PWM al aproximarse a señal

# Umbral de curvatura para reducir velocidad (radianes del error de perspectiva)
CURVE_THRESHOLD_RAD = 0.30

# Umbral de error de carril para detectar "carril perdido"
LANE_LOST_THRESHOLD_PX = 280

# Confianza mínima del detector de carril para que el autónomo avance
# Por debajo de este valor = coche fuera de la pista → freno
# Bajado de 0.30 → 0.20 para aceptar tramos con una sola línea visible
LANE_MIN_CONFIDENCE = 0.20

# ============================================================
# COMPORTAMIENTO SEÑAL STOP
# ============================================================
STOP_BRAKE_START_MM  = 700   # mm — empieza frenado progresivo
STOP_TARGET_MM       = 270   # mm — distancia final de parada (≤30 cm regla TMR)
STOP_TOLERANCE_MM    = 30    # mm — ventana de aceptación
STOP_WAIT_SEC        = 5.0   # segundos de pausa obligatoria
STOP_LED_BLINK_HZ    = 2.0   # frecuencia de parpadeo LED

# Dimensiones reales de la señal STOP (medidas 2026-05-25):
#   • Octágono rojo .................... 4 cm de alto  ← LO QUE EL DETECTOR MIDE
#   • Octágono + tapita + palo (total) . 17 cm
# El detector pinhole solo "ve" el octágono rojo (su HSV es lo único filtrado),
# así que aquí va únicamente la altura del octágono.
# Si cambias la señal por otra de tamaño distinto, ACTUALIZA aquí:
#   • 4 cm  (mini sobre cono)                  → 0.04
#   • 18 cm (señal de competencia TMR oficial) → 0.18
STOP_SIGN_REAL_HEIGHT_M  = 0.04   # metros — altura del octágono rojo real
STOP_SIGN_TOTAL_HEIGHT_M = 0.17   # metros — solo informativo (palo+tapita+octágono)
CAMERA_FOCAL_LENGTH_PX   = 490.0  # píxeles — calibrar con tablero si es posible

# ============================================================
# EMERGENCIA
# ============================================================
EMERGENCY_STOP_MM = 120  # mm — parada de emergencia por obstáculo frontal

# ============================================================
# ESTACIONAMIENTO EN BATERÍA
# ============================================================
PARK_SEARCH_SPEED   = 15   # % PWM durante búsqueda
PARK_MANEUVER_SPEED = 10   # % PWM durante maniobra
PARK_MIN_GAP_MM     = 520  # mm — mínimo hueco (ToF, si disponible)
PARK_TARGET_GAP_MM  = 600  # mm — ancho nominal del espacio

# Tiempos calibrados de la maniobra (ajustar en pista)
PARK_OVERSHOOT_SEC        = 1.2  # avanzar tras detectar el inicio del hueco
PARK_REVERSE_LOCK_SEC     = 2.5  # reversa con giro completo
PARK_REVERSE_STRAIGHT_SEC = 1.0  # reversa derecho para centrar

# Detección de hueco por cámara:
# El hueco se confirma cuando no hay AUTO en la zona lateral del frame
# durante al menos este tiempo (evita falsos positivos)
PARK_GAP_CAMERA_MIN_SEC   = 0.4  # segundos sin AUTO en zona lateral
# Fracción del ancho del frame que define "zona lateral derecha"
PARK_GAP_CAMERA_ZONE      = 0.55 # bbox con cx > W*0.55 = lado derecho

# ============================================================
# MANIOBRA DE REBASE (obstáculos estáticos y en movimiento)
# ============================================================
# Un AUTO se considera obstáculo en nuestro carril si:
OVERTAKE_MIN_BBOX_AREA    = 2500  # px² — evita rebasar objetos lejanos
OVERTAKE_LANE_RATIO       = 0.35  # cx dentro de ±35% del centro del frame
OVERTAKE_TRIGGER_Y_MIN    = 300   # y2 del bbox ≥ este valor (coche cerca)

# Tiempos de la maniobra (calibrar en pista)
OVERTAKE_LEFT_SEC    = 1.8  # tiempo girando al carril contrario
OVERTAKE_PASS_SEC    = 2.2  # tiempo pasando el obstáculo (recto)
OVERTAKE_RETURN_SEC  = 1.8  # tiempo regresando al carril propio
OVERTAKE_STEER_DEG   = 20.0 # grados desde centro para el giro de rebase

# ============================================================
# GAMEPAD  (mapeo Xbox / PS4 genérico vía pygame)
# ============================================================
#   A / Cruz      (btn 0) → Manual
#   B / Círculo   (btn 1) → Visión Test (cámara, motores OFF)
#   X / Cuadrado  (btn 2) → Autónomo (carril + STOP + crucero)
#   Y / Triángulo (btn 3) → Estacionamiento
#   Start/Options (btn 9) → Emergencia (freno + MANUAL)
BTN_MANUAL     = 0
BTN_VISION     = 1
BTN_AUTONOMOUS = 2
BTN_PARKING    = 3
BTN_EMERGENCY  = 9

# Mantener alias para compatibilidad
BTN_BACK_TO_MANUAL = BTN_MANUAL
BTN_VISION_TEST    = BTN_VISION

AXIS_STEER    = 0   # Joystick IZQUIERDO X  (−1=izq, +1=der)
AXIS_THROTTLE = 5   # Gatillo R2            (−1=suelto, +1=fondo)
AXIS_BRAKE    = 2   # Gatillo L2  (verificado con test_gamepad.py)

JOYSTICK_DEADBAND = 0.08
TRIGGER_DEADBAND  = 0.05

# ============================================================
# CRUCERO PEATONAL
# ============================================================
CROSSWALK_STOP_SEC    = 3.0   # segundos detenido en el crucero
CROSSWALK_WHITE_RATIO = 0.55  # fracción mínima de píxeles blancos en fila

# ============================================================
# RUTAS
# ============================================================
import os
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_DIR  = os.path.join(BASE_DIR, "logs")
