import pygame
import RPi.GPIO as GPIO
import time

# ===== PINES ZK-BM1 =====
IN1 = 22   # dirección
IN2 = 23   # dirección

IN3 = 17   # motor tracción
IN4 = 27   # motor tracción

PWM = 18   # velocidad (motor trasero)

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(PWM, GPIO.OUT)

pwm = GPIO.PWM(PWM, 1000)
pwm.start(0)

# ===== PS4 =====
pygame.init()
pygame.joystick.init()

joystick = pygame.joystick.Joystick(0)
joystick.init()

print("🎮 PS4 conectado")

# ===== DIRECCIÓN =====
def left():
    GPIO.output(IN1, 0)
    GPIO.output(IN2, 1)

def right():
    GPIO.output(IN1, 1)
    GPIO.output(IN2, 0)

def center():
    GPIO.output(IN1, 0)
    GPIO.output(IN2, 0)

# ===== TRACCIÓN =====
def forward(speed):
    GPIO.output(IN3, 1)
    GPIO.output(IN4, 0)
    pwm.ChangeDutyCycle(speed)

def backward(speed):
    GPIO.output(IN3, 0)
    GPIO.output(IN4, 1)
    pwm.ChangeDutyCycle(speed)

def stop_motor():
    GPIO.output(IN3, 0)
    GPIO.output(IN4, 0)
    pwm.ChangeDutyCycle(0)

# ===== LOOP =====
try:
    while True:
        pygame.event.pump()

        x = joystick.get_axis(0)   # izquierda/derecha
        y = joystick.get_axis(1)   # adelante/atrás
        r2 = joystick.get_axis(5)  # velocidad

        speed = int((r2 + 1) * 50)

        # zona muerta
        if abs(x) < 0.3:
            center()

        # 🧭 dirección
        if x < -0.3:
            left()
        elif x > 0.3:
            right()

        # 🚗 movimiento
        if y < -0.3:
            forward(speed)
        elif y > 0.3:
            backward(speed)
        else:
            stop_motor()

        time.sleep(0.05)

except KeyboardInterrupt:
    stop_motor()
    GPIO.cleanup()
    print("Apagado seguro")
