import time
import board
import busio
import digitalio
import adafruit_vl53l0x
import RPi.GPIO as GPIO

# =========================
# CONFIG LED (motor)
# =========================
LED_MOTOR = 22
DISTANCIA_MIN =90  # mm

GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_MOTOR, GPIO.OUT)

# =========================
# XSHUT
# =========================
XSHUT1 = digitalio.DigitalInOut(board.D17)
XSHUT2 = digitalio.DigitalInOut(board.D27)

XSHUT1.direction = digitalio.Direction.OUTPUT
XSHUT2.direction = digitalio.Direction.OUTPUT

# Apagar ambos
XSHUT1.value = False
XSHUT2.value = False
time.sleep(0.5)

# =========================
# I2C
# =========================
i2c = busio.I2C(board.SCL, board.SDA)

# Encender sensor delantero
XSHUT1.value = True
time.sleep(0.5)
sensor_delante = adafruit_vl53l0x.VL53L0X(i2c)
sensor_delante.set_address(0x30)

# Encender sensor trasero
XSHUT2.value = True
time.sleep(0.5)
sensor_atras = adafruit_vl53l0x.VL53L0X(i2c)

print("Sistema iniciado...")

# =========================
# LOOP
# =========================
try:
    while True:
        d1 = sensor_delante.range
        d2 = sensor_atras.range

        print(f"Delante: {d1} mm | Atrás: {d2} mm")

        # 🚗 lógica motor
        if d1 < DISTANCIA_MIN or d2 < DISTANCIA_MIN:
            GPIO.output(LED_MOTOR, GPIO.LOW)   # motor OFF
        else:
            GPIO.output(LED_MOTOR, GPIO.HIGH)  # motor ON

        time.sleep(0.2)

except KeyboardInterrupt:
    print("\nPrograma detenido")

finally:
    GPIO.cleanup()
