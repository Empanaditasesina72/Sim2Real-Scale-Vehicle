# -*- coding: utf-8 -*-
"""test_servo.py — Prueba directa del servo en PCA9685."""

from adafruit_extended_bus import ExtendedI2C
import adafruit_pca9685
from adafruit_motor import servo as adafruit_servo
import time

i2c = ExtendedI2C(3)
pca = adafruit_pca9685.PCA9685(i2c)
pca.frequency = 50

print("=== Test servo canal 15, pulsos 500-2500 us ===")
s = adafruit_servo.Servo(pca.channels[15], min_pulse=500, max_pulse=2500)
for angle in [45, 90, 135, 90]:
    print(f"  angulo: {angle}")
    s.angle = angle
    time.sleep(1.5)

print("\n=== Test servo canal 15, pulsos 1000-2000 us ===")
s2 = adafruit_servo.Servo(pca.channels[15], min_pulse=1000, max_pulse=2000)
for angle in [0, 90, 180, 90]:
    print(f"  angulo: {angle}")
    s2.angle = angle
    time.sleep(1.5)

print("\nListo.")
