"""Identify the controller's axes and buttons.
Run: python3 test_gamepad.py
Move sticks, press triggers and buttons to see their numbers.
Ctrl+C to quit.
"""
import time
import os
os.environ["SDL_VIDEODRIVER"] = "dummy"
os.environ["SDL_AUDIODRIVER"] = "dummy"

import pygame
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("No controller detected. Connect it via Bluetooth and run again.")
    exit(1)

js = pygame.joystick.Joystick(0)
js.init()
print(f"Controller: {js.get_name()}")
print(f"Axes: {js.get_numaxes()}  |  Buttons: {js.get_numbuttons()}")
print("-" * 50)
print("Move sticks/triggers or press buttons...\n")

last_axes = [0.0] * js.get_numaxes()
last_btns = [0]  * js.get_numbuttons()

try:
    while True:
        pygame.event.pump()

        axes = [js.get_axis(i) for i in range(js.get_numaxes())]
        btns = [js.get_button(i) for i in range(js.get_numbuttons())]

        for i, (a, la) in enumerate(zip(axes, last_axes)):
            if abs(a - la) > 0.08:
                print(f"  AXIS {i:2d} = {a:+.2f}")

        for i, (b, lb) in enumerate(zip(btns, last_btns)):
            if b and not lb:
                print(f"  BTN  {i:2d} PRESSED")
            elif not b and lb:
                print(f"  BTN  {i:2d} released")

        last_axes = axes
        last_btns = btns
        time.sleep(0.05)

except KeyboardInterrupt:
    print("\nDone.")
    pygame.quit()
