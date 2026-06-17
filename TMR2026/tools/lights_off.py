#!/usr/bin/env python3
"""Turn OFF ALL of the car's signaling LEDs.

Forces the BCM pins defined as LEDs in config.py and vision_config.yaml to 0.
Useful when a previous process (vision_module.py, a crashed run, the systemd
service) left an LED on -- lgpio does NOT reset the pin level on exit, it only
releases the reservation.

Usage:
    sudo systemctl stop carrito_tmr     # if the service is running
    python TMR2026/tools/lights_off.py
"""
import sys
import time

LED_PINS = [5, 6, 16, 17, 19, 20, 25, 26]


def main() -> int:
    try:
        import lgpio
    except ImportError:
        print("ERROR: lgpio not available.")
        return 1

    try:
        handle = lgpio.gpiochip_open(4)
    except Exception as e:
        print(f"ERROR opening gpiochip 4: {e}")
        return 1

    n_off = 0
    blocked: list[int] = []
    for pin in LED_PINS:
        try:
            lgpio.gpio_claim_output(handle, pin, 0)
            lgpio.gpio_write(handle, pin, 0)
            n_off += 1
            print(f"  pin BCM {pin:>2}: OFF")
        except Exception as e:
            blocked.append(pin)
            print(f"  pin BCM {pin:>2}: could not claim ({e})")

    time.sleep(0.05)

    for pin in LED_PINS:
        try:
            lgpio.gpio_free(handle, pin)
        except Exception:
            pass
    lgpio.gpiochip_close(handle)

    print(f"\n[OK] {n_off}/{len(LED_PINS)} pins forced to 0.")
    if blocked:
        print(f"Blocked pins: {blocked}")
        print("Another process has them reserved. Run:")
        print("    sudo systemctl stop carrito_tmr")
        print("    pkill -f main.py ; pkill -f vision_module")
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
