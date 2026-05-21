#!/usr/bin/env python3
"""
lights_off.py — Apaga TODOS los LEDs de señalización del coche.

Fuerza a 0 los pines BCM definidos como LEDs en config.py y vision_config.yaml.
Útil cuando un proceso previo (vision_module.py, una corrida que crasheó, el
servicio systemd) dejó un LED encendido — lgpio NO resetea el nivel del pin
al salir, sólo libera la reserva.

Uso:
    sudo systemctl stop carrito_tmr     # si el servicio está corriendo
    python TMR2026/tools/lights_off.py
"""
import sys
import time

# Pines LED en uso en TODO el repo (cableado actual + históricos):
#   17      → direccional IZQ (Pin 11) — actual
#    5      → direccional DER (Pin 29) — actual
#    6      → freno           (Pin 31) — actual
#   19, 20  → direccionales antiguas (signals.py viejo / vision_config.yaml)
#   16      → freno antiguo (brake_light.py viejo)
#   25, 26  → PIN_LED_STOP / PIN_LED_STATUS (legacy, definidos en config.py)
# Cubrimos histórico + actual para que el script siga funcionando como
# botón de pánico aunque haya basura residual de configuraciones previas.
LED_PINS = [5, 6, 16, 17, 19, 20, 25, 26]


def main() -> int:
    try:
        import lgpio
    except ImportError:
        print("ERROR: lgpio no disponible.")
        return 1

    try:
        handle = lgpio.gpiochip_open(4)   # Pi 5 chip 4
    except Exception as e:
        print(f"ERROR abriendo gpiochip 4: {e}")
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
            print(f"  pin BCM {pin:>2}: NO se pudo reclamar ({e})")

    time.sleep(0.05)

    for pin in LED_PINS:
        try:
            lgpio.gpio_free(handle, pin)
        except Exception:
            pass
    lgpio.gpiochip_close(handle)

    print(f"\n[OK] {n_off}/{len(LED_PINS)} pines forzados a 0.")
    if blocked:
        print(f"Pines bloqueados: {blocked}")
        print("Otro proceso los tiene reservados. Ejecuta:")
        print("    sudo systemctl stop carrito_tmr")
        print("    pkill -f main.py ; pkill -f vision_module")
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
