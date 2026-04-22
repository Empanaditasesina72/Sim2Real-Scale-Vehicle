# -*- coding: utf-8 -*-
"""
main.py (root) — Loader del vehículo TMR 2026.

Este archivo solo existe para que el usuario pueda correr desde la raíz:

    python main.py [--display]

Delega toda la lógica a TMR2026/main.py preservando CWD e imports relativos
(vision/, hardware/, control/, autonomy/).

El servicio systemd (TMR2026/systemd/carrito_tmr.service) sigue apuntando
directamente a TMR2026/main.py — este loader es SOLO para ejecución manual.
"""
import os
import sys
import runpy

HERE = os.path.dirname(os.path.abspath(__file__))
TMR  = os.path.join(HERE, "TMR2026")

if not os.path.isdir(TMR):
    sys.exit(f"[ERROR] No se encontró la carpeta TMR2026 en {HERE}")

# Asegurar que los imports 'from hardware.x import ...' resuelvan
os.chdir(TMR)
sys.path.insert(0, TMR)

# Ejecutar main.py como __main__ para que su bloque if __name__ == "__main__" corra
runpy.run_path(os.path.join(TMR, "main.py"), run_name="__main__")
