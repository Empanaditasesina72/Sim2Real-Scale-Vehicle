# -*- coding: utf-8 -*-
"""
run_validation.py — Ejecuta la VALIDACIÓN COMPLETA del PDF en un solo comando.

Hace las 3 pruebas del PDF (latencia, frenado PID ante STOP, transiciones FSM),
genera los CSV, calcula el tablero de puntos y produce las gráficas del artículo.

REQUISITO: Unity debe estar en PLAY (servidor escuchando en 127.0.0.1:5005).

Uso:
    python run_validation.py            # 60 s de validación
    python run_validation.py 90         # 90 s
"""

import sys
import socket
import subprocess
import os

PORT = 5005
DURATION = 60


def unity_listening(host="127.0.0.1", port=PORT, timeout=1.5) -> bool:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(timeout)
    try:
        s.connect((host, port))
        s.close()
        return True
    except Exception:
        return False


def main():
    dur = DURATION
    if len(sys.argv) > 1:
        try: dur = int(sys.argv[1])
        except ValueError: pass

    print("=" * 60)
    print("  VALIDACIÓN SIM2REAL — TMR 2026 (3 pruebas del PDF)")
    print("=" * 60)

    if not unity_listening():
        print("\n[X] Unity NO está escuchando en 127.0.0.1:5005.")
        print("    1) Abre Unity y dale PLAY.")
        print("    2) Verifica el texto 'Listening on port 5005' en la consola.")
        print("    3) Vuelve a correr:  python run_validation.py")
        sys.exit(1)
    print("[OK] Unity detectado en el puerto 5005.\n")

    here = os.path.dirname(os.path.abspath(__file__))

    # 1) Correr el control con logging de validación
    print(f">>> Ejecutando las 3 pruebas durante {dur} s...")
    print("    (el carro maneja, detecta el STOP, frena y reanuda)\n")
    subprocess.run([sys.executable, os.path.join(here, "main_simulator.py"),
                    "--validate", "--duration", str(dur)],
                   cwd=here)

    # 2) Generar gráficas del artículo
    print("\n>>> Generando gráficas del artículo...")
    subprocess.run([sys.executable, os.path.join(here, "analyze_results.py")],
                   cwd=here)

    print("\n" + "=" * 60)
    print("  VALIDACIÓN COMPLETA")
    print("=" * 60)
    print("  Resultados en la carpeta:  validation_results/")
    print("    • P1_latencia.csv  P2_pid_stop.csv  P3_fsm.csv   (datos)")
    print("    • fig1_latencia.png  fig2_frenado.png  fig3_fsm.png (gráficas)")
    print("    • PUNTAJE.txt        (tablero de puntos)")
    print("\n  Entrega al profesor: ver ENTREGA_PROFESOR.md")


if __name__ == "__main__":
    main()
