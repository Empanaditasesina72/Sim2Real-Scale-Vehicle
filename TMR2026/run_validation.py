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
    # Una sola corrida hace TODO en secuencia: maneja → STOP → espera →
    # reanuda → sigue → estaciona en batería. Duración amplia para que dé
    # tiempo a completar el estacionamiento.
    dur = 50
    for a in sys.argv[1:]:
        try: dur = int(a)
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
    print("[OK] Unity detectado en el puerto 5005.")
    print("[!] IMPORTANTE: cierra cualquier OTRA terminal de Python conectada")
    print("    (ej. main_simulator.py --display). Unity solo atiende 1 cliente.\n")

    here = os.path.dirname(os.path.abspath(__file__))

    # 1) Correr el control con logging de validación (secuencia completa)
    cmd = [sys.executable, os.path.join(here, "main_simulator.py"),
           "--validate", "--duration", str(dur), "--parking"]
    print(f">>> Ejecutando la SECUENCIA COMPLETA durante {dur} s...")
    print("    maneja → STOP (frena+espera 5s+reanuda) → sigue → ESTACIONA")
    print("    (cubre las 3 pruebas del PDF en una sola corrida)\n")
    subprocess.run(cmd, cwd=here)

    # 2) Generar gráficas del artículo
    print("\n>>> Generando gráficas del artículo...")
    subprocess.run([sys.executable, os.path.join(here, "analyze_results.py"),
                    "validation_results"], cwd=here)

    print("\n" + "=" * 60)
    print("  VALIDACIÓN COMPLETA")
    print("=" * 60)
    print("  Resultados en la carpeta:  validation_results/")
    print("    • P1_latencia.csv  P2_pid_stop.csv  P3_fsm.csv   (datos)")
    print("    • fig1_latency.png  fig2_braking.png  fig3_fsm.png (figuras EN)")
    print("    • SCOREBOARD.txt     (tablero de puntos, inglés)")
    print("\n  Arma la entrega en inglés:  python armar_entrega.py")
    print("  (mapa de requisitos: docs/DELIVERY_PROFESSOR.md)")


if __name__ == "__main__":
    main()
