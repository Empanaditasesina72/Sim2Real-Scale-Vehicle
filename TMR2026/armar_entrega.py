# -*- coding: utf-8 -*-
"""
armar_entrega.py — Empaqueta TODO lo que se entrega al profesor en un ZIP.

Toma los resultados de las dos corridas:
  - validation_results/          (Prueba 1 latencia + Prueba 2 STOP + Prueba 3 FSM)
  - validation_results_parking/  (Prueba 3 estacionamiento en batería)
y los documentos, y crea  ENTREGA_TMR2026.zip  en el Escritorio/Documentos.

Uso (tras correr 'run_validation.py' y 'run_validation.py parking'):
    python armar_entrega.py
"""

import os
import sys
import shutil

try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass

HERE = os.path.dirname(os.path.abspath(__file__))
DEST = os.path.join(os.path.expanduser("~"), "Documents", "ENTREGA_TMR2026")
ZIP  = os.path.join(os.path.expanduser("~"), "Documents", "ENTREGA_TMR2026")


def copy_into(folder, subdir):
    """Copia los archivos de 'folder' (si existe) dentro de DEST/subdir."""
    src = os.path.join(HERE, folder)
    if not os.path.isdir(src):
        print(f"   [!] no existe {folder}/ (¿corriste esa prueba?)")
        return 0
    dst = os.path.join(DEST, subdir)
    os.makedirs(dst, exist_ok=True)
    n = 0
    for f in os.listdir(src):
        if f.lower().endswith((".csv", ".png", ".txt")):
            shutil.copy2(os.path.join(src, f), os.path.join(dst, f))
            n += 1
    print(f"   ✓ {subdir}/  ({n} archivos)")
    return n


def main():
    print("=" * 60)
    print("  ARMANDO PAQUETE DE ENTREGA — TMR 2026")
    print("=" * 60)

    if os.path.isdir(DEST):
        shutil.rmtree(DEST)
    os.makedirs(DEST, exist_ok=True)

    print(">>> Copiando resultados...")
    # Una sola corrida contiene las 3 pruebas (latencia + STOP + FSM con
    # ciclo del STOP y estacionamiento en batería).
    copy_into("validation_results", "01_resultados_3_pruebas")

    print(">>> Copiando documentos...")
    docs = os.path.join(DEST, "03_documentos")
    os.makedirs(docs, exist_ok=True)
    for d in ("ENTREGA_PROFESOR.md", "DELIVERY_PROFESSOR.md", "CALIBRACION_SIM.md"):
        p = os.path.join(HERE, d)
        if os.path.exists(p):
            shutil.copy2(p, os.path.join(docs, d))
            print(f"   ✓ {d}")

    # LEEME
    with open(os.path.join(DEST, "LEEME.txt"), "w", encoding="utf-8") as f:
        f.write(
            "ENTREGA — Validacion Sim2Real del Vehiculo Autonomo (TMR 2026)\n"
            "================================================================\n\n"
            "Todo se obtuvo en UNA sola corrida (python run_validation.py):\n"
            "maneja -> detecta STOP -> frena -> espera 5s -> reanuda -> avanza\n"
            "-> estaciona en bateria.\n\n"
            "01_resultados_3_pruebas/  -> Las 3 pruebas del PDF:\n"
            "   - P1 latencia del ciclo de control\n"
            "   - P2 frenado PID ante STOP (se detiene a ~290 mm)\n"
            "   - P3 transiciones FSM: ciclo del STOP (CRUCERO, PRECAUCION,\n"
            "     FRENADO, ESPERA, REANUDAR) + estacionamiento (PARKING_SEARCH,\n"
            "     PARKING_MANEUVER, PARKED).\n"
            "   Contiene los 3 CSV + las 3 graficas (PNG) + PUNTAJE.txt (100/100).\n\n"
            "03_documentos/            -> ENTREGA_PROFESOR.md (mapa de cada\n"
            "   requisito del PDF) y CALIBRACION_SIM.md.\n\n"
            "Codigo fuente: repos de GitHub 'Carrito' (Python) y\n"
            "'TMR2026_Sim' (Unity).\n"
        )
    print("   ✓ LEEME.txt")

    # README en inglés (entrega internacional / artículo)
    with open(os.path.join(DEST, "README.txt"), "w", encoding="utf-8") as f:
        f.write(
            "DELIVERY — Sim2Real Validation of the Autonomous Vehicle (TMR 2026)\n"
            "==================================================================\n\n"
            "Everything was obtained in a SINGLE run (python run_validation.py):\n"
            "drive -> detect STOP -> brake -> wait 5s -> resume -> drive forward\n"
            "-> park in battery.\n\n"
            "01_resultados_3_pruebas/  -> The 3 PDF tests:\n"
            "   - P1 control-loop latency\n"
            "   - P2 PID braking at STOP (stops at ~290 mm)\n"
            "   - P3 FSM transitions: STOP cycle (CRUCERO, PRECAUCION, FRENADO,\n"
            "     ESPERA, REANUDAR) + parking (PARKING_SEARCH, PARKING_MANEUVER,\n"
            "     PARKED).\n"
            "   Contains the 3 CSV + 6 figures (ES + _en PNG) + SCOREBOARD.txt\n"
            "   (English) and PUNTAJE.txt (Spanish), both 100/100.\n\n"
            "03_documentos/            -> DELIVERY_PROFESSOR.md (English map of\n"
            "   every PDF requirement), ENTREGA_PROFESOR.md (Spanish) and\n"
            "   CALIBRACION_SIM.md.\n\n"
            "Source code: GitHub repos 'Carrito' (Python) and 'TMR2026_Sim' (Unity).\n"
        )
    print("   ✓ README.txt (English)")

    print(">>> Comprimiendo ZIP...")
    if os.path.exists(ZIP + ".zip"):
        os.remove(ZIP + ".zip")
    shutil.make_archive(ZIP, "zip", DEST)

    print("=" * 60)
    print(f"  LISTO. Entrega: {ZIP}.zip")
    print(f"        Carpeta:  {DEST}")
    print("=" * 60)


if __name__ == "__main__":
    main()
