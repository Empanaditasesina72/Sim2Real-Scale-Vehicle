# -*- coding: utf-8 -*-
"""
armar_entrega.py — Builds the ENGLISH delivery ZIP for the professor.

Takes a single run (validation_results/) + the English docs and creates
DELIVERY_TMR2026.zip in ~/Documents. English ONLY (the professor already has
the Spanish version). It copies an explicit allow-list, so nothing in Spanish
leaks into the package.

Usage (after running run_validation.py):
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
DEST = os.path.join(os.path.expanduser("~"), "Documents", "DELIVERY_TMR2026")
ZIP  = DEST

# Results from validation_results/  (source name -> name inside the ZIP).
RESULT_FILES = {
    "P1_latencia.csv":  "P1_latency.csv",
    "P2_pid_stop.csv":  "P2_pid_stop.csv",
    "P3_fsm.csv":       "P3_fsm.csv",
    "fig1_latency.png": "fig1_latency.png",
    "fig2_braking.png": "fig2_braking.png",
    "fig3_fsm.png":     "fig3_fsm.png",
    "SCOREBOARD.txt":   "SCOREBOARD.txt",
}

# English documents (source name -> name inside the ZIP).
DOC_FILES = {
    "DELIVERY_PROFESSOR.md": "DELIVERY_PROFESSOR.md",
    "CALIBRATION_SIM.md":    "CALIBRATION_SIM.md",
}


def _copy(src_dir, mapping, dst_dir, label):
    os.makedirs(dst_dir, exist_ok=True)
    n = 0
    for src_name, dst_name in mapping.items():
        src = os.path.join(src_dir, src_name)
        if os.path.exists(src):
            shutil.copy2(src, os.path.join(dst_dir, dst_name))
            n += 1
        else:
            print(f"   [!] missing: {src_name}  (did you run run_validation.py?)")
    print(f"   OK  {label}  ({n} files)")
    return n


def main():
    print("=" * 60)
    print("  BUILDING DELIVERY PACKAGE (English) — TMR 2026")
    print("=" * 60)

    if os.path.isdir(DEST):
        shutil.rmtree(DEST)
    os.makedirs(DEST, exist_ok=True)

    print(">>> Copying results...")
    _copy(os.path.join(HERE, "validation_results"), RESULT_FILES,
          os.path.join(DEST, "01_results_3_tests"), "01_results_3_tests/")

    print(">>> Copying documents...")
    _copy(os.path.join(HERE, "docs"), DOC_FILES,
          os.path.join(DEST, "02_documents"), "02_documents/")

    # README (English)
    with open(os.path.join(DEST, "README.txt"), "w", encoding="utf-8") as f:
        f.write(
            "DELIVERY - Sim2Real Validation of the Autonomous Vehicle (TMR 2026)\n"
            "==================================================================\n\n"
            "Everything was obtained in a SINGLE run (python run_validation.py):\n"
            "drive -> detect STOP -> brake -> wait 5s -> resume -> drive forward\n"
            "-> park in battery.\n\n"
            "01_results_3_tests/  -> The 3 PDF tests:\n"
            "   - P1 control-loop latency      (P1_latency.csv,  fig1_latency.png)\n"
            "   - P2 PID braking at STOP        (P2_pid_stop.csv, fig2_braking.png)\n"
            "   - P3 FSM transitions            (P3_fsm.csv,      fig3_fsm.png)\n"
            "     STOP cycle (CRUCERO, PRECAUCION, FRENADO, ESPERA, REANUDAR)\n"
            "     + parking (PARKING_SEARCH, PARKING_MANEUVER, PARKED).\n"
            "   Plus SCOREBOARD.txt  (100/100 - PASSED).\n\n"
            "02_documents/        -> DELIVERY_PROFESSOR.md (English map of every\n"
            "   PDF requirement) and CALIBRATION_SIM.md (simulator calibration).\n\n"
            "Note: the FSM state names (CRUCERO, PRECAUCION, FRENADO, ESPERA,\n"
            "REANUDAR) are code identifiers and are kept as-is.\n\n"
            "Source code: GitHub repos 'Carrito' (Python) and 'TMR2026_Sim' (Unity).\n"
        )
    print("   OK  README.txt")

    print(">>> Zipping...")
    if os.path.exists(ZIP + ".zip"):
        os.remove(ZIP + ".zip")
    shutil.make_archive(ZIP, "zip", DEST)

    print("=" * 60)
    print(f"  DONE. Delivery: {ZIP}.zip")
    print(f"        Folder:   {DEST}")
    print("=" * 60)


if __name__ == "__main__":
    main()
