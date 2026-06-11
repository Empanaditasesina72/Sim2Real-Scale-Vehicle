# -*- coding: utf-8 -*-
"""
export_imx500.py — Convierte tmr_signs.pt al formato del NPU IMX500 (.rpk).

El Sony IMX500 de la Pi AI Camera ejecuta el modelo DENTRO del sensor:
la Pi recibe los tensores ya inferidos en la metadata de cada frame y la
CPU queda libre. Para eso el modelo debe cuantizarse (INT8) y empaquetarse
como .rpk con el toolchain de Sony — eso hace este script, vía el export
`imx` de Ultralytics.

⚠ SOLO corre en LINUX (la propia Pi sirve; en Windows no existe el
  imx500-converter). La cuantización usa el dataset traffic_lights/ como
  calibración y puede tardar 15-60 min en la Pi 5 — se hace UNA sola vez.

Prerequisitos (una vez, en la Pi):
    sudo apt install -y imx500-all imx500-tools default-jre
    pip3 install --break-system-packages model-compression-toolkit "imx500-converter[pt]"
    # (ultralytics intenta auto-instalar lo que falte)

Uso (desde TMR2026/):
    python tools/export_imx500.py                  # export completo
    python tools/export_imx500.py --fraction 0.1   # calibración más rápida

Al terminar deja:
    weights/tmr_signs_imx500.rpk           ← lo que carga main.py (config.py)
    weights/tmr_signs_imx500_labels.txt    ← orden de clases del modelo

main.py lo detecta solo en el siguiente arranque:
    [VISION] Backend: NPU IMX500 (inferencia on-chip)
"""

import argparse
import shutil
import sys
from pathlib import Path

HERE     = Path(__file__).resolve().parent.parent          # TMR2026/
WEIGHTS  = HERE / "weights" / "tmr_signs.pt"
DATA     = HERE.parent / "traffic_lights" / "data.yaml"     # calibración INT8
DST_RPK  = HERE / "weights" / "tmr_signs_imx500.rpk"
DST_LBL  = HERE / "weights" / "tmr_signs_imx500_labels.txt"

# Orden de clases del dataset (respaldo si el export no deja labels.txt)
FALLBACK_LABELS = ("green", "left", "red", "right", "stop", "straight", "yellow")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--fraction", type=float, default=0.25,
                    help="fracción del dataset para calibrar INT8 "
                         "(default 0.25 — sube a 1.0 para máxima precisión)")
    args = ap.parse_args()

    if not sys.platform.startswith("linux"):
        print("[IMX500] El imx500-converter de Sony solo existe en Linux.")
        print("[IMX500] Corre este script EN LA RASPBERRY PI (o un Linux):")
        print("[IMX500]     cd ~/Carrito/TMR2026 && python tools/export_imx500.py")
        return 1

    if not WEIGHTS.exists():
        print(f"[IMX500] No existe {WEIGHTS}")
        return 1
    if not DATA.exists():
        print(f"[IMX500] No existe el dataset de calibración: {DATA}")
        return 1

    from ultralytics import YOLO

    print(f"[IMX500] Cargando {WEIGHTS} ...")
    model = YOLO(str(WEIGHTS))
    print(f"[IMX500] Clases: {model.names}")
    print(f"[IMX500] Exportando a formato imx (INT8, calibración "
          f"{args.fraction:.0%} de {DATA.name}) ...")
    print("[IMX500] Esto tarda 15-60 min en la Pi 5. Una sola vez. Paciencia.")

    out = model.export(format="imx", data=str(DATA), fraction=args.fraction)
    out_dir = Path(out)
    print(f"[IMX500] Export crudo en: {out_dir}")

    # Localizar el .rpk y labels dentro del directorio exportado y copiarlos
    # a los nombres canónicos que espera config.py.
    rpks = sorted(out_dir.rglob("*.rpk"))
    if not rpks:
        print("[IMX500] ERROR: el export no produjo ningún .rpk.")
        print("[IMX500] Revisa que imx500-tools y java estén instalados.")
        return 1
    shutil.copy2(rpks[0], DST_RPK)
    print(f"[IMX500] .rpk listo: {DST_RPK}")

    labels = sorted(out_dir.rglob("labels.txt"))
    if labels:
        shutil.copy2(labels[0], DST_LBL)
    else:
        DST_LBL.write_text("\n".join(FALLBACK_LABELS) + "\n", encoding="utf-8")
    print(f"[IMX500] labels:     {DST_LBL}")

    print("=" * 60)
    print("[IMX500] LISTO. En el siguiente arranque main.py mostrará:")
    print("[IMX500]   [VISION] Backend: NPU IMX500 (inferencia on-chip)")
    print("[IMX500] Para volver al camino CPU: config.py -> USE_IMX500_NPU=False")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(main())
