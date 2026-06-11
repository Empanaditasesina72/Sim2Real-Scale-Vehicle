# -*- coding: utf-8 -*-
"""
export_model.py — Exporta weights/tmr_signs.pt a NCNN para la Raspberry Pi 5.

¿Por qué NCNN?
  La Pi 5 corre YOLO en CPU (ARM Cortex-A76). PyTorch puro logra ~6-8 FPS
  con yolov8n@320; NCNN (el formato recomendado por Ultralytics para
  Raspberry Pi) logra 3-4× más con la MISMA precisión. SignDetector carga
  automáticamente `weights/tmr_signs_ncnn_model/` si existe y cae al `.pt`
  si no.

Uso (desde TMR2026/, en la PC o en la Pi — el resultado es portable):

    python tools/export_model.py                 # exporta a NCNN imgsz=320
    python tools/export_model.py --imgsz 416     # más alcance, algo más lento

El resultado queda en weights/tmr_signs_ncnn_model/ (param + bin + metadata
con los nombres de las 7 clases). Se versiona en git para que la Pi no
tenga que exportar nada.
"""

import argparse
import sys
from pathlib import Path

HERE    = Path(__file__).resolve().parent.parent   # TMR2026/
WEIGHTS = HERE / "weights" / "tmr_signs.pt"

# imgsz por defecto = 320: el MISMO con el que se validó el sistema
# (SignDetector usa imgsz=320 en el Pi y en el simulador).
DEFAULT_IMGSZ = 320


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--imgsz", type=int, default=DEFAULT_IMGSZ,
                    help=f"tamaño de inferencia (default {DEFAULT_IMGSZ})")
    ap.add_argument("--weights", type=Path, default=WEIGHTS,
                    help="ruta al .pt a exportar")
    args = ap.parse_args()

    if not args.weights.exists():
        print(f"[EXPORT] No existe {args.weights}")
        return 1

    from ultralytics import YOLO

    print(f"[EXPORT] Cargando {args.weights} ...")
    model = YOLO(str(args.weights))
    print(f"[EXPORT] Clases: {model.names}")

    # half=True: FP16 — la A76 de la Pi 5 lo acelera y el tamaño baja a la
    # mitad. La pérdida de precisión es despreciable para detección.
    print(f"[EXPORT] Exportando a NCNN (imgsz={args.imgsz}, FP16) ...")
    out = model.export(format="ncnn", imgsz=args.imgsz, half=True)

    print(f"[EXPORT] Listo: {out}")
    print("[EXPORT] SignDetector lo usará automáticamente en el próximo arranque.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
