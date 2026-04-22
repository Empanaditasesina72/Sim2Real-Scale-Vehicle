# -*- coding: utf-8 -*-
"""
conftest.py — configuración común de los tests de TMR 2026.

Añade `TMR2026/` a sys.path para que los tests puedan importar
`hardware.*`, `control.*`, `vision.*` directamente (igual que main.py).

Correr desde la raíz del repo:

    pytest TMR2026/tests -v

O desde TMR2026:

    cd TMR2026 && pytest tests -v
"""

import os
import sys

TMR_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if TMR_DIR not in sys.path:
    sys.path.insert(0, TMR_DIR)
