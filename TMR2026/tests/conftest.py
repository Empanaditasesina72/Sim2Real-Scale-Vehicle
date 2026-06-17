"""Common configuration for the TMR 2026 tests.

Adds `TMR2026/` to sys.path so the tests can import `hardware.*`,
`control.*`, `vision.*` directly (just like main.py).

Run from the repo root:

    pytest TMR2026/tests -v

Or from TMR2026:

    cd TMR2026 && pytest tests -v
"""

import os
import sys

TMR_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if TMR_DIR not in sys.path:
    sys.path.insert(0, TMR_DIR)
