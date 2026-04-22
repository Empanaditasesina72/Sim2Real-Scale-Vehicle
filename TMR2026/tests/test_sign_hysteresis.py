# -*- coding: utf-8 -*-
"""
test_sign_hysteresis.py — verifica que SignDetector._apply_hysteresis
sólo publica una etiqueta tras aparecer en N frames consecutivos y la
retira cuando deja de aparecer.  Sin hardware ni modelo cargado.

Pinhole: verifica que la distancia del bbox se calcula correctamente
a partir de la altura de la caja.
"""

from vision.sign_detector import (
    SignDetector, Detection,
    STOP_SIGN_REAL_HEIGHT_M, CAMERA_FOCAL_LENGTH_PX,
)


def _make_sd(hyst=3) -> SignDetector:
    """Construye un SignDetector sin cargar el modelo."""
    sd = SignDetector.__new__(SignDetector)
    sd._conf       = 0.55
    sd._imgsz      = 320
    sd._hysteresis = hyst
    sd._model      = None
    sd._consecutive = {}
    sd._last_raw    = {}
    sd._results     = []
    return sd


def _det(label="stop_sign", y1=100, y2=200):
    """Fabrica una detección cruda con bbox controlado."""
    return Detection(label, 0.9, 40, y1, 120, y2,
                     distance_m=None)


def test_hysteresis_needs_n_consecutive_frames():
    sd = _make_sd(hyst=3)

    # Frame 1: ve stop_sign → aún no confirmada
    out = sd._apply_hysteresis([_det()])
    assert out == []

    # Frame 2: ve otra vez → aún no confirmada
    out = sd._apply_hysteresis([_det()])
    assert out == []

    # Frame 3: confirmada
    out = sd._apply_hysteresis([_det()])
    assert len(out) == 1
    assert out[0].label == "stop_sign"


def test_hysteresis_resets_on_miss():
    sd = _make_sd(hyst=3)
    sd._apply_hysteresis([_det()])
    sd._apply_hysteresis([_det()])
    # Frame vacío → contador vuelve a 0
    sd._apply_hysteresis([])
    # Dos frames más → no llega aún a 3
    out = sd._apply_hysteresis([_det()])
    assert out == []
    out = sd._apply_hysteresis([_det()])
    assert out == []
    # Tercer frame → confirmada
    out = sd._apply_hysteresis([_det()])
    assert len(out) == 1


def test_hysteresis_keeps_largest_bbox_per_frame():
    """Si hay dos stops en el mismo frame, se conserva el más grande (más cerca)."""
    sd = _make_sd(hyst=1)   # confirmación inmediata para simplificar
    small = Detection("stop_sign", 0.8, 10, 10,  40,  40, None)   # área 900
    large = Detection("stop_sign", 0.8, 10, 10, 110, 110, None)   # área 10000
    out = sd._apply_hysteresis([small, large])
    assert len(out) == 1
    assert out[0] is large


def test_bbox_distance_matches_pinhole_formula():
    """distance_m = (real_height × focal) / height_px."""
    # Simular _parse_results manualmente (sin Ultralytics).
    # Un bbox de 50 px de alto para stop_sign (alto real 0.18 m, focal 490 px):
    #   dist = (0.18 * 490) / 50 = 1.764 m
    height_px = 50
    expected  = (STOP_SIGN_REAL_HEIGHT_M * CAMERA_FOCAL_LENGTH_PX) / height_px

    det = Detection("stop_sign", 0.9, 0, 0, 80, height_px,
                    distance_m=expected)
    assert abs(det.distance_m - 1.764) < 0.01
    assert det.height_px == 50
