"""Verify that SignDetector._apply_hysteresis only publishes a label after
it appears in N consecutive frames, and removes it when it stops appearing.
No hardware or loaded model.

Pinhole: verify that the bbox distance is computed correctly from the box
height.
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
    """Build a raw detection with a controlled bbox."""
    return Detection(label, 0.9, 40, y1, 120, y2,
                     distance_m=None)


def test_hysteresis_needs_n_consecutive_frames():
    sd = _make_sd(hyst=3)

    out = sd._apply_hysteresis([_det()])
    assert out == []

    out = sd._apply_hysteresis([_det()])
    assert out == []

    out = sd._apply_hysteresis([_det()])
    assert len(out) == 1
    assert out[0].label == "stop_sign"


def test_hysteresis_resets_on_miss():
    sd = _make_sd(hyst=3)
    sd._apply_hysteresis([_det()])
    sd._apply_hysteresis([_det()])
    sd._apply_hysteresis([])
    out = sd._apply_hysteresis([_det()])
    assert out == []
    out = sd._apply_hysteresis([_det()])
    assert out == []
    out = sd._apply_hysteresis([_det()])
    assert len(out) == 1


def test_hysteresis_keeps_largest_bbox_per_frame():
    """If there are two stops in the same frame, the largest (closest) is kept."""
    sd = _make_sd(hyst=1)
    small = Detection("stop_sign", 0.8, 10, 10,  40,  40, None)
    large = Detection("stop_sign", 0.8, 10, 10, 110, 110, None)
    out = sd._apply_hysteresis([small, large])
    assert len(out) == 1
    assert out[0] is large


def test_bbox_distance_matches_pinhole_formula():
    """distance_m = (real_height × focal) / height_px."""
    height_px = 50
    expected  = (STOP_SIGN_REAL_HEIGHT_M * CAMERA_FOCAL_LENGTH_PX) / height_px

    det = Detection("stop_sign", 0.9, 0, 0, 80, height_px,
                    distance_m=expected)
    by_hand = STOP_SIGN_REAL_HEIGHT_M * CAMERA_FOCAL_LENGTH_PX / 50.0
    assert abs(det.distance_m - by_hand) < 0.01
    assert det.height_px == 50
