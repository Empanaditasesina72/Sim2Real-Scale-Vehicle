"""Verify the PURE part of the NPU backend (vision/imx500_detector.py)
without needing picamera2 or the camera:

  1. map_raw_detections: raw tensors -> Detection with the same semantics
     as the CPU path (stop->stop_sign, conf/area/class filters, pinhole).
  2. LabelHysteresis: confirms after N frames, resets on a missed one,
     keeps the largest bbox of the frame.
"""

from vision.imx500_detector import (
    map_raw_detections, LabelHysteresis, DEFAULT_LABELS,
)
from vision.sign_detector import (
    Detection, SIGN_REAL_HEIGHT_M, CAMERA_FOCAL_LENGTH_PX,
)


def test_stop_is_normalized_and_gets_pinhole_distance():
    raw = [(100, 100, 180, 150, 0.90, 4)]
    dets = map_raw_detections(raw, DEFAULT_LABELS, conf_min=0.55)
    assert len(dets) == 1
    d = dets[0]
    assert d.label == "stop_sign"
    expected = (SIGN_REAL_HEIGHT_M["stop"] * CAMERA_FOCAL_LENGTH_PX) / 50.0
    assert abs(d.distance_m - expected) < 1e-6


def test_low_confidence_and_small_area_are_dropped():
    raw = [
        (0, 0, 100, 100, 0.30, 4),
        (0, 0, 10, 10, 0.90, 4),
        (0, 0, 100, 100, 0.90, 99),
    ]
    assert map_raw_detections(raw, DEFAULT_LABELS, conf_min=0.55) == []


def test_all_seven_classes_map_with_their_own_height():
    dets = []
    for cls_id, name in enumerate(DEFAULT_LABELS):
        raw = [(0, 0, 60, 60, 0.9, cls_id)]
        out = map_raw_detections(raw, DEFAULT_LABELS, conf_min=0.55)
        assert len(out) == 1, f"class {name} did not map"
        dets.append(out[0])
        expected = (SIGN_REAL_HEIGHT_M[name] * CAMERA_FOCAL_LENGTH_PX) / 60.0
        assert abs(out[0].distance_m - expected) < 1e-6
    labels = {d.label for d in dets}
    assert "stop_sign" in labels and "stop" not in labels


def test_swapped_coordinates_are_sanitized():
    raw = [(180, 150, 100, 100, 0.9, 2)]
    out = map_raw_detections(raw, DEFAULT_LABELS, conf_min=0.55)
    assert len(out) == 1
    d = out[0]
    assert d.x1 < d.x2 and d.y1 < d.y2


def _det(label="stop_sign", size=100):
    return Detection(label, 0.9, 0, 0, size, size, distance_m=None)


def test_hysteresis_confirms_after_n_frames():
    h = LabelHysteresis(n_frames=3)
    assert h.update([_det()]) == []
    assert h.update([_det()]) == []
    out = h.update([_det()])
    assert len(out) == 1 and out[0].label == "stop_sign"


def test_hysteresis_resets_on_missing_frame():
    h = LabelHysteresis(n_frames=3)
    h.update([_det()])
    h.update([_det()])
    h.update([])
    assert h.update([_det()]) == []
    assert h.update([_det()]) == []
    assert len(h.update([_det()])) == 1


def test_hysteresis_keeps_largest_bbox():
    h = LabelHysteresis(n_frames=1)
    small, large = _det(size=30), _det(size=120)
    out = h.update([small, large])
    assert len(out) == 1
    assert out[0] is large
