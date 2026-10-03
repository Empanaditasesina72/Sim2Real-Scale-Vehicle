"""DriveNet data pipeline and model contract. No hardware, no dataset on disk
beyond what each test writes to tmp_path. Torch/onnxruntime tests skip when
those packages are absent (the Pi runs without torch)."""

import csv
import json

import cv2
import numpy as np
import pytest

from vision.drive_net import IMG_H, IMG_W, preprocess
from vision.drive_data import (AugConfig, augment, balance_weights, read_tubs,
                               smooth_series, split_samples)


def _frame_with_line(x, w=640, h=480):
    img = np.zeros((h, w, 3), np.uint8)
    cv2.line(img, (x, h // 2), (x, h - 1), (255, 255, 255), 3)
    return img


def _bottom_line_x(img):
    row = img[-2, :, 0].astype(float)
    return float((row * np.arange(row.size)).sum() / row.sum())


def _write_tub(path, errs, source="recorded", throttle=None):
    (path / "frames").mkdir(parents=True)
    with open(path / "labels.csv", "w", newline="") as f:
        cols = ["frame", "error_px", "confidence", "source"]
        if throttle is not None:
            cols.append("throttle")
        wr = csv.writer(f)
        wr.writerow(cols)
        for i, e in enumerate(errs):
            name = f"frames/{i:06d}.jpg"
            cv2.imwrite(str(path / name), _frame_with_line(320 + int(e)))
            row = [name, e, 1.0, source]
            if throttle is not None:
                row.append(throttle[i])
            wr.writerow(row)


def test_preprocess_shape_and_roi():
    img = np.zeros((480, 640, 3), np.uint8)
    img[:240] = 255                         # everything above the ROI is white
    x = preprocess(img)
    assert x.shape == (3, IMG_H, IMG_W) and x.dtype == np.float32
    assert x[0].max() < 0.05                # Y channel: the white sky is cropped
    assert preprocess(img, roi_frac=0.5, roi_bottom=0.75).shape == (3, IMG_H, IMG_W)


def test_flip_negates_label():
    cfg = AugConfig.off()
    cfg.p_flip = 1.0
    img = _frame_with_line(400)
    out, e = augment(img, 80.0, np.random.default_rng(0), cfg)
    assert e == -80.0
    assert abs(_bottom_line_x(out) - (639 - 400)) < 2


def test_lateral_shift_moves_label_with_the_road():
    cfg = AugConfig.off()
    cfg.p_shift = 1.0
    rng = np.random.default_rng(3)
    img = _frame_with_line(320)
    out, e = augment(img, 0.0, rng, cfg)
    moved = _bottom_line_x(out) - 320
    assert abs(e) > 1.0
    assert abs(moved - e) < 2.0             # label follows the bottom-row content


def test_photometric_aug_keeps_label_and_shape():
    cfg = AugConfig()
    cfg.p_flip = cfg.p_shift = 0.0
    img = _frame_with_line(300)
    for seed in range(20):
        out, e = augment(img, 12.5, np.random.default_rng(seed), cfg)
        assert e == 12.5 and out.shape == img.shape and out.dtype == np.uint8


def test_smoothing_skips_synthetic(tmp_path):
    errs = [0, 100, 0, 100, 0, 100, 0]
    _write_tub(tmp_path / "rec", errs, "recorded")
    _write_tub(tmp_path / "syn", errs, "synthetic")
    rec = read_tubs(str(tmp_path / "rec"), smooth_window=3)
    syn = read_tubs(str(tmp_path / "syn"), smooth_window=3)
    assert [s.error_px for s in syn] == errs
    assert max(s.error_px for s in rec) < 100


def test_smooth_series_is_centred():
    v = np.array([0, 0, 0, 30, 0, 0, 0], float)
    out = smooth_series(v, 3)
    assert out.shape == v.shape and np.allclose(out[2:5], [10, 10, 10])


def test_throttle_normalized(tmp_path):
    _write_tub(tmp_path / "t", [0, 1, 2], throttle=[0, 50, 200])
    s = read_tubs(str(tmp_path / "t"), throttle_max=100)
    assert [x.throttle for x in s] == [0.0, 0.5, 1.0]


def test_block_split_has_no_overlap():
    from vision.drive_data import Sample
    samples = [Sample(f"{i}", 0.0, 1.0) for i in range(1000)]
    tr, va = split_samples(samples, 0.2, block=50)
    assert len(tr) + len(va) == 1000 and va
    assert not {s.path for s in tr} & {s.path for s in va}
    # whole blocks only: every val block of 50 frames is complete
    blocks = {int(s.path) // 50 for s in va}
    assert len(va) == 50 * len(blocks)


def test_balance_upweights_rare_large_errors():
    from vision.drive_data import Sample
    samples = [Sample("a", 0.0, 1) for _ in range(90)] + \
              [Sample("b", 140.0, 1) for _ in range(10)]
    w = balance_weights(samples, 150.0)
    assert abs(w.sum() - 1.0) < 1e-9
    assert w[-1] > w[0] * 5


torch = pytest.importorskip("torch")


@pytest.mark.parametrize("arch", ["pilotnet", "mobilenet"])
@pytest.mark.parametrize("n_out", [2, 3])
def test_models_output_shape(arch, n_out):
    from vision.drive_net import build_model
    m = build_model(arch, n_out).eval()
    with torch.no_grad():
        y = m(torch.zeros(2, 3, IMG_H, IMG_W))
    assert tuple(y.shape) == (2, n_out)


def test_drivenet_roundtrip_pt_and_onnx(tmp_path):
    from vision.drive_net import DriveNet, build_model
    meta = {"arch": "mobilenet", "n_out": 3, "img_w": IMG_W, "img_h": IMG_H,
            "roi_frac": 0.5, "roi_bottom": 1.0, "norm_px": 150.0}
    m = build_model("mobilenet", 3).eval()
    pt = tmp_path / "drive_net.pt"
    torch.save({"model": m.state_dict(), "meta": meta}, pt)
    (tmp_path / "drive_net.json").write_text(json.dumps(meta))

    frame = _frame_with_line(350)
    net = DriveNet(pt, device="cpu")
    r = net.process(frame)
    assert 0.0 <= r.confidence <= 1.0 and r.heading == 0.0
    assert net.last_throttle is not None

    pytest.importorskip("onnxruntime")
    pytest.importorskip("onnx")
    onnx_path = tmp_path / "drive_net.onnx"
    try:
        torch.onnx.export(m, torch.zeros(1, 3, IMG_H, IMG_W), str(onnx_path),
                          input_names=["frame"], output_names=["out"],
                          opset_version=17, dynamo=False)
    except TypeError:
        torch.onnx.export(m, torch.zeros(1, 3, IMG_H, IMG_W), str(onnx_path),
                          input_names=["frame"], output_names=["out"],
                          opset_version=17)
    r2 = DriveNet(onnx_path).process(frame)
    assert abs(r2.error_px - r.error_px) < 1e-2
    assert abs(r2.confidence - r.confidence) < 1e-4


def test_drivenet_never_raises_on_bad_frame(tmp_path):
    from vision.drive_net import DriveNet, build_model
    pt = tmp_path / "drive_net.pt"
    torch.save({"model": build_model().state_dict(), "meta": {}}, pt)
    net = DriveNet(pt, device="cpu")
    r = net.process(np.zeros((0, 0, 3), np.uint8))
    assert r.confidence == 0.0
