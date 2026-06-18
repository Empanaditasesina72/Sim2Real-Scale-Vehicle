# DriveNet — End-to-End Steering by Behavioral Cloning

An optional learned replacement for the classic CV lane follower
(`vision/lane_pipeline.py`). A small PilotNet-style CNN looks at the camera
frame and predicts the **lane error in pixels** — the exact same signal
`LanePipeline.process()` produces — so it drops into the existing system without
touching the FSM, the PID, the sign gating or the vehicle lights.

> **Why a network at all?** The classic pipeline needs per-track calibration
> (BEV trapezoid + HSV white threshold). A network trained with heavy
> augmentation learns to read "road-like" lanes under many lightings/curvatures,
> so it generalizes to *any basic road-like track* with far less hand-tuning.
> The trade-off: it only knows what it was trained on — **it cannot train
> without data.**

---

## The hard requirement: data

Behavioral cloning is supervised learning over `(image -> steering)` pairs.
**With no data there is nothing to train.** You have two free sources of labels,
neither of which needs hand annotation:

1. **The Unity simulator** (`TMR2026_Sim`) — drive/record across many tracks and
   lightings. Domain randomization for free.
2. **The classic CV pipeline as a teacher** — `lane_pipeline.process(frame)`
   already maps image → error. `tools/record_driving.py` uses it to auto-label
   every frame. The network imitates the pipeline but generalizes.

If you have neither yet, `tools/gen_synth_driving.py` renders domain-randomized
synthetic roads so you can build, validate and pre-train the whole pipeline
today, then fine-tune on real data later.

---

## Files

| File | Role |
|---|---|
| `vision/drive_net.py` | PilotNet model + `DriveNet` wrapper (drop-in for `LanePipeline`) |
| `tools/gen_synth_driving.py` | Synthetic domain-randomized tub (no sim/track needed) |
| `tools/record_driving.py` | Real tub from the Unity sim / Pi camera / video / images |
| `tools/train_drive.py` | Train `weights/drive_net.pt` with augmentation |
| `tools/test_drive_net.py` | Offline evaluation / preview (no GPIO) |
| `tools/export_drive.py` | TorchScript / ONNX export for the Pi |

All tools write/read one **tub** format:

```
<tub>/frames/000000.jpg, 000001.jpg, ...
<tub>/labels.csv      header: frame,error_px,confidence,source
<tub>/tub.json        {"width","height","count",...}
```

`error_px` follows the `LanePipeline` convention: `lane_centre - frame_centre`
in pixels, positive = the lane is to the right (steer right).

---

## Workflow

### 0. (Optional) Validate everything on synthetic data — works with no hardware

```bash
python TMR2026/tools/gen_synth_driving.py --n 2000 --out datasets/drive_synth --seed 0
python TMR2026/tools/gen_synth_driving.py --n 500  --out datasets/drive_synth_val --seed 999
python TMR2026/tools/train_drive.py --data datasets/drive_synth --val datasets/drive_synth_val --epochs 15
python TMR2026/tools/test_drive_net.py --tub datasets/drive_synth_val      # RMSE + montage
python TMR2026/tools/export_drive.py                                        # TorchScript
```

### 1. Record a real dataset

**From the Unity sim (recommended — it drives itself and auto-labels):**
```bash
# Unity must be listening on 127.0.0.1:5005 (same as main_simulator.py)
python TMR2026/tools/record_driving.py --source sim --max 5000 --throttle 18
```
Drive several different tracks/lighting setups, appending to new tubs
(`--out datasets/sim_trackA`, `datasets/sim_trackB`, ...). More variety = better
generalization.

**From the physical car (no sim2real gap):**
```bash
# turn captured track photos into an expert-labeled tub
python TMR2026/tools/capture_track.py --auto 0.5
python TMR2026/tools/record_driving.py --source images \
    --path TMR2026/tools/captures --out datasets/track_real
```

**Clone your own driving instead of the CV expert:**
```bash
python TMR2026/tools/record_driving.py --source sim --label human --max 6000
```

### 2. Train

```bash
python TMR2026/tools/train_drive.py \
    --data datasets/sim_trackA,datasets/sim_trackB,datasets/drive_synth \
    --val datasets/sim_trackC --epochs 60 --batch 64 --device cuda
```
Mixing the synthetic tub in as extra data regularizes against overfitting one
track. Output: `weights/drive_net.pt` + `drive_net.json` (preprocessing meta) +
`drive_train.png` (loss / val-RMSE curve). Watch **val RMSE in px** go down.

### 3. Evaluate

```bash
python TMR2026/tools/test_drive_net.py --tub datasets/sim_trackC   # RMSE vs expert
python TMR2026/tools/test_drive_net.py --camera                    # live (Pi)
```

### 4. Export for the Pi

```bash
python TMR2026/tools/export_drive.py            # weights/drive_net.torchscript
python TMR2026/tools/export_drive.py --onnx     # + ONNX (needs onnx + onnxscript)
# NCNN (fastest on the Pi 5 ARM CPU):
pip install pnnx && pnnx weights/drive_net.torchscript inputshape=[1,3,66,200]
```

### 5. Enable it on the car

In `config.py`:
```python
USE_DRIVE_NET      = True
DRIVE_NET_WEIGHTS  = "weights/drive_net.pt"
DRIVE_NET_CONF_MIN = 0.30
```
`main.py` and `main_simulator.py` then swap `LanePipeline` for `DriveNet`
automatically (guarded: if the weights or torch are missing, they fall back to
the classic pipeline and print a notice). Everything downstream is unchanged —
AUTONOMOUS mode feeds `result.error_px` to the same FSM/PID, and `--display`
shows the `[NET]` overlay. Default is `False`, so the car's behavior does not
change until you flip the flag.

---

## Architecture

- **Input**: lower part of the BGR frame (ROI), resized to 200×66, converted to
  YUV, normalized. Preprocessing lives once in `drive_net.preprocess()` and is
  shared by training, evaluation and inference so they can never drift.
- **Model**: 5 conv + adaptive pool (keeps horizontal resolution = lateral lane
  position) + 4 FC → 2 outputs `[error_norm, confidence_logit]`. ~250k params,
  ~1 MB, runs in a few ms on the Pi 5 CPU.
- **Target**: `error_px / norm_px` (default `norm_px=150`), plus a confidence the
  FSM can treat like a weak classic detection.
- **Augmentation** (in `train_drive.py`): horizontal flip (error negated),
  horizontal shift (error compensated), brightness/contrast/gamma, HSV jitter,
  random shadows, blur, noise. This is what buys track generalization.

---

## Detector (signs) retraining — `tools/train_signs.py`

Separate from steering: the 7-class YOLO sign detector. Retrain with
generalization augmentation when you have track images of the signs:

```bash
python TMR2026/tools/train_signs.py --epochs 120 --imgsz 640 --device 0
```

**Flips are disabled on purpose** (`fliplr=flipud=0`) because the dataset has
directional arrow classes (left/right/straight) — a mirrored "left" arrow would
be mislabeled. The biggest accuracy win is adding real track photos of the signs
to `traffic_lights/`, not more augmentation on the close-up set.

GPU: CUDA is set up on this PC — `torch 2.12.0+cu126` on the GTX 1650, so
`--device 0` works and `train_drive.py` / `train_signs.py` auto-select it. To
reinstall on a fresh machine (Python 3.14):
`pip install torch==2.12.0+cu126 torchvision==0.27.0+cu126 --index-url https://download.pytorch.org/whl/cu126`
(cu128 has no torch 2.12 build). Note: `train_drive` is data-loading bound (the
net is tiny) — add `--workers 4` to actually benefit; the GPU's big win is
`train_signs` (YOLO, imgsz 640). After retraining, regenerate both deploy
exports (`export_model.py` NCNN + `export_imx500.py` rpk).
