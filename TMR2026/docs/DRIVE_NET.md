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
# lighter net, recorded data: smooth the teacher's jitter, oversample curves
python TMR2026/tools/train_drive.py --data datasets/track_real --arch mobilenet \
    --smooth 5 --balance --shift-gain <BEV px per image px> --epochs 60
```
Mixing the synthetic tub in as extra data regularizes against overfitting one
track. Output: `weights/drive_net.pt` (best val RMSE, saved on every
improvement) + `drive_net.json` (preprocessing meta) + `drive_net_last.pt` +
`drive_history.json` + `drive_train.png`. Watch **val RMSE in px** go down;
training stops by itself after `--patience` epochs without a `--min-delta` px
improvement.

**No GPU at hand?** Open `TMR2026/notebooks/train_colab.ipynb` in Google Colab
(or Kaggle): it clones the repo, mounts the dataset (Drive / Kaggle input / zip
/ synthetic), previews the augmentation, trains with early stopping, plots
loss/MSE/RMSE/MAE, evaluates and exports — then saves everything to Drive.

| Flag | Effect |
|---|---|
| `--arch pilotnet\|mobilenet` | PilotNet (~0.25 M params) or MobileNetV2-style (~0.17 M) |
| `--smooth N` | centred moving average over N frames of **recorded** labels (synthetic rows are never smoothed) |
| `--balance` | inverse-frequency sampling over \|error\| bins — stops the net learning "always straight" |
| `--throttle` | third head on the tubs' `throttle` column (normalized by `--throttle-max`) |
| `--shift-gain` | label px per image px for the lateral-shift augmentation; 1.0 for synthetic tubs, the BEV/image scale for `LanePipeline`-labelled tubs |
| `--roi-frac`, `--roi-bottom` | rows of the frame the net sees |
| `--loss huber\|mse` | Huber (default) shrugs off teacher-label outliers |
| `--patience`, `--min-delta` | early stopping |

### 3. Evaluate

```bash
python TMR2026/tools/test_drive_net.py --tub datasets/sim_trackC   # RMSE vs expert
python TMR2026/tools/test_drive_net.py --camera                    # live (Pi)
python TMR2026/tools/test_drive_net.py --weights weights/drive_net_int8.onnx --camera
```

### 4. Export for the Pi

```bash
python TMR2026/tools/export_drive.py                                   # TorchScript + ONNX
python TMR2026/tools/export_drive.py --fp16 --int8 --calib datasets/track_real --bench
python TMR2026/tools/export_drive.py --tflite --int8 --calib datasets/track_real
```

| File | Runtime on the Pi | Notes |
|---|---|---|
| `drive_net.onnx` | onnxruntime | FP32 reference |
| `drive_net_fp16.onnx` | onnxruntime | half the size, float32 I/O |
| `drive_net_int8.onnx` | onnxruntime | static INT8, per-channel, calibrated on `--calib` frames |
| `drive_net_fp16.tflite` / `drive_net_int8.tflite` | ai-edge-litert | via onnx2tf; INT8 has int8 I/O, handled by `DriveNet` |

**Always calibrate INT8 on real track frames** — without `--calib` the ranges
come from noise. The table the exporter prints gives each file's steering RMSE
against the PyTorch model (synthetic test: 0.16–0.23 px for INT8). Run `--bench`
on the Pi itself for FPS; numbers from a PC say nothing about the car.

### 5. Enable it on the car

Copy the chosen export **and `drive_net.json`** into `TMR2026/weights/`, then in
`config.py`:
```python
USE_DRIVE_NET      = True
DRIVE_NET_WEIGHTS  = "weights/drive_net_int8.onnx"   # or drive_net.pt / .tflite
DRIVE_NET_CONF_MIN = 0.30
```
`main.py` and `main_simulator.py` then swap `LanePipeline` for `DriveNet`
automatically (guarded: if the weights or the runtime are missing, they fall
back to the classic pipeline and print a notice). Everything downstream is
unchanged — AUTONOMOUS mode feeds `result.error_px` to the same FSM/PID, and
`--display` shows the `[NET]` overlay. Default is `False`, so the car's
behavior does not change until you flip the flag.

---

## Architecture

- **Input**: rows `[roi_frac, roi_bottom)` of the BGR frame (ROI: the road, not
  the walls), resized to 200×66, converted to YUV, /255. Preprocessing lives once
  in `drive_net.preprocess()` and is shared by training, evaluation, export
  calibration and inference so they can never drift.
- **PilotNet**: 5 conv (BN + ELU) + adaptive pool to 1×18 (keeps horizontal
  resolution = lateral lane position) + 4 FC. ~250 k params, ~1 MB.
- **MobileNet**: MobileNetV2 inverted residuals (1×1 expand → 3×3 depthwise →
  1×1 linear), ReLU6 + BN, height-only average pool that keeps 13 horizontal
  cells, 2 FC. ~170 k params, ~0.7 MB FP32 / ~0.25 MB INT8.
- **Outputs**: `[error_norm, confidence_logit(, throttle_logit)]`.
- **Target**: `clip(error_px / norm_px, ±1.5)` (default `norm_px=150`), plus a
  confidence the FSM can treat like a weak classic detection.
- **Augmentation** (`vision/drive_data.py`): flip (error negated), perspective
  lateral shift (error compensated), brightness/contrast/gamma, HSV jitter,
  soft shadows, specular glare, Gaussian/motion blur, noise, JPEG artefacts.
  This is what buys generalization across lighting and tracks.

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
