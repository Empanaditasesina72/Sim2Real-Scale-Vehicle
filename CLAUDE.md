# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Active system: TMR2026/

Everything under `TMR2026/` is the current vehicle. Legacy prototypes live in `_legacy/` and must not be imported from TMR2026.

**Root `main.py` is a loader** — it `chdir`s into `TMR2026/` and runs `TMR2026/main.py` with `runpy` so imports like `from hardware.motor import MotorDriver` keep working. The systemd service (`TMR2026/systemd/carrito_tmr.service`) still points directly to `TMR2026/main.py`; root `main.py` is only for manual execution.

### Hardware Target

Raspberry Pi 5 with:
- Sony IMX500 NPU camera via `Picamera2` (RGB888 → BGR via `cv2.cvtColor(RGB2BGR)` — must preserve)
- IBT-2 H-bridge motor: BCM 18 (RPWM) + 13 (LPWM), `R_EN`/`L_EN` tied to 3.3 V
- PCA9685 servo on I²C bus 3 (dtoverlay GPIO 0/1), channel 0
- 2× VL53L0X ToF on I²C bus 4 (dtoverlay GPIO 23/22), addresses 0x30 (front) / 0x29 (rear), XSHUT pin `TMR2026/config.py:PIN_TOF_XSHUT_FRONT`
- Gamepad via `pygame` (PS4/Xbox) — buttons: A=MANUAL, B=VISION, X=AUTONOMOUS, Start=EMERGENCY
- GPIO LEDs for turn signals / hazards / brake — pins defined in `TMR2026/vision_config.yaml` → `gpio:` and mirrored in `config.py`

GPIO is accessed via `lgpio` (chip 4 on Pi 5) with a `RPi.GPIO` fallback.

## Running the System

```bash
# From repo root (recommended for manual runs)
python main.py               # production
python main.py --display     # with debug window

# Direct (what systemd uses)
python TMR2026/main.py
```

Runtime modes (cycled via gamepad): `STANDBY → MANUAL → VISION → AUTONOMOUS`. `Start` button = emergency freeze.

## Installing Dependencies

```bash
pip install -r TMR2026/requirements.txt
# Pi-specific extras:
pip install picamera2 lgpio adafruit-circuitpython-vl53l0x adafruit-circuitpython-pca9685 ultralytics
```

See `TMR2026/SETUP.md` for dtoverlay config and udev rules.

## Architecture (TMR2026/)

### Threads
- `CameraStream` (vision/camera_stream.py) — 30 FPS, BGR frames, locks AE/AWB after warmup
- `SignDetector` (vision/sign_detector.py) — ~12 FPS YOLO CPU, loads `weights/tmr_signs.pt`
- `DistanceSensor` (hardware/distance_sensor.py) — 50 Hz polling, front + rear VL53L0X
- `MotorDriver` (hardware/motor.py) — internal 50 Hz soft-start ramp thread (prevents voltage sag)
- Main loop in `main.py` at 50 Hz: gamepad → FSM → servo → motor

### Perception → decision → actuation
- `vision/lane_pipeline.py` — BEV + HSV-white + sliding windows + EMA; emits `LaneResult(error_px, confidence)`
- `vision/sign_detector.py` — non-blocking queue of `Detection(label, confidence, bbox)`; only `stop_sign` and `crosswalk` labels are surfaced
- `control/fsm.py` — 5-state FSM: `CRUCERO → PRECAUCION → FRENADO → ESPERA → REANUDAR`. Stop wait uses `time.monotonic()`, never `sleep()`. `brake()` is instantaneous and must not be wrapped/changed
- `control/pid_controller.py` — generic PID with anti-windup and derivative-on-measurement, used for steering (lane error → servo angle)

### Alternative modules (exist but not wired into main.py)
These are full implementations kept for future wiring. Treat as library code:
- `hardware/camera_manager.py` — IMX500 NPU-side inference (alternative to CPU sign detector)
- `hardware/motor_driver.py` — simpler lgpio-only motor (alternative to soft-start version)
- `vision/lane_detector.py` — classic ROI/threshold/histogram lane detector + crosswalk detection
- `vision/object_detector.py` — HSV traffic-light classifier + STOP distance via bbox + overtake/parking cues
- `control/gamepad_reader.py` — threaded gamepad reader at 100 Hz (main.py uses pygame directly)
- `autonomy/autonomous_mode.py` — advanced 9-state FSM (CROSSWALK_STOP, OVERTAKING_*, PARKING, OBSTACLE_HOLD)
- `autonomy/parking_maneuver.py` — Ackermann-based parallel parking sub-FSM

### Personal test scripts (do not wire into main.py)
- `vision_module.py` — user's standalone camera experiment with its own 9-state FSM and its own hazard/turn-signal implementation via `lgpio` chip 4. Pins come from `vision_config.yaml`.
- `test_gamepad.py`, `test_servo.py`, `test_vision.py` — diagnostics.

## YOLO Models

- `TMR2026/weights/tmr_signs.pt` — active model loaded by `SignDetector`. Trained from `yolov8n.pt` on `traffic_lights/data.yaml` (7 classes: `green, left, red, right, stop, straight, yellow`). Only `stop` is currently surfaced by the filter in `sign_detector.py`.
- `_legacy/runs/detect/train2/weights/` — source of the active model (checkpoint + training artifacts).
- `_legacy/runs/detect/train/weights/best.pt` — larger variant (~18 MB) kept as backup.
- `traffic_lights/` — Roboflow v9 dataset (1470 close-up sign images, no track photos). Use to re-train if adding a `crosswalk` class.

## Hard rules (don't break these)

- **Never modify `motor.brake()`** — it must remain an instantaneous hard-cut to 0.
- **Never remove `cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)`** in `vision/camera_stream.py`.
- **Never edit `vision_module.py`** — it's the user's personal camera experiment. Its hazard/turn-signal code is independent from the production `hardware/signals.py` module.
- **Never import from `_legacy/`** inside `TMR2026/`.
- **ESPERA state must use `time.monotonic()`**, not `time.sleep()` — the loop must keep serving the FSM.
- Turn-signal / hazard blink rate is `2 Hz` (per TMR regulation).

## Known inconsistencies

- `TMR2026/main.py:46` hardcodes `SERVO_CHANNEL=0` but `config.py:SERVO_CHANNEL=15`. `main.py` wins at runtime because it doesn't import `SERVO_CHANNEL` from config.
- `TMR2026/main.py:51` hardcodes `TOF_XSHUT_PIN=17`. If new GPIO LEDs reuse pin 17 the ToF bring-up will fight them — keep LED pins off 17.
