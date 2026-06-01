# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Active system: TMR2026/

Everything under `TMR2026/` is the current vehicle. Legacy prototypes live in `_legacy/` and must not be imported from TMR2026.

**Root `main.py` is a loader** — it `chdir`s into `TMR2026/` and runs `TMR2026/main.py` with `runpy` so imports like `from hardware.motor import MotorDriver` keep working. The systemd service (`TMR2026/systemd/carrito_tmr.service`) points directly to `TMR2026/main.py --display` and starts under `graphical.target` (i.e. after the desktop is ready), with `DISPLAY=:0` and `XAUTHORITY=/home/angel01/.Xauthority` exported, so OpenCV can open a window on the HDMI monitor when VISION/AUTONOMOUS mode is entered. Root `main.py` is only for manual execution.

### Hardware Target

Raspberry Pi 5 with:
- Sony IMX500 NPU camera via `Picamera2` (RGB888 → BGR via `cv2.cvtColor(RGB2BGR)` — must preserve)
- IBT-2 H-bridge motor: BCM 18 (RPWM) + 13 (LPWM), `R_EN`/`L_EN` tied to 3.3 V
- PCA9685 servo on I²C bus 3 (dtoverlay GPIO 0/1), channel 0
- 2× VL53L0X ToF on I²C bus 4 (dtoverlay GPIO 23/22), addresses 0x30 (front) / 0x29 (rear), XSHUT pin `TMR2026/config.py:PIN_TOF_XSHUT_FRONT`
- Gamepad via `pygame` (PS4/Xbox) — buttons: A=MANUAL, B=VISION, X=AUTONOMOUS, Start=EMERGENCY. Hot-plug supported: `main.py:_pump_gamepad_events()` runs every loop iteration and reacts to SDL2 `JOYDEVICEADDED` / `JOYDEVICEREMOVED` events, so the PS4 (paired+trusted as `A0:5A:5F:0B:F7:5A`) connects automatically when powered on, even if the system booted without it. BlueZ has `AutoEnable=true` in `/etc/bluetooth/main.conf` so the BT controller comes up at boot ready to accept the trusted device.
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

### Vehicle lighting (signals + brake)
Three GPIO LEDs driven via `lgpio` chip 4 (BCM 19 left, 20 right, 16 brake — see `config.py`):
- `hardware/signals.py` — `TurnSignals` with modes `OFF / LEFT / RIGHT / HAZARD`. Blink at 2 Hz (TMR regulation) is computed each frame from `time.monotonic()`; no thread, no sleep. Caller must invoke `signals.tick()` every loop iteration.
- `hardware/brake_light.py` — simple `on()` / `off()` (idempotent — only writes GPIO on state change).
- `control/fsm.py:_apply_lights()` runs every tick (not just on transitions). In `CRUCERO`/`REANUDAR` it reads `steering.current_angle` vs `SERVO_CENTER`; deviation beyond `SIGNAL_DIR_THRESH_DEG` (6°) sets `LEFT` or `RIGHT`. In `PRECAUCION`/`FRENADO`/`ESPERA` it forces `HAZARD` and `brake_light.on()`. Anywhere else → all OFF.
- `main.py` mirrors this for non-FSM modes:
  - `_do_standby` / `_do_vision` → all signals OFF, brake OFF.
  - `_do_manual` → joystick `steer_raw < -0.15` → LEFT, `> +0.15` → RIGHT, else OFF. `brake_light.on()` when `motor.current_duty < -1.0` (reversing).
  - `signals.tick()` is called once per frame in the main loop, after `_run_mode()`, so blink is always advanced regardless of mode.

### Steering inversion
The servo is mounted reversed on this chassis. `config.py:STEERING_INVERTED = True` flips the physical write inside `SteeringDriver.set_angle()`:
- `physical = 2 * SERVO_CENTER_ANGLE - angle_deg` is sent to the servo.
- `current_angle` always returns the **logical** angle (90 = recto, <90 = izq, >90 = der).
- All consumers (FSM lights, PID, signals, telemetry) see the logical convention. Never invert per-mode in callers — fix it at the driver if hardware changes.

### Telemetry log lines
- `_do_manual` prints (carriage-return updated): `[MAN] steer:±x.xx (angle°)  t:y.yy  b:z.zz  duty:±NN%  signs:<label>@<cm>cm, …`
- `_log_autonomous` (called every tick after `fsm.update`) prints: `[AUT] <STATE>  err:±NNNpx  angle:NN.N°  duty:±NN%  lidar:NNNNmm  signs:<label>@<cm>cm, …`
- `_do_vision` prints `[VIS] err:±Npx conf:NN%  P/I/D:±x.xx  corr:±x.xx° angle:NN.N° lidar:NNmm signs:…` — same fields as the on-screen panel.
- `signs:` field shows up to 2 detections from `SignDetector.get_detections()`; `—` if empty.
- `PIDController` exposes the last computed components via public attrs `last_error / last_p / last_i / last_d / last_output`. Read-only — they are written every `compute()` and reset by `reset()`. Used by both the on-screen overlay and console logs.

### Debug display (`--display` flag)
When `python main.py --display` is set, the system opens a single OpenCV window `TMR 2026 - Vision Debug` whenever the mode is **VISION** *or* **AUTONOMOUS**. The window is closed automatically when leaving those modes for STANDBY/MANUAL.
- Renderer lives in `main.py:_render_debug_view(mode_label)` and is shared by both modes — do not duplicate it per mode.
- Layout: top half = BEV (left) + HSV white mask (right), bottom half = annotated frame with lane center line + YOLO bboxes.
- Two side-by-side overlay panels at y≈200: left = PID telemetry (`err`, `P/I/D`, `corr`, target servo angle, lidar); right = `OBJETOS DETECTADOS` list with up to 5 sign labels + confidence + distance.
- VISION mode brakes motors and centers steering, then *simulates* the PID purely for the overlay (servo never moves). `_set_mode` calls `pid.reset()` on entry/exit of VISION so the integrator does not contaminate AUTONOMOUS afterward.
- AUTONOMOUS mode does its normal work (FSM updates servo + motor) and additionally calls `_render_debug_view(mode_label="AUT")` after `_log_autonomous()`.

### Diagnostic preview tool: `tools/test_camera.py`
The "common test" entry point for camera/vision iteration. Imports CameraStream + LanePipeline + PIDController + SignDetector and renders the same overlay as `_render_debug_view` — but **never imports any GPIO hardware**, so it is safe to run with the systemd service active and on dev machines. Flags: `--no-yolo` skips loading the YOLO weights for instant startup. Exit with `q` or ESC.

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
- `TMR2026/tools/test_camera.py` — official preview tool (camera + lane + PID + YOLO, no motors). See "Diagnostic preview tool" above.

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
- **Steering inversion lives in `SteeringDriver.set_angle()` only** (driven by `config.py:STEERING_INVERTED`). Never re-invert in FSM, PID, signals, or per-mode code; always trust `current_angle` as the logical value.
- **`signals.tick()` must be called every main-loop iteration** (after `_run_mode()`), or LEDs freeze mid-blink.

## Vision tuning notes

- HSV white filter in `vision/lane_pipeline.py` is **configurable per instance** via the `hsv_white_lo` / `hsv_white_hi` constructor args (they shadow the class attrs). The **class default targets the physical track under medium-low light** (e.g. phone flashlight, not just direct lamp): `HSV_WHITE_LO = [0, 0, 130]`, `HSV_WHITE_HI = [179, 60, 255]`. `main.py` (Pi) and `tools/test_camera.py` use this default; `main_simulator.py` (Unity) overrides to a much brighter white (`[0,0,200]`/`[179,40,255]`) because the sim's lines are pure-white on a dark floor. So Pi and sim share the whole algorithm but each gets the right white threshold. If the black plastic track leaks into the mask under bright light, raise `V_min` toward 150–160; if dim conditions miss the lines, lower `V_min` toward 100–110. **Never re-tune the class default to the sim's bright values — that blinds the physical car.**
- Inspect the live mask via the top-right tile of `python main.py --display` (in VISION/AUTONOMOUS) or via `python tools/test_camera.py --no-yolo`.

## Known inconsistencies

- `TMR2026/main.py:46` hardcodes `SERVO_CHANNEL=0` but `config.py:SERVO_CHANNEL=15`. `main.py` wins at runtime because it doesn't import `SERVO_CHANNEL` from config.
- `TMR2026/main.py:51` hardcodes `TOF_XSHUT_PIN=17`. If new GPIO LEDs reuse pin 17 the ToF bring-up will fight them — keep LED pins off 17.
- LED pins (BCM 19/20/16) and `vision_config.yaml` `gpio:` block (BCM 5/6 hazard, 19/20 turns) overlap on 19/20. Production main.py uses 19/20 for turn signals; `vision_module.py` reads its own pins from the YAML — they live in separate processes so there's no live conflict, but don't run both at once.

## Common Pi-side gotchas

- `lgpio.error: 'GPIO not allocated'` on `python main.py` means the systemd service is holding pins. `TMR2026/main.py:_release_gpio_from_systemd()` now detects this on startup and runs `sudo -n systemctl stop carrito_tmr` automatically (passwordless sudo is configured for `angel01`). The function skips itself when launched *by* systemd (`INVOCATION_ID` env var is set), so the service can still run normally at boot.
- Old folders from the pre-reorg layout (`AUTO_YOLO/`, `CAMARA/`, `CONTROL/`, …) may need `sudo rm -rf` if they were created under root by a prior `sudo` run.
