# Sim2Real Simulator Calibration (Unity ↔ PC)

> English version of `CALIBRACION_SIM.md`.

**Simulator** configuration (updated 2026-06-01). When the **real physical car**
calibration is available, compare and tune these values so Unity replicates the
real behavior.

---

## 1. Vehicle camera (Unity — `VehicleBuilder.cs`)

| Parameter | Current value (sim) | Notes |
|-----------|---------------------|-------|
| Local position | `(0, 0.22, 0.30)` | In front of the car, 22 cm high (= real Pi height) |
| Rotation | `Euler(10, 0, 0)` | 10° downward |
| FOV | `60` | Same as the physical Pi camera |
| Near / Far | `0.01 / 20` | |
| Background | grey `#8A8A8A` | Not white (avoids a "white screen") |
| RenderTexture | `320×240` | Upscaled to 640×480 on the PC |

**Key lesson:** the car's 3D model was blocking the camera. Fix: place the camera
IN FRONT of the car, looking slightly downward (if horizontal it sees the sky).

---

## 2. Track (Unity — `SceneBuilder.cs`)

| Element | Value |
|---------|-------|
| Track length | `60 m` (center at `z = 30`) |
| Lane width (line to line) | `54 cm` → lines at `x = ±0.27` |
| Ground | grey `#6E6E6E` (dark, contrasts with the white lines) |
| Lines (left/right) | **white** `#FFFFFF`, 4 cm wide |
| Dashed center | white, segments every `0.8 m` |
| STOP | at `z = 12 m`, right side (`x = 0.32`), **solid red panel** (no image) |
| Parking zone | at `z = 30 m` (cars + gap) |

---

## 3. Lane HSV filter (`vision/lane_pipeline.py`) — configurable

```python
# Unity simulator (VERY bright lines on a dark floor):
hsv_white_lo = [0,  0, 200]
hsv_white_hi = [179, 40, 255]

# Physical Pi (class default, medium-low light):
HSV_WHITE_LO = [0,  0, 130]
HSV_WHITE_HI = [179, 60, 255]
```
> The HSV is a constructor parameter: each camera uses its own threshold.

## 4. Bird's-Eye View (BEV) — simulator only (`main_simulator.py`)

```python
roi_frac = 0.30        # BEV looks farther ahead (both lane lines visible)
# bev_src_ratio is NOT overridden → uses the default LanePipeline trapezoid
```

## 5. Lane bias

```python
right_bias = 0.75   # sim (0.5=center, 1.0=right line). Pi default = 0.70.
```

## 6. Steering (servo) — INVERSION

The physical car's servo is mounted reversed. The simulator replicates it:
```python
# MockSteeringDriver
STEERING_INVERTED = True
physical = 2*90 - logical_angle   # the physical angle is sent to Unity
```
**Without this inversion the car turns the wrong way** (it drifts to the left line
instead of the right lane).

## 7. Steering PID (lane error → servo angle)

```python
PID_KP = 0.08
PID_KI = 0.002
PID_KD = 0.025
```

## 8. FSM — STOP behavior

| Parameter | Value |
|-----------|-------|
| Camera braking trigger | ~320 mm (brakes when the sign is close) |
| `STOP_TARGET_MM` | 270 mm (TMR rule: 270 ± 30) |
| `ESPERA_S` | 5.0 s |
| Real STOP octagon height | `0.04 m` (4 cm) |
| `CAMERA_FOCAL_LENGTH_PX` | 490 |

---

## HOW TO REPLICATE THE PHYSICAL CAR IN UNITY

When you have the photo/data of the physical car:
1. **Camera**: measure the real height (cm) and tilt angle → adjust
   `cameraMount.localPosition.y` and `Euler(x,...)`.
2. **Lane**: confirm the real width (54 cm?) → adjust the lines' `x`.
3. **BEV view**: compare the Pi's view vs Unity's; tune `roi_frac`/BEV until the
   lines are vertical in the bird's-eye view.
4. **Colors**: match the track grey and the line brightness to the real ones.
5. Verify that `err` (px) has the same sign and magnitude in sim and real.
