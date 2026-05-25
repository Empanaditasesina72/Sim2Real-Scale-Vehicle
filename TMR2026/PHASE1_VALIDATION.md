# Phase 1 Sim2Real Validation Test Scenarios

This document specifies the three primary validation scenarios for **Phase 1** of the Sim2Real system. Each scenario collects CSV data to validate critical aspects of the autonomous vehicle control loop.

---

## Overview

| Scenario | Goal | Duration | Key Metrics | Expected Result |
|----------|------|----------|-------------|-----------------|
| **S1: Latency Measurement** | Validate perception-to-actuation cycle timing | 30 s | Frame latency, command latency, FSM response time | Latency < 150 ms end-to-end |
| **S2: PID Response (STOP)** | Validate deceleration accuracy and stopping distance | 20 s | Lane tracking error, servo angle response, motor duty curve | Stop at 270 ± 30 mm from STOP sign |
| **S3: FSM State Transitions** | Validate state machine robustness and timing | 60 s | State dwell times, transition counts, error during transitions | All states executed correctly, no unexpected exits |

---

## Scenario 1: Latency Measurement (S1_Latency)

### Objective
Measure and validate the **perception-to-actuation latency** — time from camera frame capture to motor/servo response. This is critical for competition safety and real-time validation.

### Setup

**Unity Simulator Configuration**:
- Scene: Simple straight track (no obstacles)
- Lighting: Uniform, white lane markings on dark background
- Physics: Realistic but stable (no vibration)
- Sensors:
  - Camera: 640×480, 30 FPS, capture timestamp recorded
  - Lidar: 50 Hz, constant distance ~500 mm (no obstacles)

**Simulator State**:
- Initial position: Centered on lane
- Initial velocity: 0 m/s
- Road curvature: 0 (straight)

### Test Sequence

1. **Startup Phase (5 s)**
   - Python main_simulator.py connects to Unity
   - Verify camera frames arriving at 30 FPS
   - Verify TOF data arriving at 50 Hz
   - Lane detection initializes

2. **Latency Injection Phase (20 s)**
   - Introduce small lane error (simulator offset camera by ±5 pixels horizontally)
   - Change lidar reading sinusoidally (300 mm → 700 mm → 300 mm)
   - Observe FSM entering PRECAUCIÓN/FRENADO states
   - Record timestamps of:
     - Frame capture (from Unity timestamp)
     - Python receipt (from recv thread)
     - Motor command sent (from main loop)
     - Servo angle changed (from steering driver)
     - Physical effect observed (simulated wheel movement)

3. **Shutdown Phase (5 s)**
   - Graceful disconnect
   - Finalize CSV log

### CSV Output: `S1_latency_results.csv`

```
timestamp_s,frame_id,capture_time_us,received_time_us,cmd_motor_time_us,cmd_servo_time_us,motor_duty,servo_angle,lidar_mm,fsm_state
0.000,0,0,1500,2200,2300,0.0,90.0,500,CRUCERO
0.033,1,33000,34500,35200,35300,0.0,90.0,500,CRUCERO
0.066,2,66000,67800,68500,68600,0.0,88.5,485,CRUCERO
0.099,3,99000,100500,105200,105300,-5.0,85.0,420,PRECAUCION
0.132,4,132000,133700,138500,138600,-8.0,82.0,350,FRENADO
```

### Acceptance Criteria

- [ ] Average frame latency (capture → Python receipt) ≤ 20 ms
- [ ] Average command latency (Python compute → send) ≤ 10 ms
- [ ] Total perception-to-actuation ≤ 150 ms (3 frames at 30 FPS)
- [ ] No lost frames (frame_id increases monotonically)
- [ ] No dropped TOF messages (lidar_mm updates every 20 ms)
- [ ] FSM state changes occur within expected latency

### Analysis Script

```python
import pandas as pd
import numpy as np

df = pd.read_csv('S1_latency_results.csv')

# Frame latency
frame_latency = (df['received_time_us'] - df['capture_time_us']) / 1000  # ms
print(f"Frame latency: {frame_latency.mean():.1f} ± {frame_latency.std():.1f} ms")

# Command latency
cmd_latency = (df['cmd_motor_time_us'] - df['received_time_us']) / 1000  # ms
print(f"Command latency: {cmd_latency.mean():.1f} ± {cmd_latency.std():.1f} ms")

# End-to-end
e2e_latency = (df['cmd_servo_time_us'] - df['capture_time_us']) / 1000  # ms
print(f"End-to-end latency: {e2e_latency.mean():.1f} ± {e2e_latency.std():.1f} ms")
```

---

## Scenario 2: PID Response Validation (S2_STOP)

### Objective
Validate that the **STOP sign detection and deceleration** logic works correctly. The vehicle must:
1. Detect STOP sign at distance
2. Enter PRECAUCIÓN state (reduce speed)
3. Enter FRENADO state (apply brakes)
4. Stop within 270 ± 30 mm of the sign

### Setup

**Unity Simulator Configuration**:
- Scene: Straight track with STOP sign at 700 mm from vehicle start position
- STOP sign: 180 mm tall (real size), positioned at track centerline
- Lidar: Accurate distance to sign (not simulated from bbox)
- Camera: Clear view of sign (high contrast, good lighting)

**Initial State**:
- Position: -700 mm from STOP sign (i.e., 700 mm away)
- Speed: 20% PWM (forward)
- Lane: Centered (error_px = 0)
- FSM: CRUCERO

### Test Sequence

1. **Approach Phase (10 s)**
   - Vehicle moves forward at 20% PWM
   - Sign detector begins detecting STOP sign (increasing confidence)
   - Lane tracking continues (should remain error_px ≈ 0)
   - Lidar distance decreases: 700 → 600 → 500 → 400 → 300 → 250 mm
   - FSM remains in CRUCERO (approaching but not in range)

2. **Detection & Response Phase (5 s)**
   - At lidar ≈ 500 mm: SignDetector confidence reaches threshold
   - FSM transitions: CRUCERO → PRECAUCIÓN (reduce speed to 10% PWM)
   - Lidar continues: 500 → 400 → 350 → 300 mm
   - At lidar ≈ 350 mm: FSM transitions PRECAUCIÓN → FRENADO (motor = 0, brake)
   - Vehicle decelerates, slowing down due to inertia
   - Lidar: 350 → 300 → 270 → 250 → ... (approaching stall)

3. **Stop & Wait Phase (5 s)**
   - Lidar stabilizes near 270 mm (target distance)
   - FSM enters ESPERA state (5 s mandatory pause)
   - Motor = 0, servo = 90° (straight)
   - Count down: 5.0 → 4.0 → 3.0 → 2.0 → 1.0 → 0.0 s

### CSV Output: `S2_stop_results.csv`

```
timestamp_s,fsm_state,motor_duty,servo_angle,lidar_mm,sign_confidence,lane_error_px,stop_distance_target_mm,stop_distance_actual_mm,error_mm
0.0,CRUCERO,20.0,90.0,700.0,0.0,0.5,270,700,430
0.2,CRUCERO,20.0,90.0,680.0,0.0,1.2,270,680,410
...
5.0,CRUCERO,20.0,90.0,500.0,0.25,0.8,270,500,230
6.0,PRECAUCION,10.0,90.0,450.0,0.45,1.5,270,450,180
7.0,PRECAUCION,10.0,90.0,380.0,0.68,2.1,270,380,110
8.0,FRENADO,0.0,90.0,320.0,0.85,1.9,270,320,50
9.0,FRENADO,0.0,90.0,280.0,0.95,0.5,270,280,10
10.0,FRENADO,0.0,90.0,268.0,0.98,0.2,270,268,-2
11.0,ESPERA,0.0,90.0,268.0,0.98,0.1,270,268,-2
...
16.0,ESPERA,0.0,90.0,268.0,0.98,0.0,270,268,-2
16.0,REANUDAR,0.0,90.0,268.0,0.98,0.0,270,268,-2
20.0,CRUCERO,15.0,90.0,270.0,0.0,0.3,270,270,0
```

### Acceptance Criteria

- [ ] Sign detected with ≥ 0.50 confidence at lidar ≤ 600 mm
- [ ] FSM enters PRECAUCIÓN within 1 s of detection
- [ ] FSM enters FRENADO before lidar < 350 mm
- [ ] Stop distance: 270 ± 30 mm (i.e., 240–300 mm)
- [ ] ESPERA state lasts ≥ 5.0 s
- [ ] Lane tracking error stays < 20 px during deceleration
- [ ] No motor oscillation (duty duty smoothly → 0)
- [ ] Servo remains centered (angle ≈ 90°, error < 2°)

### Analysis Script

```python
import pandas as pd

df = pd.read_csv('S2_stop_results.csv')

# Find FRENADO state
frenado_idx = df[df['fsm_state'] == 'FRENADO'].index
if len(frenado_idx) > 0:
    frenado_start = frenado_idx[0]
    frenado_end = frenado_idx[-1]
    
    # Stop distance accuracy
    stop_dist = df.loc[frenado_end, 'stop_distance_actual_mm']
    target = df.loc[frenado_end, 'stop_distance_target_mm']
    error = abs(stop_dist - target)
    
    print(f"Stop distance: {stop_dist:.0f} mm (target {target}, error {error:.0f} mm)")
    print(f"PASS" if error <= 30 else f"FAIL")
    
    # Deceleration profile
    frenado_data = df.loc[frenado_start:frenado_end]
    print(f"Deceleration time: {frenado_data['timestamp_s'].max() - frenado_data['timestamp_s'].min():.1f} s")

# Lane tracking during approach
approach_data = df[df['fsm_state'] == 'CRUCERO']
lane_error = approach_data['lane_error_px'].abs().max()
print(f"Max lane error during approach: {lane_error:.1f} px")
```

---

## Scenario 3: FSM State Transition Robustness (S3_FSM)

### Objective
Validate that the **FSM correctly transitions** between all states under various conditions. This ensures robust behavior under real-world conditions with changing sensor data.

### Setup

**Unity Simulator Configuration**:
- Scene: Complex track with multiple features:
  - Straightaway (30 m)
  - Gentle curve (15 m radius)
  - STOP sign at 700 mm
  - After STOP: resumption zone (straight)
  - Optional: obstacle to trigger PRECAUCIÓN
- Sensors: All enabled (camera, lidar, sign detector)

**Initial State**:
- FSM: CRUCERO
- Speed: 22% PWM
- Position: Start of straightaway

### Test Sequence

1. **Normal Cruising (15 s)**
   - Vehicle travels straight at SPEED_STRAIGHT (22% PWM)
   - Lane tracking maintains error < 20 px
   - FSM state: CRUCERO
   - Log: timestamps, PID output, servo angle

2. **STOP Sign Approach (10 s)**
   - Vehicle detects STOP sign at ~600 mm
   - FSM: CRUCERO → PRECAUCIÓN
   - Speed reduces to 10% PWM
   - Lidar distance decreases to ~350 mm
   - FSM: PRECAUCIÓN → FRENADO
   - Motor duty: 10% → 0%

3. **Stop & Wait (8 s)**
   - FSM: FRENADO → ESPERA
   - 5 s mandatory wait
   - Lidar stabilizes ~270 mm
   - Motor = 0, servo = 90°

4. **Resume After STOP (12 s)**
   - FSM: ESPERA → REANUDAR (soft-start)
   - Motor duty ramps 0% → 20% over 2 s
   - Lane tracking re-engages
   - FSM: REANUDAR → CRUCERO
   - Resume normal speed

5. **Lane Following + Gentle Curve (15 s)**
   - FSM: CRUCERO (maintaining lane)
   - PID adjusts servo angle for curvature
   - Speed may reduce if lane error > CURVE_THRESHOLD (0.30 rad)
   - Log: servo response to curvature

### CSV Output: `S3_fsm_results.csv`

```
timestamp_s,fsm_state_prev,fsm_state,dwell_time_s,lane_error_px,pid_output,servo_angle,motor_duty,sign_detected,lidar_mm,event
0.0,—,CRUCERO,0.0,2.5,0.20,92.0,22.0,False,—,START
1.0,CRUCERO,CRUCERO,1.0,1.8,0.14,91.1,22.0,False,—,—
...
15.0,CRUCERO,CRUCERO,15.0,-0.5,-0.04,89.9,22.0,True,600,SIGN_DETECTED
15.2,CRUCERO,PRECAUCION,0.2,0.3,0.02,90.0,10.0,True,590,TRANSITION_1
...
17.5,PRECAUCION,FRENADO,2.3,1.2,0.09,90.3,0.0,True,350,TRANSITION_2
...
22.5,FRENADO,ESPERA,5.0,0.0,0.0,90.0,0.0,True,270,TRANSITION_3
...
27.5,ESPERA,REANUDAR,5.0,0.5,0.04,90.1,5.0,False,270,TRANSITION_4
29.5,REANUDAR,CRUCERO,2.0,1.8,0.14,91.2,22.0,False,—,TRANSITION_5
...
45.0,CRUCERO,CRUCERO,17.5,3.2,0.26,92.6,15.0,False,—,CURVE_MODE
60.0,CRUCERO,CRUCERO,32.5,-1.5,-0.12,88.5,22.0,False,—,TEST_COMPLETE
```

### Acceptance Criteria

- [ ] All state transitions occur (CRUCERO → PRECAUCIÓN → FRENADO → ESPERA → REANUDAR → CRUCERO)
- [ ] State dwell times match expectations:
  - CRUCERO: variable (depends on track)
  - PRECAUCIÓN: 2–5 s
  - FRENADO: 2–5 s
  - ESPERA: exactly 5.0 ± 0.5 s
  - REANUDAR: 2–3 s
- [ ] No spurious transitions (e.g., CRUCERO ↔ PRECAUCIÓN oscillation)
- [ ] Lane error remains < 50 px during curve (CURVE_THRESHOLD = 0.30 rad)
- [ ] Motor duty smooth transitions (no jumps > 5%)
- [ ] Servo angle responsive to lane error (PID gains working)

### Analysis Script

```python
import pandas as pd

df = pd.read_csv('S3_fsm_results.csv')

# State transition summary
transitions = df[df['event'].str.contains('TRANSITION')]
print("FSM State Transitions:")
for idx, row in transitions.iterrows():
    prev = row['fsm_state_prev']
    curr = row['fsm_state']
    dwell = row['dwell_time_s']
    print(f"  {prev} → {curr} (dwell: {dwell:.1f} s)")

# Check ESPERA timing
espera = df[df['fsm_state'] == 'ESPERA']
if len(espera) > 0:
    espera_time = espera['dwell_time_s'].iloc[0]
    print(f"\nESPERA state duration: {espera_time:.2f} s (target: 5.0 s)")
    if 4.5 <= espera_time <= 5.5:
        print("  PASS")
    else:
        print("  FAIL")

# Lane tracking during transitions
for state in ['CRUCERO', 'PRECAUCION', 'FRENADO', 'REANUDAR']:
    state_data = df[df['fsm_state'] == state]['lane_error_px'].abs()
    if len(state_data) > 0:
        max_error = state_data.max()
        mean_error = state_data.mean()
        print(f"\n{state}:")
        print(f"  Lane error: {mean_error:.1f} px (max {max_error:.1f} px)")
```

---

## Running Phase 1 Validation

### Prerequisites

1. **Unity Simulator running** on `127.0.0.1:5005`:
   ```bash
   # In Unity Editor or built executable
   # Ensure TcpListener on port 5005 is active
   ```

2. **Python environment**:
   ```bash
   cd TMR2026
   pip install -r requirements.txt
   # Install test dependencies:
   pip install pandas matplotlib scikit-learn
   ```

3. **Weights & models**:
   ```bash
   # Verify YOLOv8 model exists
   ls weights/tmr_signs.pt  # Should exist
   ```

### Running Tests

```bash
# Test 1: Latency (30 seconds)
python -c "
from main_simulator import VehicleSimulator
sim = VehicleSimulator()
sim.set_mode(sim.Mode.VISION)
import time; time.sleep(5)  # Warmup
sim.set_mode(sim.Mode.AUTONOMOUS)
metrics = sim.run_autonomous_test(duration_s=30)
sim.save_metrics_csv('S1_latency_results.csv')
"

# Test 2: STOP Sign (20 seconds)
# (Requires Unity scene with STOP sign at specific distance)
python -c "
from main_simulator import VehicleSimulator
sim = VehicleSimulator()
sim.set_mode(sim.Mode.AUTONOMOUS)
metrics = sim.run_autonomous_test(duration_s=20)
sim.save_metrics_csv('S2_stop_results.csv')
"

# Test 3: FSM (60 seconds)
# (Requires full track with STOP sign, curves, etc.)
python -c "
from main_simulator import VehicleSimulator
sim = VehicleSimulator()
sim.set_mode(sim.Mode.AUTONOMOUS)
metrics = sim.run_autonomous_test(duration_s=60)
sim.save_metrics_csv('S3_fsm_results.csv')
"
```

### Analysis

After each test, run the analysis scripts above to generate summary statistics and verify pass/fail criteria.

---

## Expected Validation Results (Academic Article)

Each scenario produces:
- **CSV file** with 100+ data points per second
- **Summary statistics** (mean, std dev, min/max for each metric)
- **Plots** (lane error over time, PID response, FSM state timeline)
- **Pass/Fail verdict** against acceptance criteria

These results form the **empirical validation** section of the TMR 2026 article:
- "Our implementation achieved perception-to-actuation latency of XX ms, validating real-time constraints."
- "STOP sign detection and deceleration achieved stopping distance of XX ± YY mm, meeting competition requirements."
- "The FSM transitioned through all N states without spurious errors over 60-second operation."

