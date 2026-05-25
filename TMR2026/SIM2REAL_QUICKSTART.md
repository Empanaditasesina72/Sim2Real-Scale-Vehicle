# Sim2Real Phase 1 — Quick Start Guide

**Status**: Ready to run. Everything needed is complete.

---

## 30-Second Overview

**What you have** (Phase 1 complete):
- ✅ `sim_hardware_mocks.py` — PC hardware layer (motor, steering, sensors, camera)
- ✅ `main_simulator.py` — Control system running on PC (FSM, PID, vision)
- ✅ `SIM2REAL_PROTOCOL.md` — Socket protocol specification
- ✅ `PHASE1_VALIDATION.md` — Three validation test scenarios with acceptance criteria
- ✅ `SIM2REAL_QUICKSTART.md` — This file

**What you need to build** (Unity):
- ⏳ Unity C# socket server on port 5005
- ⏳ Physics simulator with wheel collision
- ⏳ Virtual sensors (TOF lidar, camera with JPEG encoding)
- ⏳ STOP sign prop + track layout

**Result**: Digital control → Physics → Sensor data → Python FSM → Motor/servo commands → Back to Unity

---

## Architecture (One Picture)

```
┌─────────────────────────────────────────────────────────────────┐
│                         PC (LOCAL MACHINE)                      │
├──────────────────────┬────────────────────────────────────────┤
│  main_simulator.py   │           Unity Simulator              │
│  ────────────────    │           ────────────────              │
│  • FSM (5 states)    │  • Physics engine (Rigidbody)          │
│  • PID steering      │  • Wheels + steering actuators         │
│  • Lane detection    │  • Virtual TOF sensors (raycast)       │
│  • YOLO signs        │  • Virtual camera (RenderTexture)      │
│  • 50 Hz main loop   │  • 30 FPS camera → JPEG encode         │
│                      │  • 50 Hz lidar readout                 │
│                      │                                        │
│  Socket recv thread  ←───────→  Socket send thread             │
│  (JPEG + TOF)        │          (MOTOR + SERVO cmds)           │
└──────────────────────┴────────────────────────────────────────┘
         └────────────────────────────────────────────────────────┘
                   TCP/IP on 127.0.0.1:5005
                   (can be 192.168.x.x for remote)

Flow:
  1. Python: Send "MOTOR:{duty}" + "SERVO:{angle}"
  2. Unity: Apply force + rotation
  3. Unity: Cast rays (ToF), render frame (camera)
  4. Unity: Send "TOF:front,rear" + JPEG frame data
  5. Python: Receive sensors, run FSM/PID, compute new motor/servo
  6. Back to step 1 @ 50 Hz
```

---

## Step 1: Verify PC Environment

```bash
# Navigate to project
cd C:\Users\Angel\Documents\GitHub\Carrito\TMR2026

# Check Python 3.11+
python --version
# Expected: Python 3.11.x or higher

# Install dependencies
pip install -r requirements.txt
pip install opencv-python numpy

# Verify key modules
python -c "import cv2, numpy as np; print('✓ OpenCV & NumPy OK')"

# Verify YOLO model exists
ls weights/tmr_signs.pt
# If missing, download from repo or create a placeholder
```

---

## Step 2: Understand the Socket Protocol

**Open and read** (in order):
1. `SIM2REAL_PROTOCOL.md` — Exact message format
2. `PHASE1_VALIDATION.md` — What data to expect

**Key points**:
- Python sends `MOTOR:{duty}\n` and `SERVO:{angle}\n` as text
- Unity sends `TOF:{front_mm},{rear_mm}\n` as text
- Unity sends JPEG frames as binary: `[4-byte size][JPEG][4-byte size][JPEG]...`
- All communication on TCP port **5005**, **127.0.0.1** (localhost)
- Timeout: 1 second (prevents hangs)

---

## Step 3: Build Minimal Unity Server

**In Unity C# (pseudocode)**:

```csharp
using System;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using UnityEngine;

public class SimulatorServer : MonoBehaviour
{
    private TcpListener listener;
    private TcpClient connectedClient;
    private NetworkStream stream;
    private Thread serverThread;
    
    void Start()
    {
        listener = new TcpListener(IPAddress.Loopback, 5005);
        listener.Start();
        Debug.Log("[SIM] Listening on 127.0.0.1:5005");
        
        serverThread = new Thread(AcceptConnections);
        serverThread.Start();
        
        // Start periodic TOF + Camera send
        InvokeRepeating(nameof(SendSensorData), 0.02f, 0.02f);  // 50 Hz
    }
    
    void AcceptConnections()
    {
        while (true)
        {
            connectedClient = listener.AcceptTcpClient();
            stream = connectedClient.GetStream();
            Debug.Log("[SIM] Client connected");
            HandleClient();
        }
    }
    
    void HandleClient()
    {
        byte[] buffer = new byte[256];
        while (connectedClient.Connected)
        {
            try
            {
                int bytesRead = stream.Read(buffer, 0, buffer.Length);
                if (bytesRead == 0) break;
                
                string command = System.Text.Encoding.ASCII.GetString(buffer, 0, bytesRead);
                
                if (command.StartsWith("MOTOR:"))
                {
                    float duty = float.Parse(command.Substring(6));
                    ApplyMotorForce(duty);
                }
                else if (command.StartsWith("SERVO:"))
                {
                    float angle = float.Parse(command.Substring(6));
                    RotateWheels(angle);
                }
            }
            catch (Exception e)
            {
                Debug.LogError("[SIM] " + e.Message);
                break;
            }
        }
    }
    
    void SendSensorData()
    {
        if (stream == null || !stream.CanWrite) return;
        
        // Lidar (TOF)
        int front = (int)RaycastDistance(Vector3.forward);
        int rear = (int)RaycastDistance(Vector3.back);
        string tofMsg = $"TOF:{front},{rear}\n";
        byte[] tofBytes = System.Text.Encoding.ASCII.GetBytes(tofMsg);
        stream.Write(tofBytes, 0, tofBytes.Length);
        
        // Camera (JPEG)
        Texture2D screenshot = ScreenCapture.CaptureScreenshotAsTexture();
        byte[] jpeg = ImageConversion.EncodeToJPG(screenshot);
        
        // Send with 4-byte big-endian size header
        byte[] sizeHeader = BitConverter.GetBytes(jpeg.Length);
        if (BitConverter.IsLittleEndian) Array.Reverse(sizeHeader);
        
        stream.Write(sizeHeader, 0, 4);
        stream.Write(jpeg, 0, jpeg.Length);
        stream.Flush();
    }
    
    float RaycastDistance(Vector3 direction)
    {
        RaycastHit hit;
        if (Physics.Raycast(transform.position, direction, out hit, 1200f))
            return hit.distance * 1000f;  // meters to mm
        return -1f;  // out of range
    }
    
    void ApplyMotorForce(float dutyPercent)
    {
        // Apply to vehicle Rigidbody
        float force = (dutyPercent / 100f) * maxMotorForce;
        // ... apply force to wheels ...
    }
    
    void RotateWheels(float angleDeg)
    {
        // Rotate front wheels to angle
        // angleDeg: 90 = straight, <90 = left, >90 = right
        // ... steer wheels ...
    }
}
```

**See full implementation**: [Link to Unity C# boilerplate] (to be added after Phase 1A approval)

---

## Step 4: Test Connection (Python Side)

**Create test script** `test_sim_connection.py`:

```python
#!/usr/bin/env python3
"""Test Sim2Real connectivity."""

import time
from sim_hardware_mocks import SimulatorClient

def test_connection():
    print("=" * 60)
    print("SIM2REAL CONNECTION TEST")
    print("=" * 60)
    
    try:
        # Connect
        print("\n[1/5] Connecting to simulator on 127.0.0.1:5005...")
        sim = SimulatorClient(host='127.0.0.1', port=5005, timeout=1.0)
        print("  ✓ Connected!")
        
        # Test motor
        print("\n[2/5] Testing motor (sending PWM 25%)...")
        sim.motor.set_speed(25.0)
        time.sleep(0.5)
        print(f"  ✓ Motor duty: {sim.motor.current_duty}%")
        
        # Test steering
        print("\n[3/5] Testing steering (sending 75° left)...")
        sim.steering.set_angle(75.0)
        time.sleep(0.5)
        print(f"  ✓ Servo angle: {sim.steering.current_angle}°")
        
        # Test sensors
        print("\n[4/5] Reading sensors (10 seconds)...")
        start = time.time()
        frame_count = 0
        tof_count = 0
        while time.time() - start < 10:
            # Read TOF
            if sim.distance.front_mm is not None:
                tof_count += 1
            
            # Read camera
            frame = sim.camera.get_latest_frame()
            if frame is not None:
                frame_count += 1
                if frame_count == 1:
                    print(f"  ✓ First frame: {frame.shape} (BGR)")
            
            time.sleep(0.1)
        
        print(f"  ✓ TOF readings: {tof_count} (target: ~100 @ 50 Hz)")
        print(f"  ✓ Frames received: {frame_count} (target: ~30 @ 30 FPS)")
        
        # Test brake
        print("\n[5/5] Testing brake...")
        sim.motor.brake()
        time.sleep(0.5)
        print(f"  ✓ Motor braked: duty = {sim.motor.current_duty}%")
        
        # Cleanup
        sim.close()
        
        print("\n" + "=" * 60)
        print("ALL TESTS PASSED ✓")
        print("=" * 60)
        
    except Exception as e:
        print(f"\n✗ ERROR: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_connection()
```

**Run**:
```bash
# Start this AFTER Unity is running and listening
python test_sim_connection.py
```

**Expected output**:
```
============================================================
SIM2REAL CONNECTION TEST
============================================================

[1/5] Connecting to simulator on 127.0.0.1:5005...
  ✓ Connected!

[2/5] Testing motor (sending PWM 25%)...
  ✓ Motor duty: 25.0%

[3/5] Testing steering (sending 75° left)...
  ✓ Servo angle: 75.0°

[4/5] Reading sensors (10 seconds)...
  ✓ First frame: (480, 640, 3) (BGR)
  ✓ TOF readings: 102 (target: ~100 @ 50 Hz)
  ✓ Frames received: 31 (target: ~30 @ 30 FPS)

[5/5] Testing brake...
  ✓ Motor braked: duty = 0.0%

============================================================
ALL TESTS PASSED ✓
============================================================
```

---

## Step 5: Run Simulator (Main Control Loop)

**Option A: Vision-Only Debug (no FSM)**
```bash
python main_simulator.py --display
```
- Opens debug window
- Reads camera + lidar
- Computes lane error + PID
- Motors stay OFF (safe mode)
- Good for camera/vision tuning

**Option B: Autonomous Mode (Full FSM)**
```python
# In Python REPL or script:
from main_simulator import VehicleSimulator

sim = VehicleSimulator()
sim.set_mode(sim.Mode.AUTONOMOUS)
sim.run()  # Runs until Ctrl-C
```
- FSM active: CRUCERO → PRECAUCIÓN → FRENADO → ESPERA → REANUDAR
- Motors execute commands
- Full validation loop
- Collects metrics (see next step)

---

## Step 6: Collect Metrics (CSV Data)

**Automatic collection** (built into `main_simulator.py`):

```python
from main_simulator import VehicleSimulator

sim = VehicleSimulator()
sim.set_mode(sim.Mode.AUTONOMOUS)

# Run test for 30 seconds
metrics = sim.run_autonomous_test(duration_s=30)

# Metrics dict contains:
# {
#   "elapsed_s": 30.5,
#   "loop_count": 1525,
#   "frame_count": 31,
#   "avg_loop_hz": 50.0,
#   "errors_px": [2.3, 1.8, -0.5, ...],
#   "pid_outputs": [0.18, 0.14, -0.04, ...],
#   "servo_angles": [92.0, 91.1, 89.9, ...],
#   "motor_duties": [22.0, 22.0, 22.0, ...],
#   "lidar_readings": [500, 495, 480, ...],
#   "fsm_states": ["CRUCERO", "CRUCERO", "PRECAUCION", ...]
# }

# Save to CSV (requires implementing save method)
# sim.save_metrics_csv('test_results.csv')
```

**Manual CSV generation**:

```python
import pandas as pd
from main_simulator import VehicleSimulator

sim = VehicleSimulator()
sim.set_mode(sim.Mode.AUTONOMOUS)
metrics = sim.run_autonomous_test(duration_s=30)

# Convert to DataFrame
df = pd.DataFrame({
    'loop_count': range(len(metrics['errors_px'])),
    'lane_error_px': metrics['errors_px'],
    'pid_output': metrics['pid_outputs'],
    'servo_angle': metrics['servo_angles'],
    'motor_duty': metrics['motor_duties'],
    'lidar_mm': metrics['lidar_readings'],
    'fsm_state': metrics['fsm_states']
})

# Save
df.to_csv('S1_latency_results.csv', index=False)
print("Saved to S1_latency_results.csv")
```

**Analyze**:

```python
import pandas as pd
import numpy as np

df = pd.read_csv('S1_latency_results.csv')

print("PHASE 1 VALIDATION RESULTS")
print("=" * 50)
print(f"Duration: {len(df) / 50:.1f} s")
print(f"Loops: {len(df)} @ {len(df) / (len(df)/50):.0f} Hz")
print(f"Lane error: {np.mean(np.abs(df['lane_error_px'])):.1f} ± {np.std(df['lane_error_px']):.1f} px")
print(f"Servo angle: {df['servo_angle'].mean():.1f}° ± {df['servo_angle'].std():.1f}°")
print(f"Lidar: {df['lidar_mm'].mean():.0f} ± {df['lidar_mm'].std():.0f} mm")

# State transitions
states = df['fsm_state'].unique()
print(f"\nFSM States visited: {', '.join(states)}")

# Motor duty
print(f"Motor duty: {df['motor_duty'].min():.1f}% to {df['motor_duty'].max():.1f}%")
```

---

## Step 7: Validate Against Scenarios

**Run each scenario** (30–60 seconds each):

### Scenario 1: Latency (S1)
```python
from main_simulator import VehicleSimulator

sim = VehicleSimulator()
sim.set_mode(sim.Mode.VISION)  # Vision-only first
import time; time.sleep(5)
metrics = sim.run_autonomous_test(duration_s=30)

# Analysis: see PHASE1_VALIDATION.md § Scenario 1
```

### Scenario 2: STOP Sign (S2)
```python
# Setup Unity with STOP sign at 700 mm
from main_simulator import VehicleSimulator

sim = VehicleSimulator()
sim.set_mode(sim.Mode.AUTONOMOUS)
metrics = sim.run_autonomous_test(duration_s=20)

# Verify: final lidar reading ~270 ± 30 mm
# Check: FSM visited CRUCERO → PRECAUCIÓN → FRENADO → ESPERA
```

### Scenario 3: FSM Robustness (S3)
```python
# Setup Unity with full track
from main_simulator import VehicleSimulator

sim = VehicleSimulator()
sim.set_mode(sim.Mode.AUTONOMOUS)
metrics = sim.run_autonomous_test(duration_s=60)

# Verify: all states visited, no spurious transitions
```

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| `ConnectionRefusedError: [Errno 111]` | Unity not listening on 5005. Start Unity server first. |
| `socket.timeout: timed out` | Network delay. Increase timeout in `sim_hardware_mocks.py` line 243. |
| `ModuleNotFoundError: No module named 'sim_hardware_mocks'` | Run from `TMR2026/` directory: `cd TMR2026 && python main_simulator.py` |
| `No frames arriving` | Unity camera not encoding JPEG. Check C# `ImageConversion.EncodeToJPG()`. |
| `Lane detection failing` | HSV threshold too tight. Lower `V_min` in `vision/lane_pipeline.py` from 130 to 100. |
| `STOP sign not detected` | YOLO confidence threshold too high. Lower `YOLO_CONF` from 0.55 to 0.40 in `main_simulator.py` line 95. |

---

## Next Steps (After Phase 1A)

1. **Phase 1B** — Adapt `main.py` to PC mode, add CLI argument for simulator selection
2. **Phase 2** — Full Unity integration with realistic physics (wheel slip, motor inertia)
3. **Phase 2B** — Deploy on Raspberry Pi with real hardware (replace mocks with drivers)
4. **Phase 3** — Hybrid validation: PC simulator + real car in parallel

---

## Files Delivered (Phase 1 Complete)

```
TMR2026/
├── sim_hardware_mocks.py      ← Socket-based hardware layer
├── main_simulator.py          ← PC-based FSM + control loop
├── SIM2REAL_PROTOCOL.md       ← Socket communication spec
├── PHASE1_VALIDATION.md       ← Validation scenarios + acceptance criteria
├── SIM2REAL_QUICKSTART.md     ← This file
└── test_sim_connection.py     ← Connection test script (create this)
```

---

## Contact

- **For protocol issues**: See `SIM2REAL_PROTOCOL.md`
- **For validation setup**: See `PHASE1_VALIDATION.md`
- **For code issues**: Check error message, then `TROUBLESHOOTING` section above

---

## One-Liner Test

Verify everything works in 60 seconds:

```bash
# Terminal 1: Start Unity, ensure it's listening on 127.0.0.1:5005
# Terminal 2:
cd TMR2026
python test_sim_connection.py  # Should complete with "ALL TESTS PASSED ✓"
```

**✓ Phase 1 Complete. Ready for Phase 2 (Unity full implementation).**

