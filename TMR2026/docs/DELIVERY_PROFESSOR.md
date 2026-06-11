# Delivery — Sim2Real Validation of the Autonomous Vehicle (TMR 2026)

> English version of `ENTREGA_PROFESOR.md`.

This document maps **every requirement of the assignment PDF** to what was delivered, explains
**how to run everything**, and shows where the **metrics and figures** for the scientific article are.

---

## 1. Executive summary

A **digital twin** was built in Unity 3D that communicates **bidirectionally** over TCP sockets with
the real TMR 2026 Python code. The same control code (FSM + PID + vision) that runs on the Raspberry
Pi runs on the PC against the simulation, **with no physical hardware**.

The **3 tests** from the PDF are executed and produce **.csv** files + figures + a **scoreboard**
that grades compliance.

**Latest run result: 100/100 — PASSED.**

---

## 2. Point-by-point PDF compliance

### Phase 1 — Python code adaptation (Hardware Mocks) ✅
| PDF requirement | Delivered | File |
|---|---|---|
| Motor/servo mock → sockets | `MockMotorDriver`, `MockSteeringDriver` send `MOTOR:`/`SERVO:` over TCP | `sim_hardware_mocks.py` |
| VL53L0X sensor mock (mm) | `MockDistanceSensor` listens to `TOF:front,rear` | `sim_hardware_mocks.py` |
| Camera mock (frames) | `MockCameraStream` receives JPEG; vision with YOLO + OpenCV | `sim_hardware_mocks.py`, `vision/` |

> The servo replicates the real physical inversion (`STEERING_INVERTED`) so the simulator behaves
> identically to the car.

### Phase 2 — Virtual environment (Unity 3D) ✅
| PDF requirement | Delivered | Where |
|---|---|---|
| Simplified TMR track (lane, STOP, parking) | Track with white lane lines, STOP sign and a marked parking zone | `Assets/Scripts/SceneBuilder.cs` |
| Car model with physics + Ackermann | Vehicle with Ackermann steering (front wheels turn), stable motion | `Assets/Scripts/VehicleBuilder.cs`, `VehicleController.cs` |
| ToF via front/rear raycast at 50 Hz | `SensorManager.GetToFFront/Rear` (raycast → mm), sent at 50 Hz | `Assets/Scripts/SensorManager.cs`, `SimulatorServer.cs` |
| Virtual camera → RenderTexture → JPEG bytes | `SensorManager.GetCameraJPEG()` at ~30 FPS | `Assets/Scripts/SensorManager.cs` |
| Command reception (speed, angle) → motor/steering | `SimulatorServer` parses `MOTOR`/`SERVO` → `VehicleController` | `Assets/Scripts/SimulatorServer.cs` |

### Phase 3 — Tests and data (Deliverables) ✅
| PDF test | Delivered | CSV file |
|---|---|---|
| **P1 Latency** of the perception→response loop (<100–200 ms) | per-cycle latency logged every iteration | `validation_results/P1_latencia.csv` |
| **P2 PID braking** at STOP (700→270 mm, no overshoot) | ToF distance vs PWM vs time | `validation_results/P2_pid_stop.csv` |
| **P3 FSM transitions** (no blocking, clean transitions) | time log of every state change + dwell | `validation_results/P3_fsm.csv` |

> **Perpendicular (battery) parking — full P3:** in the **same run**, after completing the STOP
> cycle the vehicle resumes, drives forward and executes the parking maneuver with its states
> `PARKING_SEARCH → PARKING_MANEUVER → PARKED` (`control/parking_fsm.py`). The gap is found with the
> **front ToF distance sensor** (not vision); the entry maneuver (turn + straighten) is **open-loop,
> time-based**, like a programmed parking. Everything lands in the same
> `validation_results/P3_fsm.csv`. Run with `python run_validation.py`.

---

## 3. How to run EVERYTHING (step by step)

### One-time requirements
```bash
pip install -r requirements.txt        # opencv, numpy, matplotlib, ultralytics
```

### Execution
1. **Open Unity** (project `TMR2026_Sim`) and press **PLAY**.
   - Check the console: `[Server] Listening on port 5005...`
2. **On the PC**, a terminal in `Carrito/TMR2026/`:
   ```bash
   python run_validation.py     # EVERYTHING in a single run
   ```
   A single run executes the full sequence against the simulation: drive → detect STOP → brake →
   wait 5 s → resume → drive forward → park in battery. It covers the **3 PDF tests** (P1 latency,
   P2 PID braking at STOP, P3 FSM transitions including the STOP cycle and the parking) and produces
   the CSV files, the scoreboard (`SCOREBOARD.txt`) and the figures (PNG), all in English.

### Live view only (no data saved)
```bash
python main_simulator.py --display     # debug window with camera + BEV
```

---

## 4. What to hand in to the professor

Run `python armar_entrega.py` to build **`DELIVERY_TMR2026.zip`** (English only) in your
`Documents` folder:
```
DELIVERY_TMR2026.zip
├── 01_results_3_tests/
│   ├── P1_latency.csv · P2_pid_stop.csv · P3_fsm.csv      ← data
│   ├── fig1_latency.png · fig2_braking.png · fig3_fsm.png ← figures
│   └── SCOREBOARD.txt                                     ← scoreboard (100/100)
├── 02_documents/
│   ├── DELIVERY_PROFESSOR.md   ← this document
│   └── CALIBRATION_SIM.md      ← simulator calibration
└── README.txt
```
Plus the **source code** (GitHub repos `Carrito` and `TMR2026_Sim`).

---

## 5. Scoring system (competition style)

The script grades each test against its PDF criterion and assigns points:

| Test | Criterion | Points |
|---|---|---|
| P1 Latency | mean < 100 ms = 30; < 200 ms = 20 | /30 |
| P2 STOP braking | stop at 270 ± 30 mm without overshoot = 40 | /40 |
| P3 FSM | complete the STOP cycle (5 states) or the parking cycle (3 states) without blocking = 30 | /30 |
| **TOTAL** | | **/100** |

Verdict: ≥70% PASSED, 40–70% PARTIAL, <40% REVIEW.

---

## 6. Scope notes (technical honesty)

- **Latency**: in local simulation the loop runs at ~9–15 ms (well below the 200 ms target). In the
  article this is reported as evidence that the multi-threaded design introduces no bottleneck.
- **PID braking**: the car stops within 270 ± 30 mm of the STOP sign (measured ~292 mm), with no
  overshoot. The braking distance is taken as the median of the readings while the sign is visible.
- **Battery parking**: integrated and working in the same run (`control/parking_fsm.py`). After the
  STOP cycle the vehicle resumes, drives forward and runs `PARKING_SEARCH → PARKING_MANEUVER →
  PARKED`, logged in `P3_fsm.csv`. **Honest scope:** the *gap search* uses the **front ToF distance
  sensor** (not object detection by vision/YOLO — the YOLO model is trained only for traffic signs),
  and the *entry maneuver* (turn right + straighten) is **open-loop, time-based**, like a programmed
  parking. Detecting the slot with the camera (cars by color/HSV) would be the next improvement.

---

## 7. Architecture (for the article)

```
   PC (Python, real TMR2026 code)              Unity 3D (digital twin)
   ┌─────────────────────────────┐  TCP 5005   ┌──────────────────────────┐
   │ Vision (OpenCV / YOLO)      │ ─MOTOR/SERVO─►│ VehicleController        │
   │ FSM (5 states)              │             │ (Ackermann + physics)    │
   │ PID steering                │ ◄─TOF/JPEG── │ SensorManager (raycast + │
   │ Hardware mocks (sockets)    │             │ RenderTexture)           │
   └─────────────────────────────┘             └──────────────────────────┘
        3 threads: control 50 Hz · vision · receiver
```

**Data flow:** Unity renders the camera and casts the ToF rays → sends JPEG (30 FPS) + `TOF` (50 Hz)
→ Python runs vision + FSM + PID → sends `MOTOR`/`SERVO` → Unity moves the car. Closed loop, exactly
like the physical vehicle but with no risk to the hardware.
