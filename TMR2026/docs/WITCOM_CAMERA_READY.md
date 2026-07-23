# WITCOM 2026 / Springer CCIS — Camera-Ready Plan (Paper 2069)

Single source of truth for the revision. **Camera-ready due 2026-07-31**;
registration 2026-08-15. Rule from the chairs: *address every reviewer comment
without changing the accepted scientific contribution* → moderate the claims +
add rigor, don't rewrite the science.

---

## 0. What is already DONE (generated in the repo)

| Item | Where | Status |
|---|---|---|
| Latency **distribution** (mean/median/std/p95/p99/jitter/deadline-miss) from the 2 423-cycle log | `tools/latency_stats.py` → `validation_results/P1_latencia_distribution.png` + `_stats.txt` | ✅ |
| Fixed the **20 ms vs 200 ms** bug (Rev1 #5) | `analyze_results.py` (deadline line 20 ms) | ✅ |
| Figures now export at **300 dpi** (Rev requirement) | `analyze_results.py` | ✅ |
| Braking **steady-state error + tolerance** (Rev all) | computed: 292.5 mm, err 22.5 mm/8.3%, **within ±30 mm**, no overshoot | ✅ |
| **On-device latency benchmark** (real Pi 5 + IMX500) | `tools/bench_latency.py` | ✅ ready to run on Pi |
| **Physical braking experiment** (10 trials, sim-vs-real) | `tools/bench_braking_physical.py` | ✅ ready to run on car |

---

## 1. Reviewer → action map (the checklist)

**🔴 Must-fix (all 3 reviewers):**
- [ ] **Reframe "Sim2Real" → "Software-in-the-Loop (SIL)"** in title/abstract/conclusion *(chat)*
- [x] **Latency distribution** not just mean → p95/p99/jitter/deadline-miss *(done: table below)*
- [ ] **Define latency formally** + say what was measured *(text in `bench_latency.py` docstring; paste into paper)* *(chat)*
- [ ] **Measure REAL latency on the Pi** → `bench_latency.py` *(Pi, later today)*
- [ ] **Fix Table 2** (it duplicates Table 1) → use the test matrix in §4 *(chat)*
- [x] **20 ms vs 200 ms** inconsistency *(fixed in figure; also fix in text — chat)*
- [x] **Braking error + tolerance band** (22.5 mm within ±30 mm, no overshoot) *(text — chat)*
- [ ] Moderate language: remove "flawless", "confirms viability" *(chat)*

**🟠 Strongly expected:**
- [ ] Baseline/ablation: **CPU vs NPU**, multithread vs single-thread *(some measurable on Pi)*
- [x] **Perception metrics** (§3) — already have F1 99% *(text — chat)*
- [ ] **PID equations** + gains + sampling period (all in `config.py`) *(chat)*
- [ ] "Digital twin" → "virtual prototype / SIL environment" *(chat)*
- [ ] FSM state names must match Figure 3 *(chat)*
- [ ] **Translate the Spanish paragraph in the conclusion** (Rev2) *(chat)*
- [ ] Repo **tagged release + Zenodo DOI** *(Code)*
- [ ] Add **Limitations** subsection + more recent literature *(chat)*
- [ ] Fig 2 = real hardware **photo** (not a screenshot); Figs 5–7 higher-res *(you + chat)*

---

## 2. Real results generated tonight (drop straight into the paper)

### Control-loop latency — SIL (Unity), n = 2 423 cycles
| mean | median | std (jitter) | p95 | p99 | max | < 20 ms deadline |
|---|---|---|---|---|---|---|
| 9.23 ms | 9.18 ms | 1.29 ms | 11.34 ms | 13.64 ms | 20.83 ms | **99.96 %** (1 miss) |

> Honest framing: this is the **SIL control-loop** distribution. The physical
> Pi 5 + IMX500 number comes from `bench_latency.py` and will be reported
> separately. Figure: `P1_latencia_distribution.png` (histogram + CDF, 300 dpi).

### Braking (P2) — SIL
Setpoint **270 mm**, tolerance **±30 mm** (`STOP_TOLERANCE_MM`, defined before testing).
Result: stops at **292.5 mm** → steady-state error **22.5 mm (8.3 %)**, **no overshoot**
(min distance = 292.5 mm), **within tolerance**. Physical distribution from the 10-trial run.

---

## 3. Perception metrics (already measured — add as a table)

Detector `tmr_signs` (YOLOv8n, 7 classes: green, left, red, right, stop, straight, yellow):
| Split | mAP@50 | mAP@50-95 | Precision | Recall | F1 |
|---|---|---|---|---|---|
| Validation | 0.995 | 0.647 | — | 1.00 (all classes) | — |
| Held-out test @conf 0.55 | — | — | 99.3 % | 98.6 % | **99.0 %** |

Lane detector: report **lane-center error (px)** distribution from the logs
(`error_px` column) and, on the physical track, under normal / shadow / glare light.

---

## 4. Test matrix — replaces the broken Table 2

| ID | Purpose | Independent var | Initial condition | Reps | Metric | Acceptance | Result |
|---|---|---|---|---|---|---|---|
| **P1** | Control-loop latency | — | 50 Hz loop, ~30 s | 2 423 cycles (SIL) + Pi run | mean, p95, p99, deadline-miss | < 20 ms | 9.23 ms mean, p99 13.64, **99.96 % < 20 ms** |
| **P2** | PID braking at STOP | approach speed | car cruising, STOP at end | 1 (SIL) + **10 (physical)** | stop distance, SS error, overshoot | 270 ± 30 mm, no overshoot | 292.5 mm, err 22.5 mm, no overshoot (SIL) |
| **P3** | FSM transitions | — | full run (drive→stop→resume→park) | 1 (SIL) + physical | states visited, non-blocking wait | STOP 5/5 + parking 3/3 | 5/5 + 3/3, `ESPERA` non-blocking |

Fill the "physical" columns after tomorrow's runs.

---

## 5. Physical test plan (later today, on the Pi + car)

**Order matters — calibrate first, then measure.**

**Step 0 — Calibrate vision for YOUR home track & light** (no motors — THIS is
what makes it drive like Unity):
```bash
python tools/tune_track.py                 # live sliders: HSV + roi + right_bias + PID
```
Drag until the green lane line is stable and centered, press **s** to save
`track_calib.json`. From then on `main.py` loads it automatically. Quick check
without tuning: `python tools/test_camera.py --no-yolo`.

**Step 1 — Real latency (no movement, biggest win):**
```bash
python tools/bench_latency.py --cycles 3000 --hz 50
python tools/latency_stats.py validation_results/bench_latency_pi.csv --deadline 20 --label "Pi 5 + IMX500 (on-device)"
```

**Step 2 — Physical braking, 10 trials (car MOVES — wheels-off-ground test FIRST):**
```bash
python tools/bench_braking_physical.py --trials 10 --cruise 25
```

**Step 3 (optional) — Lane following & parking attempts:**
```bash
python main.py --display     # count clean laps; try the parking button (Y)
```
> Reality check: physical lane-following isn't robust yet. **P1 latency + P2 braking
> are the high-value, high-probability results** — they alone turn "0 physical
> experiments" into real Sim2Real evidence. Lane/parking are bonus.

---

## 6. Track to build at home — measurements & shopping list

Your car: length 35 cm, width 20 cm, min turn radius ≈ **0.41 m**. Build modular;
you only NEED Module A for the critical experiments.

### Module A — Straight lane + STOP sign  *(mandatory: P1, P2, emergency stop)*
```
   |<------------------ 1.5 – 2.0 m ------------------>|
   ||=================================================||   <- white line (tape) ~3 cm wide
   ||                                                 ||
   ||        [ car 20 cm ]  --->                 [STOP]||   <- STOP sign at the end
   ||                                                 ||
   ||=================================================||   <- white line
   |<--------------- ~50 cm between lines ------------>|
```
- **Length:** 1.5–2.0 m (car reaches cruise, detects STOP ≤ 700 mm, brakes to 270 mm).
- **Lane width (inner edge to inner edge):** **~50 cm** (matches `LANE_WIDTH_M ≈ 0.54`).
- **Line width:** **2–4 cm** white tape, both sides.
- **STOP sign:** printed red octagon **~5–8 cm**, on a stick **~15–20 cm** tall, at the end.

### Module B — Curve  *(optional: lane following)*
- One 90° arc, **centerline radius ≥ 0.7 m** (well above the 0.41 m minimum).

### Module C — Parking bay  *(optional: parking)*
- Two car-sized boxes (~35×20 cm) as "parked cars", perpendicular, with a
  **60 cm gap** between them (`PARK_TARGET_GAP_MM = 600`).

### Surface & light (important for the vision)
- **Dark, MATTE surface** with **white** lines. Glossy black causes reflections
  that leak into the white mask — the HSV default is tuned for **medium-low, even light**.
- Even lighting (a desk lamp); avoid direct glare/flash on the surface.

### 🛒 Shopping list (todo barato, ~$150–400 MXN)
| Item | For | Note |
|---|---|---|
| Black plastic tablecloth (*mantel negro*) **or** black foam board ×2–3 | track surface | matte, not glossy; or use a dark floor |
| White gaffer tape **or** white electrical tape ×2 rolls (2–4 cm) | lane lines | matte white reads best |
| Cardboard + printed STOP sign + wood dowel/popsicle sticks + glue | STOP sign | print the sign at home |
| 2 cardboard boxes (car-sized) | parking "cars" | free, from home |
| Clip-on LED lamp *(optional)* | even lighting | helps the HSV mask |
| Tape measure, scissors, cutter | build | you probably have these |

Optional: print **red/green/yellow traffic-light cards** to also test those classes.

---

## 7. Which tool for what (from here)

| **Claude Code** (here, the repo/Pi/car) | **Claude chat** (claude.ai, the paper) |
|---|---|
| Run `bench_latency.py`, `bench_braking_physical.py` on the Pi | Reframe Sim2Real → SIL, title, abstract |
| `latency_stats.py`, regenerate 300-dpi figures | Rewrite Table 2 from §4, fix 20/200 ms text |
| Tag release + Zenodo DOI | PID equations, limitations, literature |
| Any code / `config.py` edits | **Response-to-reviewers letter**, Springer format |

**Next actions:** (1) later today run Steps 1–2 on the Pi/car → real numbers;
(2) in the chat, upload the manuscript and start §1's *chat* items in parallel.

---

## 8. Reproducibility & DOI (reviewers 2 & 3)

**Order matters** — Zenodo only mints a DOI for releases created *after* you
connect it. Do the FINAL release once the camera-ready code is frozen.

1. **Add `CITATION.cff`** at the repo root (fill surname + license):
```yaml
cff-version: 1.2.0
message: "If you use this software, please cite it."
title: "Sim2Real-Scale-Vehicle: Monolithic Edge AI Autonomous Navigation (TMR 2026)"
authors:
  - given-names: Angel Emmanuel
    family-names: "<your surname>"
    email: carreraangel55555@gmail.com
repository-code: "https://github.com/Robotics-TESE/Sim2Real-Scale-Vehicle"
license: "<choose, e.g. MIT>"
version: v1.0.0
date-released: 2026-07-31
```
2. **Connect Zenodo:** log in at zenodo.org with GitHub → *GitHub* settings →
   toggle **ON** the `Robotics-TESE/Sim2Real-Scale-Vehicle` repo.
3. **Create the release** (when the paper code is final) — I can run this for you:
```bash
git tag -a v1.0.0 -m "WITCOM 2026 camera-ready"
git push org v1.0.0
gh release create v1.0.0 --repo Robotics-TESE/Sim2Real-Scale-Vehicle \
  --title "WITCOM 2026 camera-ready" --notes "Camera-ready snapshot (Paper 2069)."
```
4. Zenodo auto-mints the DOI; put the DOI badge in the README and cite it in the
   paper's reproducibility statement.

> Not created yet on purpose — the "permanent release" should be the final
> version. Say the word when the code is frozen and I run step 3.

**Also report for reproducibility:** Python version, Ultralytics/OpenCV versions,
Unity version (6000.4), and the calibration file `track_calib.json`. A dependency
freeze: `pip freeze > TMR2026/requirements-lock.txt`.
