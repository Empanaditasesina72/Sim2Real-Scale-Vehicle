# Camera-ready snippets — ready to paste (do this in the chat, with the manuscript)

Latency (P1) has its own file: `paper_latency_snippet.md`. This file covers the
rest: terminology reframe, braking (P2), the test matrix (Table 2 replacement),
perception metrics, and the PID equations.

---

## 1. Terminology reframe (title / abstract / conclusion)

- **Title / abstract / everywhere:** "Sim2Real validation" → **"software-in-the-loop
  (SIL) validation"** or **"simulation-based pre-validation toward Sim2Real
  deployment"**. Suggested title tweak: *"…: Monolithic Edge AI Architecture with
  Software-in-the-Loop Validation and On-Device Latency Measurement"*.
- **"Digital twin"** → **"virtual prototype / SIL environment"** (no physical
  calibration is demonstrated, so "twin" is too strong).
- Remove overclaims: **"flawless execution"**, **"confirms the viability"** →
  replace with the actual, bounded statements ("within the ±30 mm tolerance",
  "99.96 % of cycles met the 20 ms deadline in SIL").
- Add one sentence in the intro/limitations: the reported evaluation is SIL
  (Unity control code identical to the on-device code) plus an on-device latency
  measurement; full physical trajectory transfer is ongoing work.

---

## 2. Braking (P2) — paragraph + numbers

Setpoint **270 mm**, tolerance **±30 mm** (`STOP_TOLERANCE_MM`, defined before
testing). Median stop **292.5 mm**, steady-state error **22.5 mm (8.3 %)**, **no
overshoot** (minimum distance = final distance = 292.5 mm).

> The braking controller was evaluated against a 270 mm setpoint with a ±30 mm
> acceptance band defined a priori. In simulation the vehicle came to rest at a
> median distance of 292.5 mm, a steady-state error of 22.5 mm (8.3 %) that lies
> within the tolerance band, with no overshoot (the minimum approach distance
> equals the final distance). On the physical vehicle, N = 10 trials gave a mean
> stopping distance of **[mean] ± [std] mm** with **[k]/10** trials inside the
> ±30 mm band. (Do NOT call this "ideal convergence"; report the residual error
> and the tolerance band, as above.)

Report also (from the CSV): rise/deceleration time, settling time, and, across
the 10 physical trials, the repeatability (mean ± std).

---

## 3. Test matrix — REPLACES the broken Table 2

Table 2 currently duplicates Table 1. Replace it with:

| ID | Purpose | Independent var | Initial condition | Reps | Metric | Acceptance | Result |
|---|---|---|---|---|---|---|---|
| **P1** | Control-loop latency | — | 50 Hz loop, ~30 s | 2 423 cycles (SIL) + Pi | mean, p95, p99, deadline-miss | < 20 ms | 9.23 ms mean, p99 13.64, 99.96 % < 20 ms (SIL) |
| **P2** | PID braking at STOP | approach speed | cruising, STOP at end | 1 (SIL) + 10 (phys) | stop distance, SS error, overshoot | 270 ± 30 mm, no overshoot | 292.5 mm, err 22.5 mm, no overshoot (SIL) |
| **P3** | FSM transitions | — | full run (drive→stop→resume→park) | 1 (SIL) + phys | states visited, non-blocking wait | STOP 5/5 + parking 3/3 | 5/5 + 3/3, `ESPERA` non-blocking |

LaTeX skeleton:
```latex
\begin{table}[t]\centering
\caption{Evaluation test matrix.}\label{tab:matrix}
\begin{tabular}{@{}llp{2.2cm}cll@{}}\toprule
ID & Purpose & Initial condition & Reps & Acceptance & Result\\\midrule
P1 & Control-loop latency & 50\,Hz, $\sim$30\,s & 2423 & $<20$\,ms & 9.23\,ms mean, p99 13.64, 99.96\% $<20$\,ms\\
P2 & PID braking at STOP & cruise, STOP end & 1+10 & $270\pm30$\,mm, no overshoot & 292.5\,mm, err 22.5\,mm, no overshoot\\
P3 & FSM transitions & full run & 1+phys & 5/5 + 3/3 & 5/5 + 3/3, non-blocking\\
\bottomrule\end{tabular}\end{table}
```

---

## 4. Perception metrics (add a table — reviewers 2 & 3 asked for this)

Sign detector `tmr_signs` (YOLOv8n, 7 classes: green, left, red, right, stop,
straight, yellow), trained on the traffic_lights dataset, evaluated with
`tools/eval_yolo.py`:

| Split | mAP@50 | mAP@50-95 | Precision | Recall | F1 |
|---|---|---|---|---|---|
| Validation | 0.995 | 0.647 | — | 1.00 (all classes) | — |
| Held-out test @ conf 0.55 | — | — | 0.993 | 0.986 | **0.990** |

> The traffic-sign detector reaches mAP@50 = 0.995 on validation and, on a
> held-out test set at the deployed confidence threshold (0.55), a precision of
> 0.993, recall of 0.986 and F1 of 0.990. Because only the *stop* and *red*
> classes gate braking, the safety-critical false-trigger rate is bounded by the
> per-class precision of those two classes. (On the physical track, also report
> the lane-center error distribution and detection behaviour under shadow/glare.)

---

## 5. PID — equations, gains, period, tuning (reviewers all asked)

Steering PID (lane error in pixels → servo correction in degrees), sampled at
**50 Hz (T_s = 20 ms)**, gains **Kp = 0.08, Ki = 0.002, Kd = 0.025**, output
saturation **±32°** (servo authority 58°–122° about 90°), integral clamp **±25**
(anti-windup), **derivative on the measurement** (not the error) to avoid
derivative kick.

Continuous form (derivative-on-measurement):
```latex
u(t) = K_p\,e(t) + K_i\!\int_0^t e(\tau)\,d\tau - K_d\,\frac{dy(t)}{dt},
\qquad e(t)=r-y(t),\; r=0
```
Discrete form actually implemented (T_s = 20 ms, anti-windup clamp on I):
```latex
\begin{aligned}
I_k &= \mathrm{clamp}\!\big(I_{k-1} + e_k T_s,\; -25,\; 25\big)\\
D_k &= -K_d\,(y_k - y_{k-1})/T_s\\
u_k &= \mathrm{clamp}\!\big(K_p e_k + K_i I_k + D_k,\; -32,\; +32\big)\ [^{\circ}]
\end{aligned}
```
Servo command: `angle = 90° + u_k`.

> Gain selection: the gains were tuned manually by increasing K_p until the
> steering began to oscillate and then halving it, followed by small K_d for
> damping and a small K_i to remove steady-state bias; the integrator is clamped
> to ±25 for anti-windup and the derivative is computed on the measurement to
> suppress noise-induced kick.

The same gains and 20 ms period are used by the braking/speed logic; state them
once and reference.
