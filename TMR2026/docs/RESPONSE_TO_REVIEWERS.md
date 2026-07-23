# Response to Reviewers — WITCOM 2026 / Springer CCIS (Paper 2069)

Draft rebuttal. Refine in the chat with the manuscript. Tone: acknowledge the
valid points, describe the concrete change, cite the data. Line up each response
with the actual edits and the artifacts in the repository.

---

## Summary of major changes

1. **Reframed the central claim** from "Sim2Real validation" to **software-in-the-loop
   (SIL) validation with on-device latency measurement**, throughout title,
   abstract, methodology and conclusions.
2. **Added an on-device latency measurement** on the physical Raspberry Pi 5 +
   IMX500, with a **formal latency definition** and full **distribution**
   (mean, median, std/jitter, p95, p99, max, deadline-miss), replacing the
   single simulated mean.
3. **Added physical braking trials** (N = 10) reporting mean ± std stopping
   distance and a simulation-vs-physical comparison.
4. **Replaced Table 2** with a proper test matrix (purpose, conditions,
   repetitions, metrics, acceptance criteria, results).
5. **Corrected the 20 ms / 200 ms inconsistency** (the 50 Hz loop deadline is
   20 ms) in text and figures.
6. **Reported the braking steady-state error against a pre-defined ±30 mm
   tolerance band** instead of describing it as "ideal".
7. **Added perception metrics** (detector precision/recall/F1) and a **CPU-vs-NPU**
   comparison.
8. **Added PID equations, gains, sampling period and anti-windup**, a
   **Limitations** subsection, and expanded related work; moderated all claims.
9. Figures re-exported at **≥300 dpi**; Fig. 2 replaced with a hardware photo;
   the untranslated Spanish paragraph corrected; a tagged repository release with
   a DOI added.

---

## Reviewer 1 (score 0)

**1. The paper does not demonstrate Sim2Real transfer.**
Acknowledged. We have reframed the contribution as **SIL validation** (the
control code is identical in simulation and on the device) plus an **on-device
latency measurement** and **N = 10 physical braking trials** with a
simulation-vs-physical comparison. All claims of full trajectory transfer were
removed; remaining transfer is stated as ongoing work in the new Limitations
subsection.

**2. The latency experiment is conceptually ambiguous.**
We added a **formal definition** (L = t_read + t_lane + t_sign + t_control; the
20 ms deadline of the 50 Hz loop) and clarified the instrumentation
(`time.perf_counter`, ≥2000 cycles, warm-up discarded). We now report **two
clearly labelled measurements**: the SIL control-loop latency and the
**on-device** latency on the Pi 5 + IMX500. We state explicitly that on the NPU
path inference runs on the sensor in parallel and is not part of L.

**3. No comparison with a distributed architecture.**
We added an **ablation**: on-device **NPU vs CPU** inference and
**multithreaded vs single-threaded** execution, reporting latency, jitter and
deadline-miss. We tempered the introduction's claims about distributed systems
to what the ablation supports.

**4. Insufficient statistical validation.**
The evaluation now reports the **full latency distribution** (mean, median, std
as jitter, p95, p99, max, deadline-miss rate over 2 423 SIL cycles and the
on-device run) and, for braking, **mean ± std over 10 trials** with a
within-tolerance rate. We added CPU/thermal readings from the Pi.

**5. 20 ms vs 200 ms inconsistency.**
Corrected. The control cycle deadline is **20 ms** (50 Hz); the figure and text
were updated accordingly.

**6. The stopping experiment does not demonstrate convergence.**
We now report the residual **steady-state error (22.5 mm, 8.3 %)** against the
**±30 mm tolerance band defined a priori**, and no longer describe it as "ideal
asymptotic behaviour". We add deceleration/settling time and the 10-trial
repeatability.

**Table 2 duplicates Table 1.** Corrected — Table 2 is now a test matrix with
purpose, variables, initial conditions, repetitions, metrics, thresholds and
results per scenario.

**Comment list (monolithic, deterministic, RT scheduling, thresholds, servo,
Ackermann, PID, etc.):** addressed — we define "monolithic" precisely (single
process, on-device, on-sensor inference), avoid "deterministic" without WCET,
state the Pi 5 clock/RAM/cooling/power mode, explain stale-frame handling, give
the 120 mm and 700 mm threshold rationale, specify the servo command as a
calibrated steering angle, list the Ackermann parameters, and add the PID
equations, gains and sampling period.

**Five questions.** (i) The 9.2 ms was the **SIL** control-loop latency; the
physical IMX500 number is now reported separately. (ii) **2 423 SIL cycles**;
p95 = 11.34, p99 = 13.64, std = 1.29 ms, deadline-miss = 0.04 % — plus the
on-device run. (iii) We add the calibration evidence available and state
unmodeled dynamics as a limitation. (iv) The ±30 mm band was defined before
testing; 22.5 mm is within it. (v) The physical braking trials + the
simulation-vs-physical comparison are the transfer evidence added.

---

## Reviewer 2 (score 1)

**1. State contribution/scope accurately.** Done — the contribution is framed as
the **integration** (on-device perception + threaded control + FSM + PID +
SIL workflow), not novel individual components.

**2. Real-world validation missing.** Added on-device latency + physical braking
trials; the title/claims now say SIL + on-device.

**3. Baseline comparison.** Added the CPU-vs-NPU and multithread-vs-single ablation
(latency, jitter, deadline-miss, CPU).

**4–5. Protocol & statistics.** The test matrix specifies trials, conditions,
success/failure criteria; results now include the full distribution.

**6. Table 2.** Corrected (see above).

**7. "Digital twin".** Renamed to **virtual prototype / SIL environment**.

**8. Perception evaluation.** Added: detector P/R/F1 (0.993/0.986/0.990 @0.55,
mAP@50 0.995) and, on track, lane-center error under varied lighting.

**9. PID interpretation moderated;** 22 mm reported against the ±30 mm band.

**10. FSM naming.** Harmonized between the text and Fig. 3.

**Minor comments.** Spanish paragraph translated; "flawless" replaced; 700 mm and
120 mm thresholds justified; PID equations and limits added; thread
synchronization described (locks/queues, non-blocking latest-value reads);
camera/sensor-failure handling stated; a hardware cost and power table added; a
clear hardware photo added; Fig. 3 readability improved; Unity/Python versions
reported; a tagged release + DOI provided; a Limitations subsection and expanded
related work added.

---

## Reviewer 3 (score 1)

**1. Reframe Sim2Real → SIL.** Done (software-in-the-loop).

**2. Limited physical validation.** Added on-device latency + 10 braking trials +
emergency-stop trials, and a simulation-vs-physical stopping-distance comparison.

**3. Complete experimental table.** Added (test matrix).

**4. Quantitative perception results.** Added (detector metrics; lane error on track).

**5. Real-time performance.** Now a histogram + CDF with p95/p99/jitter, not a
single time series.

**6. Baseline/ablation.** Added (multithread on/off, CPU vs NPU).

**7. PID evaluation.** The ±25 mm/±30 mm band is stated and the 22 mm error
reported against it, with repeated trials.

**8. Reproducibility.** Tagged release + DOI; config, simulation parameters,
logs and plotting scripts (`analyze_results.py`, `latency_stats.py`,
`bench_latency.py`) accompany the code.

**9. Literature.** Expanded (embedded vision sensors, Edge-AI robotics, HIL/SIL,
Sim2Real) with a short comparison table.

**10. Moderated claims.** "flawless execution" / "confirms the viability" replaced
with statements limited to the measured tests.

**Questions.** Physical transfer evidence = the braking trials + comparison; the
latency measurement scope is defined formally; the multithreading advantage is
quantified by the ablation; FSM robustness to missed/repeated/noisy detections
is handled by the 3-frame hysteresis and the stop/red-only gating (described);
the simulation parameters needed to reproduce the virtual dynamics are listed in
the reproducibility appendix.
