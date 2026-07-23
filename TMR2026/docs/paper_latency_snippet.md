# Latency (Test P1) — ready-to-paste text for the camera-ready

Copy these into the paper (in the chat, with the manuscript). They answer the
reviewers' three latency demands: **a formal definition**, **what exactly was
measured**, and **the distribution** (not just the mean).

---

## 1. Formal latency definition (put in Methodology)

> We define the per-cycle control latency **L** as the wall-clock time of one
> control iteration:
>
> **L = t_read + t_lane + t_sign + t_control**, where
> *t_read* reads the latest camera frame, *t_lane* is the lane pipeline
> (bird's-eye transform + HSV white segmentation + sliding windows),
> *t_sign* is the sign-gating decision (retrieving detections and evaluating
> the stop/red condition), and *t_control* is the PID update plus servo-command
> formation. On the IMX500 path, YOLO inference runs **on the sensor in
> parallel**, so it is not part of L; L is the CPU work the Pi 5 performs each
> 50 Hz (20 ms) control cycle. A cycle **misses its deadline** when L > 20 ms.

## 2. Instrumentation (put in Experimental setup)

> Latency is measured per control cycle with a monotonic high-resolution timer
> (`time.perf_counter`) over N ≥ 2 000 consecutive cycles at the 50 Hz loop
> rate; the first (warm-up) cycle is discarded. We report the full distribution
> (mean, median, standard deviation used as jitter, p95, p99, maximum) and the
> deadline-miss rate against the 20 ms budget, rather than the mean alone.

## 3. Results table (Table — replace the single mean)

| Configuration | n | mean | median | std (jitter) | p95 | p99 | max | < 20 ms |
|---|---|---|---|---|---|---|---|---|
| **SIL** (Unity, control-loop) | 2 423 | 9.23 | 9.18 | 1.29 | 11.34 | 13.64 | 20.83 | **99.96 %** |
| **Pi 5 + IMX500** (on-device) | ____ | ____ | ____ | ____ | ____ | ____ | ____ | ____ % |
| *reference: lane+PID compute, x86* | 1 499 | 3.78 | 3.62 | 0.94 | 5.08 | 5.73 | (24.9)\* | 99.93 % |

All values in ms. \*single OS-scheduling outlier. Fill the **Pi 5 + IMX500** row
after running `bench_latency.py` on the car (see §5).

## 4. Results paragraph (fill the on-device numbers)

> Over 2 423 control cycles in the software-in-the-loop setup, the control-loop
> latency had a mean of 9.23 ms (median 9.18 ms) with 1.29 ms jitter; the 95th
> and 99th percentiles were 11.34 ms and 13.64 ms, and 99.96 % of cycles met the
> 20 ms deadline (a single cycle at 20.83 ms). On the physical Raspberry Pi 5
> with on-sensor IMX500 inference, the on-device latency was **[mean] ms**
> (median **[…]**, p99 **[…]**), with **[…] %** of cycles under the 20 ms
> deadline, confirming that the monolithic on-sensor architecture sustains the
> 50 Hz loop on the target hardware.

## 5. How to get the on-device row (on the Pi)

```bash
python tools/bench_latency.py --cycles 3000
python tools/latency_stats.py validation_results/bench_latency_pi.csv --deadline 20 --label "Pi 5 + IMX500 (on-device)"
```
Then send me `validation_results/bench_latency_pi.csv` and I fill the table + the
paragraph and regenerate the 300-dpi distribution figure.

## 6. Honesty note (this is what protects you from Reviewer 1)

- The **9.23 ms** figure is the **SIL control-loop** latency (frames arrive from
  Unity over TCP). Label it as SIL — do **not** claim it as the physical IMX500
  number. The physical number comes from the row you fill in §3.
- Reframe the term **"Sim2Real validation" → "software-in-the-loop validation
  with on-device latency measurement"** so the claim matches the evidence.
- State plainly that on the NPU path the detector runs on the sensor, so the
  reported L is the per-cycle CPU cost, not the inference time (report the
  camera/inference rate separately if asked).
