"""Generate the article FIGURES from the 3 CSVs in validation_results/
(created by main_simulator.py --validate).

Output (PNG in validation_results/):
  - fig1_latency.png     Control-loop latency vs time (Test 1)
  - fig2_braking.png     ToF distance and PWM vs time -- PID braking (Test 2)
  - fig3_fsm.png         FSM state timeline (Test 3)

Usage:
    python analyze_results.py
"""

import os
import csv
import sys

try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError:
    print("matplotlib is missing. Install it with:  pip install matplotlib")
    sys.exit(1)

DIR = "validation_results"
for _a in sys.argv[1:]:
    if _a.startswith("validation_results"):
        DIR = _a


def _read(name):
    path = os.path.join(DIR, name)
    if not os.path.exists(path):
        return []
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def _f(x):
    try: return float(x)
    except (ValueError, TypeError): return None


def fig_latency():
    rows = _read("P1_latency.csv")
    if not rows:
        print("  (no P1_latency.csv)"); return
    t   = [_f(r["t_s"]) for r in rows]
    lat = [_f(r["latency_ms"]) for r in rows]
    avg = sum(lat) / len(lat)

    plt.figure(figsize=(9, 4.5))
    plt.plot(t, lat, lw=1.0, color="#2C7BE5", label="latency per cycle")
    plt.axhline(avg, color="#00A36C", ls="--", label=f"mean {avg:.1f} ms")
    plt.axhline(20, color="#E55", ls=":", label="deadline 20 ms (50 Hz)")
    plt.xlabel("Time (s)")
    plt.ylabel("Perception-to-actuation loop latency (ms)")
    plt.title("Test 1: Control-loop latency")
    plt.legend(); plt.grid(alpha=0.3); plt.tight_layout()
    out = os.path.join(DIR, "fig1_latency.png")
    plt.savefig(out, dpi=300); plt.close()
    print(f"  [OK] {out}  (avg {avg:.1f} ms)")


def fig_braking():
    rows = _read("P2_pid_stop.csv")
    rows = [r for r in rows if _f(r["distance_mm"]) is not None]
    if not rows:
        print("  (no P2_pid_stop.csv)"); return
    t    = [_f(r["t_s"]) for r in rows]
    dist = [_f(r["distance_mm"]) for r in rows]
    pwm  = [_f(r["pwm_duty"]) for r in rows]

    fig, ax1 = plt.subplots(figsize=(9, 4.5))
    ax1.plot(t, dist, color="#2C7BE5", lw=1.8, label="ToF distance (mm)")
    ax1.axhline(270, color="#00A36C", ls="--", label="target 270 mm")
    ax1.axhspan(240, 300, color="#00A36C", alpha=0.12)
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Distance (mm)", color="#2C7BE5")
    ax1.tick_params(axis="y", labelcolor="#2C7BE5")

    ax2 = ax1.twinx()
    ax2.plot(t, pwm, color="#E5703A", lw=1.5, label="motor PWM (%)")
    ax2.set_ylabel("motor PWM (%)", color="#E5703A")
    ax2.tick_params(axis="y", labelcolor="#E5703A")

    plt.title("Test 2: PID braking at STOP sign")
    l1, lab1 = ax1.get_legend_handles_labels()
    l2, lab2 = ax2.get_legend_handles_labels()
    ax1.legend(l1 + l2, lab1 + lab2, loc="upper right")
    plt.tight_layout()
    out = os.path.join(DIR, "fig2_braking.png")
    plt.savefig(out, dpi=300); plt.close()
    print(f"  [OK] {out}")


def fig_fsm():
    rows = _read("P3_fsm.csv")
    if not rows:
        print("  (no P3_fsm.csv)"); return
    states_order = ["CRUCERO", "PRECAUCION", "FRENADO", "ESPERA", "REANUDAR",
                    "PARKING_SEARCH", "PARKING_MANEUVER"]
    ypos = {s: i for i, s in enumerate(states_order)}

    segs = []
    for i, r in enumerate(rows):
        st = r["to_state"]
        t0 = _f(r["t_s"])
        t1 = _f(rows[i + 1]["t_s"]) if i + 1 < len(rows) else t0 + 1.0
        if st in ypos:
            segs.append((st, t0, t1))

    plt.figure(figsize=(10, 4.5))
    for st, t0, t1 in segs:
        plt.plot([t0, t1], [ypos[st], ypos[st]], lw=8, solid_capstyle="butt",
                 color="#2C7BE5")
        plt.plot(t0, ypos[st], "o", color="#E5703A")
    plt.yticks(range(len(states_order)), states_order)
    plt.xlabel("Time (s)")
    plt.ylabel("FSM state")
    plt.title("Test 3: State Machine timeline")
    plt.grid(alpha=0.3, axis="x"); plt.tight_layout()
    out = os.path.join(DIR, "fig3_fsm.png")
    plt.savefig(out, dpi=300); plt.close()
    print(f"  [OK] {out}")


if __name__ == "__main__":
    print("Generating article figures from validation_results/ ...")
    if not os.path.isdir(DIR):
        print(f"Folder '{DIR}' does not exist. Run first:")
        print("   python main_simulator.py --validate --duration 60")
        sys.exit(1)
    fig_latency()
    fig_braking()
    fig_fsm()
    print("Done. 3 figures in validation_results/ "
          "(fig1_latency, fig2_braking, fig3_fsm).")
