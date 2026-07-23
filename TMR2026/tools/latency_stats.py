"""Compute the full latency distribution from a per-cycle latency CSV.

Turns a per-control-cycle latency log (column ``latency_ms``, one row per
cycle) into the statistics that reviewers require for a real-time control
system: mean, median, standard deviation, p90/p95/p99, jitter, and the
percentage of cycles that miss the control deadline.

It works on any CSV with a ``latency_ms`` column, so the SAME tool analyses
both the simulator log (validation_results/P1_latency.csv) and the on-device
Raspberry Pi benchmark (tools/bench_latency.py output).

Formal latency definition (state this in the paper):
    Per-cycle latency = wall-clock time to run one control iteration:
    read the latest frame -> lane pipeline -> sign gating -> PID/command.
    A cycle "misses the deadline" when this time exceeds 1000/LOOP_HZ ms
    (20 ms for the 50 Hz loop).

Usage:
    python tools/latency_stats.py                              # default P1
    python tools/latency_stats.py path/to/latency.csv --deadline 20 --label "Pi 5 (IMX500)"
"""

import argparse
import os
import sys

import numpy as np

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    _HAVE_MPL = True
except ImportError:
    _HAVE_MPL = False

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _load_latencies(path: str, column: str = "latency_ms") -> np.ndarray:
    import csv
    vals = []
    with open(path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        if column not in reader.fieldnames:
            sys.exit(f"[ERROR] column '{column}' not in {path} "
                     f"(found: {reader.fieldnames})")
        for row in reader:
            try:
                v = float(row[column])
            except (ValueError, TypeError):
                continue
            if v > 0:
                vals.append(v)
    return np.asarray(vals, dtype=np.float64)


def summarize(lat: np.ndarray, deadline_ms: float) -> dict:
    return {
        "n":            int(lat.size),
        "mean":         float(np.mean(lat)),
        "median":       float(np.median(lat)),
        "std":          float(np.std(lat, ddof=1)) if lat.size > 1 else 0.0,
        "min":          float(np.min(lat)),
        "max":          float(np.max(lat)),
        "p90":          float(np.percentile(lat, 90)),
        "p95":          float(np.percentile(lat, 95)),
        "p99":          float(np.percentile(lat, 99)),
        "iqr":          float(np.percentile(lat, 75) - np.percentile(lat, 25)),
        "jitter_std":   float(np.std(lat, ddof=1)) if lat.size > 1 else 0.0,
        "deadline_ms":  float(deadline_ms),
        "misses":       int(np.sum(lat > deadline_ms)),
        "miss_pct":     float(100.0 * np.mean(lat > deadline_ms)),
        "under_pct":    float(100.0 * np.mean(lat <= deadline_ms)),
    }


def print_table(s: dict, label: str) -> str:
    lines = [
        "=" * 58,
        f"  LATENCY DISTRIBUTION  -  {label}",
        "=" * 58,
        f"  samples (control cycles) : {s['n']}",
        f"  mean                     : {s['mean']:.2f} ms",
        f"  median                   : {s['median']:.2f} ms",
        f"  std deviation (jitter)   : {s['std']:.2f} ms",
        f"  min / max                : {s['min']:.2f} / {s['max']:.2f} ms",
        f"  p90 / p95 / p99          : {s['p90']:.2f} / {s['p95']:.2f} / {s['p99']:.2f} ms",
        f"  IQR                      : {s['iqr']:.2f} ms",
        "-" * 58,
        f"  deadline ({s['deadline_ms']:.0f} ms @ 50 Hz)",
        f"  cycles under deadline    : {s['under_pct']:.2f} %",
        f"  deadline misses          : {s['misses']} ({s['miss_pct']:.3f} %)",
        "=" * 58,
    ]
    return "\n".join(lines)


def make_figure(lat: np.ndarray, s: dict, label: str, out_png: str) -> None:
    if not _HAVE_MPL:
        print("[WARN] matplotlib missing, skipping figure")
        return
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.2))

    ax1.hist(lat, bins=60, color="#2C7BE5", alpha=0.85, edgecolor="none")
    ax1.axvline(s["mean"], color="#00A36C", ls="--", lw=1.5, label=f"mean {s['mean']:.1f} ms")
    ax1.axvline(s["p99"],  color="#E5703A", ls=":",  lw=1.5, label=f"p99 {s['p99']:.1f} ms")
    ax1.axvline(s["deadline_ms"], color="#E55", ls="-", lw=1.5,
                label=f"deadline {s['deadline_ms']:.0f} ms")
    ax1.set_xlabel("Control-loop latency (ms)")
    ax1.set_ylabel("Count (control cycles)")
    ax1.set_title(f"Latency histogram  (n={s['n']})")
    ax1.legend(fontsize=8)
    ax1.grid(alpha=0.3)

    xs = np.sort(lat)
    ys = np.arange(1, xs.size + 1) / xs.size
    ax2.plot(xs, ys, color="#2C7BE5", lw=1.8)
    for p, lbl in [(s["p95"], "p95"), (s["p99"], "p99")]:
        ax2.axvline(p, color="#E5703A", ls=":", lw=1.0)
        ax2.text(p, 0.05, f" {lbl}", color="#E5703A", fontsize=8)
    ax2.axvline(s["deadline_ms"], color="#E55", ls="-", lw=1.5, label=f"deadline {s['deadline_ms']:.0f} ms")
    ax2.set_xlabel("Control-loop latency (ms)")
    ax2.set_ylabel("Cumulative probability")
    ax2.set_title("Latency CDF")
    ax2.legend(fontsize=8)
    ax2.grid(alpha=0.3)

    fig.suptitle(f"Perception-to-actuation control-loop latency  -  {label}", fontsize=11)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(out_png, dpi=300)
    plt.close(fig)
    print(f"  [OK] figure (300 dpi): {out_png}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    default_csv = os.path.join(HERE, "validation_results", "P1_latency.csv")
    if not os.path.exists(default_csv):
        alt = os.path.join(HERE, "validation_results", "P1_latencia.csv")
        if os.path.exists(alt):
            default_csv = alt
    ap.add_argument("csv", nargs="?", default=default_csv, help="per-cycle latency CSV")
    ap.add_argument("--deadline", type=float, default=20.0, help="deadline in ms (50 Hz -> 20)")
    ap.add_argument("--label", default="SIL (Unity, control-loop)", help="label for the report/figure")
    ap.add_argument("--column", default="latency_ms")
    ap.add_argument("--drop-first", type=int, default=1, help="drop N warm-up cycles")
    args = ap.parse_args()

    if not os.path.exists(args.csv):
        sys.exit(f"[ERROR] not found: {args.csv}")

    lat = _load_latencies(args.csv, args.column)
    if args.drop_first > 0:
        lat = lat[args.drop_first:]
    if lat.size == 0:
        sys.exit("[ERROR] no latency samples")

    s = summarize(lat, args.deadline)
    table = print_table(s, args.label)
    print(table)

    out_dir = os.path.dirname(os.path.abspath(args.csv))
    base = os.path.splitext(os.path.basename(args.csv))[0]
    with open(os.path.join(out_dir, base + "_stats.txt"), "w", encoding="utf-8") as f:
        f.write(table + "\n")
    make_figure(lat, s, args.label, os.path.join(out_dir, base + "_distribution.png"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
