# -*- coding: utf-8 -*-
"""
analyze_results.py — Genera las GRÁFICAS del artículo científico a partir de
los 3 CSV de validation_results/ (creados por main_simulator.py --validate).

Salida (PNG en validation_results/):
  - fig1_latencia.png    Latencia del ciclo de control vs tiempo (Prueba 1)
  - fig2_frenado.png     Distancia ToF y PWM vs tiempo — frenado PID (Prueba 2)
  - fig3_fsm.png         Línea de tiempo de estados de la FSM (Prueba 3)

Uso:
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
    matplotlib.use("Agg")   # sin ventana, guarda PNG
    import matplotlib.pyplot as plt
except ImportError:
    print("Falta matplotlib. Instala con:  pip install matplotlib")
    sys.exit(1)

DIR = "validation_results"


def _read(name):
    path = os.path.join(DIR, name)
    if not os.path.exists(path):
        return []
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def _f(x):
    try: return float(x)
    except (ValueError, TypeError): return None


# ── Figura 1: Latencia ────────────────────────────────────────────────────────
def fig_latencia():
    rows = _read("P1_latencia.csv")
    if not rows:
        print("  (sin P1_latencia.csv)"); return
    t   = [_f(r["t_s"]) for r in rows]
    lat = [_f(r["latency_ms"]) for r in rows]
    avg = sum(lat) / len(lat)

    plt.figure(figsize=(9, 4.5))
    plt.plot(t, lat, lw=1.0, color="#2C7BE5", label="latencia por ciclo")
    plt.axhline(avg, color="#00A36C", ls="--", label=f"media {avg:.1f} ms")
    plt.axhline(200, color="#E55", ls=":", label="objetivo 200 ms")
    plt.xlabel("Tiempo (s)"); plt.ylabel("Latencia ciclo percepción→actuación (ms)")
    plt.title("Prueba 1: Latencia del ciclo de control")
    plt.legend(); plt.grid(alpha=0.3); plt.tight_layout()
    out = os.path.join(DIR, "fig1_latencia.png")
    plt.savefig(out, dpi=130); plt.close()
    print(f"  ✓ {out}  (media {avg:.1f} ms)")


# ── Figura 2: Frenado PID ante STOP ───────────────────────────────────────────
def fig_frenado():
    rows = _read("P2_pid_stop.csv")
    rows = [r for r in rows if _f(r["distance_mm"]) is not None]
    if not rows:
        print("  (sin P2_pid_stop.csv)"); return
    t    = [_f(r["t_s"]) for r in rows]
    dist = [_f(r["distance_mm"]) for r in rows]
    pwm  = [_f(r["pwm_duty"]) for r in rows]

    fig, ax1 = plt.subplots(figsize=(9, 4.5))
    ax1.plot(t, dist, color="#2C7BE5", lw=1.8, label="Distancia ToF (mm)")
    ax1.axhline(270, color="#00A36C", ls="--", label="objetivo 270 mm")
    ax1.axhspan(240, 300, color="#00A36C", alpha=0.12)
    ax1.set_xlabel("Tiempo (s)"); ax1.set_ylabel("Distancia (mm)", color="#2C7BE5")
    ax1.tick_params(axis="y", labelcolor="#2C7BE5")

    ax2 = ax1.twinx()
    ax2.plot(t, pwm, color="#E5703A", lw=1.5, label="PWM motor (%)")
    ax2.set_ylabel("PWM motor (%)", color="#E5703A")
    ax2.tick_params(axis="y", labelcolor="#E5703A")

    plt.title("Prueba 2: Frenado PID ante señal STOP")
    l1, lab1 = ax1.get_legend_handles_labels()
    l2, lab2 = ax2.get_legend_handles_labels()
    ax1.legend(l1 + l2, lab1 + lab2, loc="upper right")
    plt.tight_layout()
    out = os.path.join(DIR, "fig2_frenado.png")
    plt.savefig(out, dpi=130); plt.close()
    print(f"  ✓ {out}")


# ── Figura 3: Línea de tiempo FSM ─────────────────────────────────────────────
def fig_fsm():
    rows = _read("P3_fsm.csv")
    if not rows:
        print("  (sin P3_fsm.csv)"); return
    estados_orden = ["CRUCERO", "PRECAUCION", "FRENADO", "ESPERA", "REANUDAR",
                     "PARKING_SEARCH", "PARKING_MANEUVER"]
    ypos = {s: i for i, s in enumerate(estados_orden)}

    # construir segmentos (estado, t_ini, t_fin)
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
    plt.yticks(range(len(estados_orden)), estados_orden)
    plt.xlabel("Tiempo (s)"); plt.ylabel("Estado FSM")
    plt.title("Prueba 3: Línea de tiempo de la Máquina de Estados")
    plt.grid(alpha=0.3, axis="x"); plt.tight_layout()
    out = os.path.join(DIR, "fig3_fsm.png")
    plt.savefig(out, dpi=130); plt.close()
    print(f"  ✓ {out}")


if __name__ == "__main__":
    print("Generando gráficas del artículo desde validation_results/ ...")
    if not os.path.isdir(DIR):
        print(f"No existe la carpeta '{DIR}'. Corre primero:")
        print("   python main_simulator.py --validate --duration 60")
        sys.exit(1)
    fig_latencia()
    fig_frenado()
    fig_fsm()
    print("Listo. Las 3 figuras están en validation_results/ para el artículo.")
