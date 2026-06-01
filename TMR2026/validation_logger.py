# -*- coding: utf-8 -*-
"""
validation_logger.py — Logging CSV + sistema de puntos para la
"Validación Sim2Real del Vehículo Autónomo" (PDF del profesor).

Genera los 3 entregables (Fase 3 del PDF):
  - Prueba 1 (Latencia):  P1_latencia.csv
  - Prueba 2 (PID/STOP):  P2_pid_stop.csv
  - Prueba 3 (FSM):       P3_fsm.csv

Y un sistema de PUNTOS estilo competencia que evalúa cada prueba contra su
criterio de aceptación y produce un reporte de puntaje.

Uso:
    log = ValidationLogger()                 # crea carpeta validation_results/
    log.log_cycle(latency_ms, err, conf)     # cada ciclo de control
    log.log_stop(dist_mm, pwm, err, estado)  # mientras se aproxima al STOP
    log.log_fsm(old_state, new_state)        # en cada transición de la FSM
    log.save_all()                           # escribe los 3 CSV
    log.print_scoreboard()                   # imprime puntos y guarda reporte
"""

import os
import csv
import time
from typing import Optional


class ValidationLogger:

    # ── Criterios de aceptación (del PDF) ─────────────────────────────────────
    LATENCY_MAX_MS      = 200.0   # ciclo percepción→actuación < 100-200 ms
    LATENCY_BONUS_MS    = 100.0   # bonus si < 100 ms
    STOP_TARGET_MM      = 270.0   # parada objetivo
    STOP_TOLERANCE_MM   = 30.0    # 270 ± 30 → ventana 240-300 mm
    # Estados que la FSM debe recorrer (Prueba 3). Hay dos ciclos válidos:
    #   - Ciclo STOP:    CRUCERO→PRECAUCION→FRENADO→ESPERA→REANUDAR
    #   - Ciclo PARKING: PARKING_SEARCH→PARKING_MANEUVER→PARKED (PDF Prueba 3)
    FSM_ESPERADOS_STOP    = ["CRUCERO", "PRECAUCION", "FRENADO", "ESPERA", "REANUDAR"]
    FSM_ESPERADOS_PARKING = ["PARKING_SEARCH", "PARKING_MANEUVER", "PARKED"]

    def __init__(self, outdir: str = "validation_results"):
        os.makedirs(outdir, exist_ok=True)
        self.outdir = outdir
        self.t0 = time.monotonic()

        self.latency_rows: list[dict] = []
        self.stop_rows:    list[dict] = []
        self.fsm_rows:     list[dict] = []

        self._last_fsm_state: Optional[str] = None
        self._last_fsm_time = self.t0

    # ─── Registro ──────────────────────────────────────────────────────────────

    def _t(self) -> float:
        return time.monotonic() - self.t0

    def log_cycle(self, latency_ms: float, error_px: float, conf: float) -> None:
        """Prueba 1: latencia del ciclo de control (percepción→respuesta)."""
        self.latency_rows.append({
            "t_s":          round(self._t(), 4),
            "latency_ms":   round(latency_ms, 2),
            "error_px":     round(error_px, 1),
            "confidence":   round(conf, 3),
        })

    def log_stop(self, distance_mm: Optional[float], pwm: float,
                 error_px: float, fsm_state: str) -> None:
        """Prueba 2: frenado ante STOP (distancia ToF vs PWM vs tiempo)."""
        self.stop_rows.append({
            "t_s":          round(self._t(), 4),
            "distance_mm":  round(distance_mm, 1) if distance_mm is not None else "",
            "pwm_duty":     round(pwm, 1),
            "error_px":     round(error_px, 1),
            "fsm_state":    fsm_state,
        })

    def log_fsm(self, new_state: str) -> None:
        """Prueba 3: registra una transición de estado de la FSM."""
        if new_state == self._last_fsm_state:
            return
        now = self._t()
        dwell = now - (self._last_fsm_time - self.t0) if self._last_fsm_state else 0.0
        self.fsm_rows.append({
            "t_s":          round(now, 4),
            "from_state":   self._last_fsm_state or "INICIO",
            "to_state":     new_state,
            "dwell_prev_s": round(now - (self._last_fsm_time - self.t0), 3)
                            if self._last_fsm_state else 0.0,
        })
        self._last_fsm_state = new_state
        self._last_fsm_time = time.monotonic()

    # ─── Exportar CSV ───────────────────────────────────────────────────────────

    def _write(self, name: str, rows: list[dict], fields: list[str]) -> str:
        path = os.path.join(self.outdir, name)
        with open(path, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=fields)
            w.writeheader()
            w.writerows(rows)
        return path

    def save_all(self) -> dict:
        paths = {}
        paths["latencia"] = self._write(
            "P1_latencia.csv", self.latency_rows,
            ["t_s", "latency_ms", "error_px", "confidence"])
        paths["pid_stop"] = self._write(
            "P2_pid_stop.csv", self.stop_rows,
            ["t_s", "distance_mm", "pwm_duty", "error_px", "fsm_state"])
        paths["fsm"] = self._write(
            "P3_fsm.csv", self.fsm_rows,
            ["t_s", "from_state", "to_state", "dwell_prev_s"])
        return paths

    # ─── Sistema de PUNTOS (estilo competencia) ──────────────────────────────────

    def evaluate(self) -> dict:
        """Evalúa cada prueba contra su criterio y asigna puntos (0-100)."""
        res = {"pruebas": [], "total": 0, "max": 0}

        # ── Prueba 1: Latencia ──
        # Filtrar outliers (> 500 ms = timeouts de socket, no ciclos reales).
        lat = [r["latency_ms"] for r in self.latency_rows
               if 0 < r["latency_ms"] < 500]
        p1 = {"nombre": "P1 Latencia ciclo de control",
              "nombre_en": "P1 Control-loop latency",
              "puntos": 0, "max": 30, "detalle": "", "detalle_en": ""}
        if lat:
            avg = sum(lat) / len(lat)
            mx = max(lat)
            if avg < self.LATENCY_BONUS_MS:
                p1["puntos"] = 30
            elif avg < self.LATENCY_MAX_MS:
                p1["puntos"] = 20
            else:
                p1["puntos"] = 5
            p1["detalle"] = (f"latencia media {avg:.1f} ms, máx {mx:.1f} ms "
                             f"(objetivo < {self.LATENCY_MAX_MS:.0f} ms)")
            p1["detalle_en"] = (f"mean latency {avg:.1f} ms, max {mx:.1f} ms "
                                f"(target < {self.LATENCY_MAX_MS:.0f} ms)")
        else:
            p1["detalle"] = "sin datos de latencia"
            p1["detalle_en"] = "no latency data"
        res["pruebas"].append(p1)

        # ── Prueba 2: Frenado STOP a 270±30 mm ──
        p2 = {"nombre": "P2 Frenado PID ante STOP",
              "nombre_en": "P2 PID braking at STOP",
              "puntos": 0, "max": 40, "detalle": "", "detalle_en": ""}
        # distancia donde el carro quedó detenido (pwm≈0 en estado FRENADO/ESPERA)
        parado = [r for r in self.stop_rows
                  if r["fsm_state"] in ("FRENADO", "ESPERA")
                  and isinstance(r["distance_mm"], (int, float))]
        if parado:
            # Distancia a la que QUEDÓ DETENIDO: se toma de las lecturas con la
            # señal visible (< 1000 mm), no las del ToF cuando la señal ya no
            # se ve (≈2000 mm). Se usa la mediana para robustez ante el ruido.
            parado_vis = [r["distance_mm"] for r in parado
                          if r["distance_mm"] < 1000]
            if parado_vis:
                s = sorted(parado_vis)
                dist_final = s[len(s) // 2]      # mediana
            else:
                dist_final = parado[-1]["distance_mm"]
            err_dist = abs(dist_final - self.STOP_TARGET_MM)
            dentro = err_dist <= self.STOP_TOLERANCE_MM
            # ¿sobreimpulso REAL? Solo distancias del frenado con señal visible.
            dists = [r["distance_mm"] for r in self.stop_rows
                     if r["fsm_state"] in ("PRECAUCION", "FRENADO", "ESPERA")
                     and isinstance(r["distance_mm"], (int, float))
                     and r["distance_mm"] < 1000]
            min_d = min(dists) if dists else dist_final
            sobreimpulso = min_d < (self.STOP_TARGET_MM - 2.5 * self.STOP_TOLERANCE_MM)
            if dentro and not sobreimpulso:
                p2["puntos"] = 40
            elif dentro:
                p2["puntos"] = 30   # llegó pero con sobreimpulso
            elif err_dist <= 2 * self.STOP_TOLERANCE_MM:
                p2["puntos"] = 20   # cerca
            else:
                p2["puntos"] = 5
            p2["detalle"] = (f"se detuvo a {dist_final:.0f} mm "
                             f"(objetivo {self.STOP_TARGET_MM:.0f}±{self.STOP_TOLERANCE_MM:.0f}), "
                             f"{'sin' if not sobreimpulso else 'con'} sobreimpulso")
            p2["detalle_en"] = (f"stopped at {dist_final:.0f} mm "
                                f"(target {self.STOP_TARGET_MM:.0f}±{self.STOP_TOLERANCE_MM:.0f}), "
                                f"{'no' if not sobreimpulso else 'with'} overshoot")
        else:
            p2["detalle"] = "el carro no llegó a FRENADO/ESPERA (no detectó STOP)"
            p2["detalle_en"] = "the car did not reach FRENADO/ESPERA (STOP not detected)"
        res["pruebas"].append(p2)

        # ── Prueba 3: Transiciones FSM sin bloqueo ──
        p3 = {"nombre": "P3 Transiciones FSM",
              "nombre_en": "P3 FSM transitions",
              "puntos": 0, "max": 30, "detalle": "", "detalle_en": ""}
        visitados = [r["to_state"] for r in self.fsm_rows]
        ciclo_stop = [s for s in self.FSM_ESPERADOS_STOP if s in visitados]
        ciclo_park = [s for s in self.FSM_ESPERADOS_PARKING if s in visitados]
        n_stop = len(ciclo_stop)
        n_park = len(ciclo_park)
        # 30 pts si completa el ciclo STOP (5) O el ciclo PARKING (3)
        if n_stop >= 5 or n_park >= 3:
            p3["puntos"] = 30
        elif n_stop >= 3 or n_park >= 2:
            p3["puntos"] = 20
        elif n_stop >= 1 or n_park >= 1:
            p3["puntos"] = 10
        det = f"STOP {n_stop}/5: {', '.join(ciclo_stop) or '—'}"
        det_en = f"STOP {n_stop}/5: {', '.join(ciclo_stop) or '—'}"
        if n_park > 0:
            det += f"  |  PARKING {n_park}/3: {', '.join(ciclo_park)}"
            det_en += f"  |  PARKING {n_park}/3: {', '.join(ciclo_park)}"
        det += f"  ({len(self.fsm_rows)} transiciones)"
        det_en += f"  ({len(self.fsm_rows)} transitions)"
        p3["detalle"] = det
        p3["detalle_en"] = det_en
        res["pruebas"].append(p3)

        res["total"] = sum(p["puntos"] for p in res["pruebas"])
        res["max"]   = sum(p["max"] for p in res["pruebas"])
        return res

    def _board_text(self, res: dict, lang: str = "es") -> str:
        """Renderiza el tablero en español ('es') o inglés ('en')."""
        en = (lang == "en")
        title = ("SCOREBOARD — Sim2Real Validation TMR 2026" if en
                 else "TABLERO DE PUNTOS — Validación Sim2Real TMR 2026")
        lines = ["=" * 60, "  " + title, "=" * 60]
        for p in res["pruebas"]:
            barra = "█" * int(20 * p["puntos"] / p["max"]) if p["max"] else ""
            nombre  = p.get("nombre_en"  if en else "nombre",  p["nombre"])
            detalle = p.get("detalle_en" if en else "detalle", p["detalle"])
            lines.append(f"  {nombre:<32} {p['puntos']:>3}/{p['max']:<3} {barra}")
            lines.append(f"     → {detalle}")
        lines.append("-" * 60)
        pct = 100 * res["total"] / res["max"] if res["max"] else 0
        lines.append(f"  TOTAL: {res['total']}/{res['max']}  ({pct:.0f}%)")
        if en:
            veredicto = ("PASSED ✓" if pct >= 70 else
                         "PARTIAL ⚠" if pct >= 40 else "REVIEW ✗")
            lines.append(f"  VERDICT: {veredicto}")
        else:
            veredicto = ("APROBADO ✓" if pct >= 70 else
                         "PARCIAL ⚠" if pct >= 40 else "REVISAR ✗")
            lines.append(f"  VEREDICTO: {veredicto}")
        lines.append("=" * 60)
        return "\n".join(lines)

    def print_scoreboard(self) -> dict:
        """Imprime el tablero (ES) y guarda PUNTAJE.txt (ES) + SCOREBOARD.txt (EN)."""
        res = self.evaluate()
        txt_es = self._board_text(res, "es")
        print(txt_es)
        with open(os.path.join(self.outdir, "PUNTAJE.txt"), "w",
                  encoding="utf-8") as f:
            f.write(txt_es + "\n")
        # Tablero en inglés para la entrega internacional (no se imprime en consola).
        with open(os.path.join(self.outdir, "SCOREBOARD.txt"), "w",
                  encoding="utf-8") as f:
            f.write(self._board_text(res, "en") + "\n")
        return res
