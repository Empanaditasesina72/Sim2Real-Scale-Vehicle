# -*- coding: utf-8 -*-
"""
test_fsm_transitions.py — verifica las 5 transiciones de AutonomousFSM y
la interacción con signals + brake_light.  Sin hardware.

Transiciones probadas:
  1. CRUCERO → PRECAUCION (por sign_visible)
  2. PRECAUCION → FRENADO  (por lidar ≤ LIDAR_STOP_MM)
  3. PRECAUCION → FRENADO  (por bbox cuando NO hay lidar)   ← fallback
  4. ESPERA usa time.monotonic() y NO bloquea durante los 5 s
  5. FRENADO/ESPERA encienden brake_light + HAZARD
"""

import time
from control.fsm import AutonomousFSM, FSMState
from hardware.signals import SignalMode


# ─── Dobles de prueba (mocks mínimos) ─────────────────────────────────────────

class FakeMotor:
    def __init__(self):
        self.duty = 0.0
        self.brake_calls = 0
    def set_speed(self, v): self.duty = v
    def brake(self):        self.duty = 0.0; self.brake_calls += 1


class FakeSteering:
    def __init__(self): self.angle = 90.0
    def set_angle(self, a): self.angle = a
    def center(self):       self.angle = 90.0


class FakePID:
    def __init__(self): self.reset_calls = 0
    def compute(self, err, dt): return 0.0
    def reset(self):            self.reset_calls += 1


class FakeSignals:
    def __init__(self): self.mode = SignalMode.OFF; self.ticks = 0
    def set_mode(self, m): self.mode = m
    def tick(self):        self.ticks += 1


class FakeBrake:
    def __init__(self): self.on_calls = 0; self.off_calls = 0; self.state = False
    def on(self):
        if not self.state: self.on_calls += 1
        self.state = True
    def off(self):
        if self.state: self.off_calls += 1
        self.state = False


def _make_fsm():
    return AutonomousFSM(
        motor=FakeMotor(),
        steering=FakeSteering(),
        pid=FakePID(),
        signals=FakeSignals(),
        brake_light=FakeBrake(),
    )


# ─── Tests ────────────────────────────────────────────────────────────────────

def test_crucero_to_precaucion_on_sign():
    fsm = _make_fsm()
    fsm.activate()
    fsm.lane_conf    = 1.0
    fsm.lane_error   = 0.0
    fsm.sign_visible = True
    fsm.update(dt=0.02)
    assert fsm.state == FSMState.PRECAUCION
    assert fsm.signals.mode == SignalMode.HAZARD    # aviso encendido
    assert fsm.brake_light.state is False           # aún no frenando


def test_precaucion_to_frenado_by_lidar():
    fsm = _make_fsm()
    fsm.activate()
    fsm.lane_conf    = 1.0
    fsm.sign_visible = True
    fsm.update(0.02)                               # → PRECAUCION
    fsm.lidar_mm     = 250                         # ≤ LIDAR_STOP_MM (300)
    fsm.update(0.02)                               # → FRENADO → ESPERA
    # El ciclo FRENADO transiciona inmediato a ESPERA
    assert fsm.state in (FSMState.FRENADO, FSMState.ESPERA)
    assert fsm.brake_light.state is True
    assert fsm.signals.mode == SignalMode.HAZARD
    assert fsm.motor.brake_calls >= 1


def test_precaucion_to_frenado_by_bbox_when_no_lidar():
    """Fallback: sin lidar, el bbox dispara FRENADO cuando la señal está cerca."""
    fsm = _make_fsm()
    fsm.activate()
    fsm.lane_conf    = 1.0
    fsm.sign_visible = True
    fsm.update(0.02)                               # → PRECAUCION
    assert fsm.state == FSMState.PRECAUCION

    fsm.lidar_mm         = None                   # lidar caído
    fsm.sign_distance_mm = 300                    # 30 cm → ≤ 350
    fsm.update(0.02)
    assert fsm.state in (FSMState.FRENADO, FSMState.ESPERA)
    assert fsm.brake_light.state is True


def test_espera_uses_monotonic_not_sleep(monkeypatch):
    """La FSM NO debe dormir — el tiempo avanza con monotonic()."""
    fake_t = [10_000.0]
    monkeypatch.setattr(time, "monotonic", lambda: fake_t[0])

    fsm = _make_fsm()
    fsm.activate()
    fsm.lane_conf    = 1.0
    fsm.sign_visible = True
    fsm.lidar_mm     = 100

    # Iterar updates hasta llegar a ESPERA (≤4 transiciones)
    for _ in range(5):
        fsm.update(0.02)
        if fsm.state == FSMState.ESPERA:
            break
    assert fsm.state == FSMState.ESPERA
    assert fsm.brake_light.state is True

    # A los 4.5 s todavía NO debe salir
    fake_t[0] += 4.5
    fsm.update(0.02)
    assert fsm.state == FSMState.ESPERA

    # A los 5.1 s debe saltar a REANUDAR
    fake_t[0] += 0.7
    fsm.update(0.02)
    assert fsm.state == FSMState.REANUDAR
    assert fsm.brake_light.state is False   # freno suelto al reanudar
    assert fsm.signals.mode == SignalMode.OFF


def test_signals_tick_called_every_update():
    """El parpadeo debe avanzar en cada update(), activa o no la FSM."""
    fsm = _make_fsm()
    # Incluso sin activate(), tick() sigue corriendo (para no congelar LEDs)
    for _ in range(5):
        fsm.update(0.02)
    assert fsm.signals.ticks == 5
