# -*- coding: utf-8 -*-
"""
test_signals.py — verifica que TurnSignals parpadea a 2 Hz usando
time.monotonic() (sin sleep) y que HAZARD enciende ambos LEDs en fase.

Sin hardware: se subclasea TurnSignals para interceptar _write() y
se mockea time.monotonic() para controlar el avance del parpadeo.
"""

import time
from hardware.signals import TurnSignals, SignalMode


class FakeSignals(TurnSignals):
    """Subclase que NO toca GPIO y registra cada escritura."""
    def __init__(self, blink_hz=2.0):
        self.writes: list[tuple[int, int]] = []
        super().__init__(pin_left=100, pin_right=101, blink_hz=blink_hz)

    def _setup_gpio(self):
        # Forzar modo 'sin backend': no se tocan pines reales
        self._backend = None

    def _write(self, pin, value):
        self.writes.append((pin, value))


def _last_write(sig: FakeSignals, pin: int):
    """Último valor escrito al pin, o None."""
    for p, v in reversed(sig.writes):
        if p == pin:
            return v
    return None


def test_off_keeps_both_leds_low(monkeypatch):
    """OFF nunca debe escribir 1 a ningún LED."""
    fake_t = [0.0]
    monkeypatch.setattr(time, "monotonic", lambda: fake_t[0])

    sig = FakeSignals()
    # Estado inicial OFF — tick() no escribe nada
    for _ in range(10):
        fake_t[0] += 0.30
        sig.tick()
    assert all(v == 0 for _, v in sig.writes)

    # Cambiar a LEFT y volver a OFF debe forzar 0 en ambos pines
    sig.set_mode(SignalMode.LEFT)
    sig.set_mode(SignalMode.OFF)
    sig.tick()
    assert _last_write(sig, 100) == 0
    assert _last_write(sig, 101) == 0


def test_left_only_blinks_left_pin(monkeypatch):
    sig = FakeSignals(blink_hz=2.0)   # semi-periodo = 0.25 s
    fake_t = [1000.0]
    monkeypatch.setattr(time, "monotonic", lambda: fake_t[0])

    sig.set_mode(SignalMode.LEFT)    # t=1000.0 → blink_on=True
    assert _last_write(sig, 100) == 1
    assert _last_write(sig, 101) == 0

    fake_t[0] += 0.30   # supera semi-periodo → toggle
    sig.tick()
    assert _last_write(sig, 100) == 0
    assert _last_write(sig, 101) == 0   # derecho nunca enciende

    fake_t[0] += 0.30
    sig.tick()
    assert _last_write(sig, 100) == 1
    assert _last_write(sig, 101) == 0


def test_hazard_blinks_both_in_phase(monkeypatch):
    sig = FakeSignals(blink_hz=2.0)
    fake_t = [0.0]
    monkeypatch.setattr(time, "monotonic", lambda: fake_t[0])

    sig.set_mode(SignalMode.HAZARD)
    assert _last_write(sig, 100) == 1
    assert _last_write(sig, 101) == 1

    fake_t[0] += 0.30
    sig.tick()
    assert _last_write(sig, 100) == 0
    assert _last_write(sig, 101) == 0    # ambos apagados a la vez

    fake_t[0] += 0.30
    sig.tick()
    assert _last_write(sig, 100) == 1
    assert _last_write(sig, 101) == 1    # ambos encendidos a la vez


def test_tick_does_not_block():
    """tick() debe retornar en <1 ms aunque se llame muchas veces."""
    sig = FakeSignals()
    sig.set_mode(SignalMode.HAZARD)
    t0 = time.perf_counter()
    for _ in range(1000):
        sig.tick()
    elapsed = time.perf_counter() - t0
    assert elapsed < 0.1   # 1000 ticks deben tardar <100 ms
