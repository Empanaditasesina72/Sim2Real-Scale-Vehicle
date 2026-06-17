"""Verify that TurnSignals blinks at 2 Hz using time.monotonic() (no sleep)
and that HAZARD turns on both LEDs in phase.

No hardware: TurnSignals is subclassed to intercept _write() and
time.monotonic() is mocked to control the blink progression.
"""

import time
from hardware.signals import TurnSignals, SignalMode


class FakeSignals(TurnSignals):
    """Subclase que NO toca GPIO y registra cada escritura."""
    def __init__(self, blink_hz=2.0):
        self.writes: list[tuple[int, int]] = []
        super().__init__(pin_left=100, pin_right=101, blink_hz=blink_hz)

    def _setup_gpio(self):
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
    """OFF must never write 1 to any LED."""
    fake_t = [0.0]
    monkeypatch.setattr(time, "monotonic", lambda: fake_t[0])

    sig = FakeSignals()
    for _ in range(10):
        fake_t[0] += 0.30
        sig.tick()
    assert all(v == 0 for _, v in sig.writes)

    sig.set_mode(SignalMode.LEFT)
    sig.set_mode(SignalMode.OFF)
    sig.tick()
    assert _last_write(sig, 100) == 0
    assert _last_write(sig, 101) == 0


def test_left_only_blinks_left_pin(monkeypatch):
    sig = FakeSignals(blink_hz=2.0)
    fake_t = [1000.0]
    monkeypatch.setattr(time, "monotonic", lambda: fake_t[0])

    sig.set_mode(SignalMode.LEFT)
    assert _last_write(sig, 100) == 1
    assert _last_write(sig, 101) == 0

    fake_t[0] += 0.30
    sig.tick()
    assert _last_write(sig, 100) == 0
    assert _last_write(sig, 101) == 0

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
    assert _last_write(sig, 101) == 0

    fake_t[0] += 0.30
    sig.tick()
    assert _last_write(sig, 100) == 1
    assert _last_write(sig, 101) == 1


def test_tick_does_not_block():
    """tick() debe retornar en <1 ms aunque se llame muchas veces."""
    sig = FakeSignals()
    sig.set_mode(SignalMode.HAZARD)
    t0 = time.perf_counter()
    for _ in range(1000):
        sig.tick()
    elapsed = time.perf_counter() - t0
    assert elapsed < 0.1
