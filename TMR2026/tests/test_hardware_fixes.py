"""Regression tests for the driver fixes of 2026-10-03. No GPIO, no camera:
the motor falls back to its mock backend off the Pi, and CameraStream is
built without its constructor so Picamera2 is never imported."""

import threading
import time

import numpy as np


class _SlowReleaseLock:
    """Lock that stalls the ramp thread right AFTER each release.

    That is exactly the gap the old ramp loop had between reading the duty and
    writing it back; stretching it to 4 ms makes a brake() land inside it on
    most trials, so the race reproduces deterministically instead of once in a
    million ticks. Harmless for a loop that holds the lock across the write.
    """

    def __init__(self):
        self._l = threading.Lock()

    def __enter__(self):
        self._l.acquire()
        return self

    def __exit__(self, *exc):
        self._l.release()
        if threading.current_thread().name == "MotorRamp":
            time.sleep(0.004)


def test_brake_is_never_undone_by_the_ramp():
    from hardware.motor import MotorDriver

    m = MotorDriver()
    m._lock = _SlowReleaseLock()
    writes = []
    lock = threading.Lock()
    orig = m._apply_hw

    def spy(duty):
        with lock:
            writes.append(duty)
        orig(duty)

    m._apply_hw = spy
    try:
        for _ in range(40):
            m.set_speed(60.0)
            time.sleep(0.03)
            m.set_speed(5.0)                   # ramping DOWN: 8 %/tick steps
            time.sleep(0.005)
            m.brake()
            with lock:
                writes.clear()
            time.sleep(0.05)                   # >2 ramp ticks after the brake
            assert m.current_duty == 0.0
            with lock:
                assert all(w == 0.0 for w in writes), writes
    finally:
        m.cleanup()


def _bare_camera():
    from vision.camera_stream import CameraStream
    cam = CameraStream.__new__(CameraStream)
    cam._lock = threading.Lock()
    cam._frame = None
    cam._frame_t = 0.0
    return cam


def test_camera_serves_fresh_frames_and_drops_stale_ones():
    cam = _bare_camera()
    assert cam.get_frame() is None

    cam._frame = np.zeros((4, 4, 3), np.uint8)
    cam._frame_t = time.monotonic()
    f = cam.get_frame()
    assert f is not None and f is not cam._frame        # a copy
    assert cam.frame_age_s < 0.5

    cam._frame_t = time.monotonic() - (cam.STALE_FRAME_S + 0.1)
    assert cam.get_frame() is None                      # capture stalled
