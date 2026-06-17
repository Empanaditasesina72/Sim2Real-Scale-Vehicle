"""Bluetooth gamepad (generic Xbox/PS4) via pygame.

The reader thread polls the joystick at 100 Hz and exposes the state through
thread-safe properties. The main loop never blocks.

Default mapping (configurable in config.py):
  Axis 3   -> Right stick X   (steering)
  Axis 5   -> R2 trigger       (throttle, range [-1, 1])
  Axis 4   -> L2 trigger       (brake)
  Button 0 -> Back to Manual
  Button 1 -> Vision Test mode
  Button 2 -> Autonomous mode
"""

import threading
import time

import pygame

from config import (
    BTN_BACK_TO_MANUAL, BTN_VISION_TEST, BTN_AUTONOMOUS,
    AXIS_STEER, AXIS_THROTTLE, AXIS_BRAKE,
    JOYSTICK_DEADBAND, TRIGGER_DEADBAND,
)


class GamepadState:
    """Snapshot inmutable del estado del mando en un instante."""
    __slots__ = (
        "throttle", "brake", "steer",
        "btn_manual", "btn_vision", "btn_auto",
        "connected",
    )

    def __init__(
        self,
        throttle: float = 0.0,
        brake: float = 0.0,
        steer: float = 0.0,
        btn_manual: bool = False,
        btn_vision: bool = False,
        btn_auto: bool = False,
        connected: bool = False,
    ):
        self.throttle  = throttle
        self.brake     = brake
        self.steer     = steer
        self.btn_manual = btn_manual
        self.btn_vision = btn_vision
        self.btn_auto   = btn_auto
        self.connected  = connected


class GamepadReader:
    """
    Lector de gamepad Bluetooth en hilo dedicado.

    Detecta reconexión automáticamente: si el mando se desconecta,
    el sistema regresa a STANDBY y el hilo espera hasta que vuelva
    a estar disponible.
    """

    POLL_HZ = 100

    def __init__(self):
        pygame.init()
        pygame.joystick.init()

        self._lock  = threading.Lock()
        self._state = GamepadState()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

        self._prev_buttons: dict[int, bool] = {
            BTN_BACK_TO_MANUAL: False,
            BTN_VISION_TEST: False,
            BTN_AUTONOMOUS: False,
        }
        self._button_pressed: dict[int, bool] = {
            BTN_BACK_TO_MANUAL: False,
            BTN_VISION_TEST: False,
            BTN_AUTONOMOUS: False,
        }

    def start(self):
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._poll_loop,
            name="Gamepad",
            daemon=True,
        )
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        pygame.quit()

    @property
    def state(self) -> GamepadState:
        with self._lock:
            return self._state

    @property
    def is_connected(self) -> bool:
        return self.state.connected

    def consume_button(self, btn_id: int) -> bool:
        """
        Devuelve True UNA SOLA VEZ cuando el botón fue presionado
        (flanco de subida), luego lo resetea.  Útil para cambios de modo.
        """
        with self._lock:
            pressed = self._button_pressed.get(btn_id, False)
            if pressed:
                self._button_pressed[btn_id] = False
            return pressed

    def _poll_loop(self):
        interval = 1.0 / self.POLL_HZ
        joy: pygame.joystick.JoystickType | None = None

        while not self._stop_event.is_set():
            if joy is None:
                pygame.joystick.quit()
                pygame.joystick.init()
                if pygame.joystick.get_count() > 0:
                    joy = pygame.joystick.Joystick(0)
                    joy.init()
                    for _ in range(5):
                        pygame.event.pump()
                        time.sleep(0.02)
                else:
                    with self._lock:
                        self._state = GamepadState(connected=False)
                    time.sleep(0.5)
                    continue

            try:
                pygame.event.pump()
            except pygame.error:
                joy = None
                continue

            try:
                raw_steer    = joy.get_axis(AXIS_STEER)
                raw_throttle = joy.get_axis(AXIS_THROTTLE)
                raw_brake    = joy.get_axis(AXIS_BRAKE)
            except pygame.error:
                joy = None
                continue

            steer    = self._apply_deadband(raw_steer, JOYSTICK_DEADBAND)
            throttle = self._trigger_to_01(raw_throttle, TRIGGER_DEADBAND)
            brake    = self._trigger_to_01(raw_brake,    TRIGGER_DEADBAND)

            btn_states: dict[int, bool] = {}
            for btn_id in (BTN_BACK_TO_MANUAL, BTN_VISION_TEST, BTN_AUTONOMOUS):
                try:
                    current = bool(joy.get_button(btn_id))
                except pygame.error:
                    current = False
                btn_states[btn_id] = current

            with self._lock:
                for btn_id, current in btn_states.items():
                    prev = self._prev_buttons[btn_id]
                    if current and not prev:
                        self._button_pressed[btn_id] = True
                    self._prev_buttons[btn_id] = current

                self._state = GamepadState(
                    throttle   = throttle,
                    brake      = brake,
                    steer      = steer,
                    btn_manual = btn_states[BTN_BACK_TO_MANUAL],
                    btn_vision = btn_states[BTN_VISION_TEST],
                    btn_auto   = btn_states[BTN_AUTONOMOUS],
                    connected  = True,
                )

            time.sleep(interval)

    @staticmethod
    def _apply_deadband(value: float, band: float) -> float:
        if abs(value) < band:
            return 0.0
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - band) / (1.0 - band)

    @staticmethod
    def _trigger_to_01(raw: float, deadband: float) -> float:
        """Convierte el rango de gatillo [-1, 1] → [0, 1]."""
        normalized = (raw + 1.0) / 2.0
        return 0.0 if normalized < deadband else normalized
