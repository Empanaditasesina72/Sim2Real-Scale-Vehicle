# -*- coding: utf-8 -*-
"""
main.py — Sistema TMR 2026.

Botones PS4 / Xbox:
  Cuadrado / X  → Autónomo  (TOGGLE — presionar de nuevo apaga)
  Círculo  / B  → Visión    (cámara encendida, motores OFF)
  Por defecto   → Manual

Manual:
  Palanca izquierda X → servo (dirección)
  R2 (gatillo)        → motor adelante (progresivo)
  L2 (gatillo)        → reversa suave
"""

import sys
import time
import signal

import RPi.GPIO as GPIO

from config import (
    PIN_LED_STOP, PIN_LED_STATUS,
    SERVO_CENTER_ANGLE,
    BTN_MANUAL, BTN_VISION, BTN_AUTONOMOUS, BTN_PARKING,
)
from hardware.motor_driver    import MotorDriver
from hardware.steering_driver import SteeringDriver
from hardware.distance_sensor import DistanceSensor
from hardware.camera_manager  import CameraManager
from control.gamepad_reader   import GamepadReader
from vision.lane_detector     import LaneDetector, LaneData
from vision.object_detector   import ObjectDetector
from autonomy.autonomous_mode import AutonomousController


class VehicleMode:
    STANDBY    = "STANDBY"
    MANUAL     = "MANUAL"
    VISION     = "VISION"
    AUTONOMOUS = "AUTONOMOUS"
    PARKING    = "PARKING"


class CarritoTMR:

    LOOP_HZ = 50
    MODE_COOLDOWN = 0.4   # segundos mínimos entre cambios de modo

    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        self._setup_leds()

        print("[INIT] Inicializando hardware...")
        self.motor    = MotorDriver()
        self.steering = SteeringDriver()
        self.sensor   = DistanceSensor()
        self.camera   = CameraManager()
        self.gamepad  = GamepadReader()

        self.lane_detector = LaneDetector(debug=False)
        self.obj_detector  = ObjectDetector()
        self.autonomous    = AutonomousController(self.motor, self.steering)

        self._mode           = VehicleMode.STANDBY
        self._running        = True
        self._dt             = 1.0 / self.LOOP_HZ
        self._last_t         = time.monotonic()
        self._last_mode_change = 0.0   # timestamp del último cambio de modo

        signal.signal(signal.SIGINT,  self._handle_signal)
        signal.signal(signal.SIGTERM, self._handle_signal)

        print("[INIT] Listo. Esperando mando Bluetooth...")

    # ----------------------------------------------------------
    def run(self):
        self.sensor.start()
        self.camera.start()
        self.gamepad.start()
        try:
            self._main_loop()
        finally:
            self._shutdown()

    # ----------------------------------------------------------
    def _main_loop(self):
        while self._running:
            now = time.monotonic()
            self._dt     = now - self._last_t
            self._last_t = now

            gp         = self.gamepad.state
            tof        = self.sensor.distance_mm
            frame_data = self.camera.get_latest_frame()

            lane = LaneData(0, 0, False, 0, SERVO_CENTER_ANGLE)
            obj  = ObjectDetector.AnalysisResult()
            if frame_data is not None:
                lane = self.lane_detector.process(frame_data.image)
                obj  = self.obj_detector.analyze(
                    frame_data.detections, frame_data.image, tof)

            self._handle_mode_transitions(gp)

            match self._mode:
                case VehicleMode.STANDBY:
                    self._standby(gp)
                case VehicleMode.MANUAL:
                    self._manual(gp)
                case VehicleMode.VISION:
                    self._vision(lane, obj, tof)
                case VehicleMode.AUTONOMOUS | VehicleMode.PARKING:
                    self.autonomous.update(lane, obj, tof, self._dt)

            elapsed = time.monotonic() - now
            wait    = (1.0 / self.LOOP_HZ) - elapsed
            if wait > 0:
                time.sleep(wait)

    # ----------------------------------------------------------
    def _handle_mode_transitions(self, gp):
        # Sin mando → STANDBY siempre
        if not gp.connected:
            if self._mode != VehicleMode.STANDBY:
                print("\n[FSM] Mando desconectado → STANDBY")
                self._safe_stop()
                self._set_mode(VehicleMode.STANDBY)
            return

        # Cooldown para evitar cambios accidentales por botón rebotado
        if time.monotonic() - self._last_mode_change < self.MODE_COOLDOWN:
            # Vaciar colas de botones para que no se acumulen
            for btn in (BTN_MANUAL, BTN_VISION, BTN_AUTONOMOUS, BTN_PARKING):
                self.gamepad.consume_button(btn)
            return

        # ── Cuadrado / X → TOGGLE autónomo ──────────────────────
        if self.gamepad.consume_button(BTN_AUTONOMOUS):
            if self._mode == VehicleMode.AUTONOMOUS:
                # Apagar autónomo → volver a manual
                self.autonomous.deactivate()
                self._set_mode(VehicleMode.MANUAL)
            else:
                self._safe_stop()
                self._set_mode(VehicleMode.AUTONOMOUS)
                self.autonomous.activate()
            return

        # ── Círculo / B → Visión ────────────────────────────────
        if self.gamepad.consume_button(BTN_VISION):
            if self._mode != VehicleMode.VISION:
                self._safe_stop()
                self._set_mode(VehicleMode.VISION)
            else:
                # Presionar de nuevo → volver a manual
                self._set_mode(VehicleMode.MANUAL)
            return

        # ── Cruz / A → Manual ───────────────────────────────────
        if self.gamepad.consume_button(BTN_MANUAL):
            self._safe_stop()
            self._set_mode(VehicleMode.MANUAL)
            return

        # ── Triángulo / Y → Parking ─────────────────────────────
        if self.gamepad.consume_button(BTN_PARKING):
            if self._mode != VehicleMode.AUTONOMOUS:
                self._safe_stop()
                self._set_mode(VehicleMode.AUTONOMOUS)
                self.autonomous.activate()
            self.autonomous.trigger_parking()
            return

    def _set_mode(self, new_mode: str):
        if new_mode != self._mode:
            print(f"\n[FSM] {self._mode} → {new_mode}")
        self._mode = new_mode
        self._last_mode_change = time.monotonic()

    # ----------------------------------------------------------
    # STANDBY
    # ----------------------------------------------------------
    def _standby(self, gp):
        if gp.connected:
            print("[FSM] Mando conectado → MANUAL")
            self._set_mode(VehicleMode.MANUAL)
            self._set_led(PIN_LED_STATUS, True)
        else:
            self._set_led(PIN_LED_STATUS, int(time.monotonic() * 2) % 2 == 0)

    # ----------------------------------------------------------
    # MANUAL
    # ----------------------------------------------------------
    def _manual(self, gp):
        """
        Palanca izquierda X → dirección servo.
        R2 → motor adelante progresivo.
        L2 → reversa suave.
        """
        # Dirección — palanca izquierda X (eje 0)
        rango = SERVO_CENTER_ANGLE - 45   # 45°
        servo_angle = SERVO_CENTER_ANGLE + gp.steer * rango
        self.steering.set_angle(servo_angle)

        # Motor
        if gp.brake > 0.05:
            # L2 → reversa, máximo 50%
            self.motor.set_throttle(-(gp.brake ** 2) * 50)
        elif gp.throttle > 0.05:
            # R2 → adelante progresivo (cuadrático para suavidad)
            self.motor.set_throttle((gp.throttle ** 1.5) * 100)
        else:
            self.motor.brake()

    # ----------------------------------------------------------
    # VISION TEST
    # ----------------------------------------------------------
    def _vision(self, lane, obj, tof_mm):
        """Motores OFF. Imprime en terminal lo que detecta la cámara."""
        self.motor.brake()
        self.steering.center()

        stop_info = "no"
        if obj.stop_sign_detected:
            d = obj.stop_sign_distance_mm
            stop_info = f"SI {d:.0f}mm" if d else "SI ?mm"

        semaforo = obj.traffic_light.color.upper() if obj.traffic_light else "---"

        print(
            f"\r[VIS] "
            f"Carril:{lane.error_px:+6.1f}px | "
            f"Curva:{'SI' if lane.is_curve else 'no'} | "
            f"Crucero:{'SI' if lane.crosswalk_detected else 'no'} | "
            f"ToF:{str(tof_mm or '---'):>5}mm | "
            f"STOP:{stop_info:<10} | "
            f"Luz:{semaforo}",
            end="", flush=True,
        )

    # ----------------------------------------------------------
    # Helpers
    # ----------------------------------------------------------
    def _safe_stop(self):
        self.motor.brake()
        self.steering.center()
        if self._mode == VehicleMode.AUTONOMOUS:
            self.autonomous.deactivate()

    def _setup_leds(self):
        for pin in (PIN_LED_STOP, PIN_LED_STATUS):
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)

    def _set_led(self, pin: int, state):
        GPIO.output(pin, GPIO.HIGH if bool(state) else GPIO.LOW)

    def _handle_signal(self, signum, frame):
        print(f"\n[SYS] Señal {signum} → apagando...")
        self._running = False

    def _shutdown(self):
        print("\n[SYS] Apagando...")
        self._safe_stop()
        self.gamepad.stop()
        self.sensor.stop()
        self.camera.stop()
        self.motor.cleanup()
        for pin in (PIN_LED_STOP, PIN_LED_STATUS):
            GPIO.output(pin, GPIO.LOW)
        GPIO.cleanup()
        print("[SYS] Listo.")


# ----------------------------------------------------------
if __name__ == "__main__":
    CarritoTMR().run()
