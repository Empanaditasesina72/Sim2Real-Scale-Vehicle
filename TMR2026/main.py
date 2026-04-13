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

import os
import sys
import time
import signal
import argparse

import cv2
import lgpio

# --display : abre ventana de cámara en el HDMI de la Pi
#   Uso: python3 main.py --display
_ap = argparse.ArgumentParser()
_ap.add_argument("--display", action="store_true",
                 help="Mostrar ventana de cámara en el monitor de la Pi")
_args, _ = _ap.parse_known_args()

_HAS_DISPLAY = _args.display
if _HAS_DISPLAY:
    os.environ.setdefault("DISPLAY", ":0")

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
                    self._vision(lane, obj, tof, frame_data)
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
            if self._mode == VehicleMode.VISION and _HAS_DISPLAY:
                cv2.destroyAllWindows()
        self._mode = new_mode
        self._last_mode_change = time.monotonic()

    # ----------------------------------------------------------
    # STANDBY
    # ----------------------------------------------------------
    def _standby(self, gp):
        if not gp.connected:
            self._set_led(PIN_LED_STATUS, int(time.monotonic() * 2) % 2 == 0)
            return

        # Motor desactivado — nunca moverse en STANDBY
        self.motor.disable()

        # Mando conectado — LED parpadeo lento, espera que el usuario elija modo
        self._set_led(PIN_LED_STATUS, int(time.monotonic()) % 2 == 0)
        print("\r[STANDBY] Cruz=Manual  Círculo=Cámara  Cuadrado=Autónomo   ", end="", flush=True)

        if self.gamepad.consume_button(BTN_MANUAL):
            print()
            self._set_mode(VehicleMode.MANUAL)
            self._set_led(PIN_LED_STATUS, True)
        elif self.gamepad.consume_button(BTN_VISION):
            print()
            self._set_mode(VehicleMode.VISION)
        elif self.gamepad.consume_button(BTN_AUTONOMOUS):
            print()
            self._safe_stop()
            self._set_mode(VehicleMode.AUTONOMOUS)
            self.autonomous.activate()

    # ----------------------------------------------------------
    # MANUAL
    # ----------------------------------------------------------
    def _manual(self, gp):
        """
        Palanca izquierda X → dirección servo.
        R2 → motor adelante progresivo con rampa suave.
        L2 → reversa suave.
        """
        # ── Dirección — palanca izquierda X (eje 0) ──────────────
        rango = SERVO_CENTER_ANGLE - 45   # ±45° desde centro
        servo_angle = SERVO_CENTER_ANGLE + gp.steer * rango
        self.steering.set_angle(servo_angle)

        # Debug: muestra steer y ángulo en terminal para verificar servo
        print(f"\r[MAN] steer:{gp.steer:+.2f} servo:{servo_angle:.0f}°  "
              f"R2:{gp.throttle:.2f} L2:{gp.brake:.2f}   ", end="", flush=True)

        # ── Motor ─────────────────────────────────────────────────
        if gp.brake > 0.05:
            # L2 → reversa (funciona bien, sin cambios)
            duty = max((gp.brake ** 2) * 50, 25.0)
            self.motor.set_throttle(-duty)

        elif gp.throttle > 0.05:
            # R2 → rampa muy suave: 2%/tick (50 Hz → 150 ms para llegar a 10%)
            # Máximo 60% con R2 a fondo para no saturar la batería
            target = max((gp.throttle ** 1.5) * 60, 10.0)
            current = abs(self.motor.duty)
            ramped  = min(current + 2.0, target)
            self.motor.set_throttle(ramped)

        else:
            self.motor.disable()

    # ----------------------------------------------------------
    # VISION TEST
    # ----------------------------------------------------------
    def _vision(self, lane, obj, tof_mm, frame_data):
        """
        Motores OFF. Muestra la cámara con anotaciones en tiempo real:
          - Bounding boxes de objetos detectados (STOP, semáforo, persona, auto)
          - Línea de centro del carril detectado
          - Estado del semáforo, distancia ToF y confianza del carril
        Presiona Círculo de nuevo para volver a Manual.
        """
        self.motor.brake()
        self.steering.center()

        if frame_data is None:
            print("\r[VIS] Esperando frame de cámara...", end="", flush=True)
            return

        vis = frame_data.image.copy()
        H, W = vis.shape[:2]

        # ── Bounding boxes de detecciones ────────────────────────
        COLOR_MAP = {
            "STOP":    (0,   0,   255),
            "SEMAFORO":(0,   200, 255),
            "PERSONA": (255, 100, 0  ),
            "AUTO":    (200, 0,   200),
        }
        for det in frame_data.detections:
            color = COLOR_MAP.get(det.label, (180, 180, 180))
            # Ajustar color del semáforo según su estado
            if det.label == "SEMAFORO" and obj.traffic_light:
                lc = obj.traffic_light.color
                if lc == "red":    color = (0,   0,   255)
                elif lc == "green":color = (0,   255, 0  )
                elif lc == "yellow":color = (0,  220, 220)
            cv2.rectangle(vis, (det.x1, det.y1), (det.x2, det.y2), color, 2)
            label_txt = f"{det.label} {det.confidence:.0%}"
            cv2.putText(vis, label_txt, (det.x1, max(det.y1 - 6, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

        # ── Centro del carril ─────────────────────────────────────
        lane_cx = W // 2 + int(lane.error_px)
        lane_cx = max(0, min(W - 1, lane_cx))
        # Línea de referencia central (amarillo tenue)
        cv2.line(vis, (W // 2, H), (W // 2, H // 2), (0, 200, 200), 1)
        # Línea de centro del carril detectado (verde si OK, rojo si perdido)
        lane_color = (0, 255, 0) if lane.confidence >= 0.30 else (0, 0, 255)
        cv2.line(vis, (lane_cx, H), (lane_cx, H // 2), lane_color, 2)

        # ── Overlay de texto ──────────────────────────────────────
        def put(text, y, color=(255, 255, 255)):
            cv2.putText(vis, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, color, 2, cv2.LINE_AA)

        conf_col = (0, 255, 0) if lane.confidence >= 0.30 else (0, 80, 255)
        put(f"Carril: {lane.error_px:+.0f}px  conf:{lane.confidence:.0%}"
            + ("  CURVA" if lane.is_curve else ""),
            28, conf_col)

        tof_txt = f"{tof_mm:.0f} mm" if tof_mm else "--- mm"
        put(f"ToF: {tof_txt}", 56, (255, 220, 0))

        if obj.stop_sign_detected:
            d = obj.stop_sign_distance_mm
            d_txt = f"{d:.0f} mm" if d else "? mm"
            put(f"STOP: {d_txt}", 84, (0, 80, 255))

        if obj.traffic_light:
            lc = obj.traffic_light.color.upper()
            lc_col = {"RED":(0,0,255),"GREEN":(0,255,0),"YELLOW":(0,220,220)}.get(lc,(200,200,200))
            put(f"Semaforo: {lc}", 112, lc_col)

        if lane.crosswalk_detected:
            put("CRUCERO DETECTADO", 140, (0, 180, 255))

        if _HAS_DISPLAY:
            cv2.imshow("TMR2026 - Vision", vis)
            cv2.waitKey(1)
        else:
            # Sin pantalla: resumen en terminal
            semaforo = obj.traffic_light.color.upper() if obj.traffic_light else "---"
            stop_txt = f"STOP {obj.stop_sign_distance_mm:.0f}mm" if obj.stop_sign_detected else "no"
            conf_txt = f"{lane.confidence:.0%}"
            print(f"\r[VIS] Carril:{lane.error_px:+.0f}px conf:{conf_txt} | "
                  f"ToF:{tof_mm or '---'}mm | STOP:{stop_txt} | Luz:{semaforo}   ",
                  end="", flush=True)

    # ----------------------------------------------------------
    # Helpers
    # ----------------------------------------------------------
    def _safe_stop(self):
        self.motor.disable()   # 0%/0% — sin pulso de corriente al IBT-2
        self.steering.center()
        if self._mode == VehicleMode.AUTONOMOUS:
            self.autonomous.deactivate()

    def _setup_leds(self):
        try:
            self._led_h = lgpio.gpiochip_open(4)
            for pin in (PIN_LED_STOP, PIN_LED_STATUS):
                lgpio.gpio_claim_output(self._led_h, pin, 0, 0)
            self._leds_ok = True
        except Exception as e:
            print(f"[WARN] LEDs no disponibles (GPIO): {e}")
            self._led_h  = None
            self._leds_ok = False

    def _set_led(self, pin: int, state):
        if not self._leds_ok or self._led_h is None:
            return
        try:
            lgpio.gpio_write(self._led_h, pin, 1 if bool(state) else 0)
        except Exception:
            pass

    def _handle_signal(self, signum, frame):
        print(f"\n[SYS] Señal {signum} → apagando...")
        self._running = False

    def _shutdown(self):
        print("\n[SYS] Apagando...")
        if _HAS_DISPLAY:
            cv2.destroyAllWindows()
        self._safe_stop()
        self.gamepad.stop()
        self.sensor.stop()
        self.camera.stop()
        self.motor.cleanup()
        if self._leds_ok and self._led_h is not None:
            for pin in (PIN_LED_STOP, PIN_LED_STATUS):
                try:
                    lgpio.gpio_write(self._led_h, pin, 0)
                    lgpio.gpio_free(self._led_h, pin)
                except Exception:
                    pass
            lgpio.gpiochip_close(self._led_h)
        print("[SYS] Listo.")


# ----------------------------------------------------------
if __name__ == "__main__":
    CarritoTMR().run()
