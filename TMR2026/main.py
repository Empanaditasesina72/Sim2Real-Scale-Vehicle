# -*- coding: utf-8 -*-
"""
main.py — Punto de entrada del vehículo autónomo TMR 2026.
Raspberry Pi 5 · Sony IMX500 · IBT-2 · PCA9685 · VL53L0X

Arquitectura de hilos:
  CameraStream  → hilo demonio, actualiza frame BGR a 30 FPS
  SignDetector  → hilo demonio, inferencia YOLO a ~12 FPS
  DistanceSensor→ hilo demonio, polling VL53L0X a 50 Hz
  Main loop     → 50 Hz: gamepad + FSM + servo + motor

Modos:
  STANDBY    → Motor OFF, servo al centro. Espera al mando.
  MANUAL     → Palanca izq → servo | R2 → avance | L2 → reversa
  AUTONOMOUS → FSM de 5 estados (CRUCERO/PRECAUCIÓN/FRENADO/ESPERA/REANUDAR)

Botones gamepad (PS4/Xbox genérico):
  Cuadrado / X  → Toggle AUTONOMOUS (activa/desactiva)
  Cruz    / A   → MANUAL
  Círculo / B   → Modo VISION (cámara ON, motores OFF, debug)
  Start         → Emergencia: freno inmediato + MANUAL
"""

import os
import sys
import time
import signal
import threading
from typing import Optional

import cv2

# ── Flag de display (--display para abrir ventana) ────────────────────────────
_DISPLAY = "--display" in sys.argv
if _DISPLAY:
    os.environ.setdefault("DISPLAY", ":0")

# ─────────────────────────────────────────────────────────────────────────────
# CONSTANTES HARDWARE (NO MODIFICAR SIN CAMBIAR EL CABLEADO)
# ─────────────────────────────────────────────────────────────────────────────
MOTOR_RPWM        = 18      # GPIO BCM
MOTOR_LPWM        = 13      # GPIO BCM
SERVO_I2C_BUS     = 3       # Bus I2C via dtoverlay (GPIO 0=SDA, 1=SCL)
SERVO_CHANNEL     = 0       # Canal PCA9685
SERVO_MIN_PULSE   = 500     # µs → 0°
SERVO_MAX_PULSE   = 2500    # µs → 180°
SERVO_CENTER      = 90.0    # grados
SERVO_MIN         = 45.0    # límite físico izquierda
SERVO_MAX         = 135.0   # límite físico derecha
TOF_I2C_BUS       = 4       # Bus I2C via dtoverlay (GPIO 23=SDA, 22=SCL)
TOF_XSHUT_PIN     = 17      # GPIO XSHUT del VL53L0X frontal

# ─────────────────────────────────────────────────────────────────────────────
# CONSTANTES DE COMPORTAMIENTO
# ─────────────────────────────────────────────────────────────────────────────
LOOP_HZ           = 50      # Hz del bucle de control principal
CAMERA_W          = 640
CAMERA_H          = 480
CAMERA_FPS        = 30
AWB_WARMUP_S      = 2.0     # segundos estabilización AE/AWB
YOLO_MODEL        = "weights/tmr_signs.pt"
YOLO_CONF         = 0.55
YOLO_IMGSZ        = 320

# PID de dirección (error en px → corrección en grados)
PID_KP            = 0.08    # Subir si el coche oscila poco → aumentar
PID_KI            = 0.002   # Anti-windup a long-term drift
PID_KD            = 0.025   # Amortiguación → aumentar si el servo vibra
PID_OUT_MIN       = -(SERVO_CENTER - SERVO_MIN)   # -45°
PID_OUT_MAX       =  (SERVO_MAX - SERVO_CENTER)   # +45°

# Gamepad (PS4 / Xbox genérico)
BTN_AUTONOMOUS    = 2       # Cuadrado / X
BTN_MANUAL        = 0       # Cruz / A
BTN_VISION        = 1       # Círculo / B
BTN_EMERGENCY     = 9       # Start / Options
AXIS_STEER        = 0       # Palanca izquierda X  (−1=izq, +1=der)
AXIS_THROTTLE     = 5       # Gatillo R2           (−1=libre, +1=fondo)
AXIS_BRAKE        = 2       # Gatillo L2
DEADBAND          = 0.08

# ─────────────────────────────────────────────────────────────────────────────
# IMPORTAR MÓDULOS DEL PROYECTO
# ─────────────────────────────────────────────────────────────────────────────
from hardware.motor            import MotorDriver
from hardware.steering_driver  import SteeringDriver
from hardware.distance_sensor  import DistanceSensor
from hardware.signals          import TurnSignals, SignalMode
from hardware.brake_light      import BrakeLight
from vision.camera_stream      import CameraStream
from vision.lane_pipeline      import LanePipeline, LaneResult
from vision.sign_detector      import SignDetector
from control.pid_controller    import PIDController
from control.fsm               import AutonomousFSM, FSMState

from config import (
    PIN_LED_TURN_LEFT, PIN_LED_TURN_RIGHT, PIN_LED_BRAKE, SIGNAL_BLINK_HZ,
)


# ─────────────────────────────────────────────────────────────────────────────
# CLASE PRINCIPAL
# ─────────────────────────────────────────────────────────────────────────────

class VehicleTMR:
    """
    Controlador completo del vehículo TMR 2026.

    Integra hardware, visión y FSM en un único bucle de control a 50 Hz.
    El procesamiento de imagen ocurre en hilos separados — el bucle de
    control NUNCA espera a la visión.
    """

    class Mode:
        STANDBY    = "STANDBY"
        MANUAL     = "MANUAL"
        VISION     = "VISION"
        AUTONOMOUS = "AUTONOMOUS"

    MODE_COOLDOWN_S = 0.4   # mínimo entre cambios de modo (anti-rebote)

    def __init__(self):
        print("=" * 55)
        print("  TMR 2026 — Inicializando hardware...")
        print("=" * 55)

        # ── Hardware ──────────────────────────────────────────────
        self.motor    = MotorDriver(pin_rpwm=MOTOR_RPWM, pin_lpwm=MOTOR_LPWM)
        self.steering = SteeringDriver()
        self.sensor   = DistanceSensor()
        self.signals  = TurnSignals(
            pin_left  = PIN_LED_TURN_LEFT,
            pin_right = PIN_LED_TURN_RIGHT,
            blink_hz  = SIGNAL_BLINK_HZ,
        )
        self.brake_light = BrakeLight(pin=PIN_LED_BRAKE)

        # ── Visión ────────────────────────────────────────────────
        self.camera = CameraStream(
            width=CAMERA_W, height=CAMERA_H,
            fps=CAMERA_FPS, awb_warmup_s=AWB_WARMUP_S,
        )
        self.lane_pipe = LanePipeline(
            frame_w=CAMERA_W, frame_h=CAMERA_H, debug=_DISPLAY
        )
        self.sign_det = SignDetector(
            model_path=YOLO_MODEL, conf=YOLO_CONF, imgsz=YOLO_IMGSZ
        )

        # ── PID y FSM ─────────────────────────────────────────────
        self.pid = PIDController(
            kp=PID_KP, ki=PID_KI, kd=PID_KD,
            setpoint=0.0,
            output_limits=(PID_OUT_MIN, PID_OUT_MAX),
            integral_limits=(-25.0, 25.0),
        )
        self.fsm = AutonomousFSM(
            self.motor, self.steering, self.pid,
            signals     = self.signals,
            brake_light = self.brake_light,
        )

        # ── Gamepad (pygame) ──────────────────────────────────────
        self._joystick = None
        self._init_gamepad()

        # ── Estado interno ────────────────────────────────────────
        self._mode            = self.Mode.STANDBY
        self._running         = True
        self._last_mode_t     = 0.0
        self._last_lane       = LaneResult(error_px=0.0, confidence=0.0)
        self._btn_prev: dict  = {}

        signal.signal(signal.SIGINT,  self._handle_signal)
        signal.signal(signal.SIGTERM, self._handle_signal)

        print("[INIT] Hardware listo. Esperando mando Bluetooth...")

    # ─── Arranque y bucle principal ───────────────────────────────────────────

    def run(self) -> None:
        # Arrancar hilos de hardware y visión
        self.sensor.start()
        self.camera.start()
        self.sign_det.start()

        print(f"[MAIN] Bucle de control a {LOOP_HZ} Hz iniciado.")
        dt = 1.0 / LOOP_HZ
        t_last = time.monotonic()

        try:
            while self._running:
                now   = time.monotonic()
                dt    = now - t_last
                t_last = now

                self._process_gamepad()
                self._update_vision()
                self._run_mode(dt)

                # Avanzar parpadeo en CADA iteración — vale para todos los modos
                self.signals.tick()

                elapsed = time.monotonic() - now
                sleep   = max(0.0, (1.0 / LOOP_HZ) - elapsed)
                if sleep > 0:
                    time.sleep(sleep)

        finally:
            self._shutdown()

    # ─── Gamepad ─────────────────────────────────────────────────────────────

    def _init_gamepad(self) -> None:
        try:
            import pygame
            pygame.init()
            pygame.joystick.init()
            if pygame.joystick.get_count() > 0:
                self._joystick = pygame.joystick.Joystick(0)
                self._joystick.init()
                print(f"[PAD] Mando conectado: {self._joystick.get_name()}")
            else:
                print("[PAD] Sin mando detectado — solo modo autónomo disponible.")
        except Exception as e:
            print(f"[PAD] pygame no disponible: {e}")

    def _process_gamepad(self) -> None:
        if self._joystick is None:
            return
        try:
            import pygame
            pygame.event.pump()
        except Exception:
            return

        now = time.monotonic()
        if now - self._last_mode_t < self.MODE_COOLDOWN_S:
            return

        def btn(idx: int) -> bool:
            """True SOLO en el flanco de subida del botón."""
            cur = bool(self._joystick.get_button(idx))
            prev = self._btn_prev.get(idx, False)
            self._btn_prev[idx] = cur
            return cur and not prev

        # Emergencia — tiene prioridad absoluta
        if btn(BTN_EMERGENCY):
            print("[PAD] EMERGENCIA — freno + MANUAL")
            self.motor.brake()
            self.fsm.deactivate()
            self._set_mode(self.Mode.MANUAL)
            return

        # Cuadrado / X → Toggle AUTONOMOUS
        if btn(BTN_AUTONOMOUS):
            if self._mode == self.Mode.AUTONOMOUS:
                self.fsm.deactivate()
                self._set_mode(self.Mode.MANUAL)
            else:
                self.motor.brake()
                self._set_mode(self.Mode.AUTONOMOUS)
                self.fsm.activate()
            return

        # Cruz / A → MANUAL
        if btn(BTN_MANUAL):
            self.fsm.deactivate()
            self._set_mode(self.Mode.MANUAL)
            return

        # Círculo / B → VISION (debug, motores OFF)
        if btn(BTN_VISION):
            if self._mode == self.Mode.VISION:
                self._set_mode(self.Mode.MANUAL)
            else:
                self.motor.brake()
                self._set_mode(self.Mode.VISION)
            return

    def _set_mode(self, mode: str) -> None:
        if mode != self._mode:
            print(f"[MODE] {self._mode} → {mode}")
            if self._mode == self.Mode.VISION and _DISPLAY:
                cv2.destroyAllWindows()
        self._mode = mode
        self._last_mode_t = time.monotonic()

    # ─── Actualización de visión (no bloqueante) ──────────────────────────────

    def _update_vision(self) -> None:
        """
        Obtiene el frame más reciente y actualiza LanePipeline + SignDetector.
        Nunca bloquea — si no hay frame nuevo, usa el resultado anterior.
        """
        frame = self.camera.get_frame()
        if frame is None:
            return

        # Lane detection (rápido: ~10 ms)
        self._last_lane = self.lane_pipe.process(frame)

        # Proveer frame al detector de señales (el hilo YOLO lo consumirá)
        self.sign_det.update_frame(frame)

        # Actualizar entradas de la FSM
        self.fsm.lane_error   = self._last_lane.error_px
        self.fsm.lane_conf    = self._last_lane.confidence
        self.fsm.lidar_mm     = self.sensor.front_mm
        self.fsm.sign_visible = self.sign_det.has_any_sign()

        # Distancia a la señal STOP estimada por bbox (fallback si no hay lidar)
        closest = self.sign_det.closest_sign("stop_sign")
        if closest is not None and closest.distance_m is not None:
            self.fsm.sign_distance_mm = closest.distance_m * 1000.0
        else:
            self.fsm.sign_distance_mm = None

    # ─── Modos de operación ───────────────────────────────────────────────────

    def _run_mode(self, dt: float) -> None:
        match self._mode:
            case self.Mode.STANDBY:
                self._do_standby()
            case self.Mode.MANUAL:
                self._do_manual()
            case self.Mode.VISION:
                self._do_vision()
            case self.Mode.AUTONOMOUS:
                self.fsm.update(dt)
                self._log_autonomous()

    def _log_autonomous(self) -> None:
        """Línea de status del modo autónomo — incluye lo que ve YOLO."""
        dets = self.sign_det.get_detections()
        if dets:
            sign_txt = ", ".join(
                f"{d.label}@{(d.distance_m or 0)*100:.0f}cm" for d in dets[:2]
            )
        else:
            sign_txt = "—"
        lidar_txt = f"{self.sensor.front_mm:.0f}" if self.sensor.front_mm else "---"
        print(f"\r[AUT] {self.fsm.state.name:<10}  "
              f"err:{self._last_lane.error_px:+5.0f}px  "
              f"angle:{self.steering.current_angle:5.1f}°  "
              f"duty:{self.motor.current_duty:+.0f}%  "
              f"lidar:{lidar_txt}mm  signs:{sign_txt}   ",
              end="", flush=True)

    def _do_standby(self) -> None:
        self.motor.brake()
        self.steering.center()
        self.signals.set_mode(SignalMode.OFF)
        self.brake_light.off()
        if self._joystick is None:
            return
        # En STANDBY: cualquier botón activa MANUAL
        try:
            import pygame
            pygame.event.pump()
            for i in range(self._joystick.get_numbuttons()):
                if self._joystick.get_button(i):
                    self._set_mode(self.Mode.MANUAL)
                    return
        except Exception:
            pass

    def _do_manual(self) -> None:
        """Control manual: palanca izquierda X → servo | R2 → avance | L2 → reversa."""
        if self._joystick is None:
            self.motor.brake()
            return

        # Dirección — la inversión física del servo se maneja en SteeringDriver,
        # aquí trabajamos en el sistema lógico (joystick izq = ángulo < 90).
        steer_raw = self._joystick.get_axis(AXIS_STEER)
        if abs(steer_raw) < DEADBAND:
            steer_raw = 0.0
        angle = SERVO_CENTER + steer_raw * (SERVO_CENTER - SERVO_MIN)
        self.steering.set_angle(angle)

        # Direccionales según dirección del giro (umbral alto para ignorar drift)
        if   steer_raw < -0.30:
            self.signals.set_mode(SignalMode.LEFT)
        elif steer_raw > +0.30:
            self.signals.set_mode(SignalMode.RIGHT)
        else:
            self.signals.set_mode(SignalMode.OFF)

        # Velocidad
        throttle = self._joystick.get_axis(AXIS_THROTTLE)   # −1=suelto, +1=fondo
        brake    = self._joystick.get_axis(AXIS_BRAKE)      # −1=suelto, +1=fondo

        # Normalizar triggers: (valor + 1) / 2 ∈ [0, 1]
        t = (throttle + 1.0) / 2.0
        b = (brake    + 1.0) / 2.0

        if b > DEADBAND:
            # Reversa suave (máx 40%)
            self.motor.set_speed(-(b ** 2) * 40.0)
        elif t > DEADBAND:
            # Avance con soft-start implícito en MotorDriver
            self.motor.set_speed((t ** 1.3) * 55.0)   # máx 55% en manual
        else:
            self.motor.set_speed(0.0)   # rueda libre (no freno)

        # Luz de freno: encendida sólo cuando hay reversa activa
        if self.motor.current_duty < -1.0:
            self.brake_light.on()
        else:
            self.brake_light.off()

        # Log con detecciones YOLO (lo que ve el carro)
        dets = self.sign_det.get_detections()
        if dets:
            sign_txt = ", ".join(
                f"{d.label}@{(d.distance_m or 0)*100:.0f}cm" for d in dets[:2]
            )
        else:
            sign_txt = "—"

        print(f"\r[MAN] steer:{steer_raw:+.2f} ({angle:.0f}°)  "
              f"t:{t:.2f}  b:{b:.2f}  duty:{self.motor.current_duty:+.0f}%  "
              f"signs:{sign_txt}   ",
              end="", flush=True)

    def _do_vision(self) -> None:
        """Debug de visión — motores OFF, muestra pipeline en pantalla."""
        self.motor.brake()
        self.steering.center()
        self.signals.set_mode(SignalMode.OFF)
        self.brake_light.off()

        frame = self.camera.get_frame()
        if frame is None:
            return

        lane  = self._last_lane
        dets  = self.sign_det.get_detections()
        lidar = self.sensor.front_mm

        # Dibujar pipeline en consola o ventana
        if _DISPLAY:
            vis = self.lane_pipe.draw_debug(frame, lane)

            # BEV y máscara si debug está activo
            if lane.bev_frame is not None:
                small_bev  = cv2.resize(lane.bev_frame,  (320, 180))
                small_mask = cv2.resize(lane.mask_frame, (320, 180))
                vis[0:180, 0:320]   = small_bev
                vis[0:180, 320:640] = small_mask

            # Bboxes YOLO
            for d in dets:
                cv2.rectangle(vis, (d.x1, d.y1), (d.x2, d.y2), (0, 0, 255), 2)
                cv2.putText(vis, f"{d.label} {d.confidence:.0%}",
                            (d.x1, max(d.y1-6, 12)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0,0,255), 1)

            # Info
            lidar_txt = f"{lidar:.0f}mm" if lidar else "---"
            cv2.putText(vis, f"Lidar:{lidar_txt}  FPS:30",
                        (8, CAMERA_H - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 220, 0), 1)

            cv2.imshow("TMR 2026 — Vision Debug", vis)
            cv2.waitKey(1)
        else:
            sign_txt = ", ".join(f"{d.label}({d.confidence:.0%})" for d in dets) or "—"
            lidar_txt = f"{lidar:.0f}" if lidar else "---"
            print(f"\r[VIS] err:{lane.error_px:+.0f}px "
                  f"conf:{lane.confidence:.0%}  "
                  f"lidar:{lidar_txt}mm  signs:{sign_txt}   ",
                  end="", flush=True)

    # ─── Apagado limpio ───────────────────────────────────────────────────────

    def _handle_signal(self, signum, _frame) -> None:
        print(f"\n[SYS] Señal {signum} recibida → apagando...")
        self._running = False

    def _shutdown(self) -> None:
        print("\n[SYS] Apagando sistema...")
        if _DISPLAY:
            cv2.destroyAllWindows()
        self.fsm.deactivate()
        self.motor.brake()
        time.sleep(0.1)
        self.sensor.stop()
        self.sign_det.stop()
        self.camera.stop()
        try:    self.signals.close()
        except: pass
        try:    self.brake_light.close()
        except: pass
        self.motor.cleanup()
        print("[SYS] Apagado completado.")


# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    VehicleTMR().run()
