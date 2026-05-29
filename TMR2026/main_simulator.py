# -*- coding: utf-8 -*-
"""
main_simulator.py — Punto de entrada del TMR 2026 en PC (Sim2Real validación)

Versión adaptada de main.py para ejecutar en PC conectado a simulador Unity.
Reemplaza:
  - MotorDriver         → MockMotorDriver (vía SimulatorClient)
  - SteeringDriver      → MockSteeringDriver (vía SimulatorClient)
  - DistanceSensor      → MockDistanceSensor (vía SimulatorClient)
  - CameraStream        → MockCameraStream (vía SimulatorClient)
  - GPIO signals/brake  → NoOp versions (no hardware físico)

Uso:
  python main_simulator.py               # PC mode, sin display
  python main_simulator.py --display     # PC mode con ventana de debug

El simulador Unity debe estar escuchando en 127.0.0.1:5005 y enviando:
  - Frames JPEG (4-byte size header + JPEG data)
  - Mensajes TOF: "TOF:front_mm,rear_mm"

La arquitectura sigue siendo multihilo con 50 Hz main loop.
"""

import os
import sys
import time
import signal
import threading
from typing import Optional

# Forzar UTF-8 en la consola (Windows usa cp1252 y crashea con ✓ → etc.)
try:
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")
except Exception:
    pass

import cv2
import numpy as np

# ── Flags de linea de comandos ────────────────────────────────────────────────
#   --display  → abre ventana de debug (cámara + carril + señales)
#   --standby  → arranca quieto (por defecto arranca en AUTONOMOUS y se mueve)
_DISPLAY = "--display" in sys.argv
_START_STANDBY = "--standby" in sys.argv
if _DISPLAY:
    os.environ.setdefault("DISPLAY", ":0")

# ─────────────────────────────────────────────────────────────────────────────
# CONSTANTES COMPORTAMIENTO
# ─────────────────────────────────────────────────────────────────────────────
LOOP_HZ           = 50      # Hz del bucle de control principal
CAMERA_W          = 640
CAMERA_H          = 480
CAMERA_FPS        = 30
SERVO_CENTER      = 90.0    # grados
SERVO_MIN         = 45.0    # límite físico izquierda
SERVO_MAX         = 135.0   # límite físico derecha
PID_KP            = 0.08
PID_KI            = 0.002
PID_KD            = 0.025
PID_OUT_MIN       = -(SERVO_CENTER - SERVO_MIN)
PID_OUT_MAX       =  (SERVO_MAX - SERVO_CENTER)

# ── Simulador ──
SIMULATOR_HOST    = "127.0.0.1"
SIMULATOR_PORT    = 5005

# ─────────────────────────────────────────────────────────────────────────────
# IMPORTAR MÓDULOS DEL PROYECTO
# ─────────────────────────────────────────────────────────────────────────────
from sim_hardware_mocks import SimulatorClient
from vision.lane_pipeline import LanePipeline, LaneResult
from vision.sign_detector import SignDetector
from control.pid_controller import PIDController
from control.fsm import AutonomousFSM, FSMState

# Placeholder para mocks de GPIO (no-op en PC)
class NoOpSignals:
    """Mock de TurnSignals para PC."""
    class SignalMode:
        OFF = 0
        LEFT = 1
        RIGHT = 2
        HAZARD = 3

    def __init__(self, *args, **kwargs):
        pass

    def set_mode(self, mode):
        pass

    def tick(self):
        pass

    def close(self):
        pass


class NoOpBrakeLight:
    """Mock de BrakeLight para PC."""
    def __init__(self, *args, **kwargs):
        pass

    def on(self):
        pass

    def off(self):
        pass

    def close(self):
        pass


# ─────────────────────────────────────────────────────────────────────────────
# ENVOLTORIO DE CÁMARA SIMULADO
# ─────────────────────────────────────────────────────────────────────────────

class SimulatedCameraWrapper:
    """
    Envuelve MockCameraStream para proporcionar interfaz compatible con
    vision/lane_pipeline.py (que espera get_frame() que retorna un array BGR).
    """

    def __init__(self, mock_camera):
        self.mock_camera = mock_camera
        self._lock = threading.Lock()

    def get_frame(self) -> Optional[np.ndarray]:
        """Retorna el último frame recibido (BGR)."""
        with self._lock:
            return self.mock_camera.get_latest_frame()

    def start(self):
        """No-op: MockCameraStream ya está corriendo en hilos."""
        pass

    def stop(self):
        """No-op."""
        pass


# ─────────────────────────────────────────────────────────────────────────────
# CLASE PRINCIPAL
# ─────────────────────────────────────────────────────────────────────────────

class VehicleSimulator:
    """
    Controlador del TMR 2026 en modo simulación (PC).
    Idéntico a VehicleTMR salvo que usa SimulatorClient en lugar de hardware Pi.
    """

    class Mode:
        STANDBY    = "STANDBY"
        MANUAL     = "MANUAL"
        VISION     = "VISION"
        AUTONOMOUS = "AUTONOMOUS"

    MODE_COOLDOWN_S = 0.4

    def __init__(self):
        print("=" * 70)
        print("  TMR 2026 — SIMULATOR MODE (PC)")
        print("=" * 70)

        # ── Conectar al simulador ──────────────────────────────────────────
        try:
            self.sim = SimulatorClient(host=SIMULATOR_HOST, port=SIMULATOR_PORT)
        except Exception as e:
            print(f"[SIM] ERROR: No puedo conectar al simulador en {SIMULATOR_HOST}:{SIMULATOR_PORT}")
            print(f"[SIM] ¿Unity está corriendo?")
            raise

        # ── Usar interfaces SimulatorClient ────────────────────────────────
        self.motor    = self.sim.motor
        self.steering = self.sim.steering
        self.sensor   = self.sim.distance
        self.camera_raw = self.sim.camera

        # ── Wrapper compatível ─────────────────────────────────────────────
        self.camera = SimulatedCameraWrapper(self.camera_raw)

        # ── Mocks de GPIO (no-op) ─────────────────────────────────────────
        self.signals    = NoOpSignals()
        self.brake_light = NoOpBrakeLight()

        # ── Visión ─────────────────────────────────────────────────────────
        self.lane_pipe = LanePipeline(
            frame_w=CAMERA_W, frame_h=CAMERA_H, debug=_DISPLAY
        )
        self.sign_det = SignDetector(
            model_path="weights/tmr_signs.pt",
            conf=0.55,
            imgsz=320
        )

        # ── PID y FSM ──────────────────────────────────────────────────────
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

        # ── Estado interno ────────────────────────────────────────────────
        self._mode            = self.Mode.STANDBY
        self._running         = True
        self._last_lane       = LaneResult(error_px=0.0, confidence=0.0)
        self._start_time      = time.monotonic()

        # ── Metrics para Phase 1 validation ────────────────────────────────
        self._metrics = {
            "frame_count": 0,
            "loop_count": 0,
            "errors_px": [],
            "pid_outputs": [],
            "servo_angles": [],
            "motor_duties": [],
            "lidar_readings": [],
            "fsm_states": [],
        }

        signal.signal(signal.SIGINT,  self._handle_signal)
        signal.signal(signal.SIGTERM, self._handle_signal)

        print("[INIT] Conexión al simulador establecida.")
        print("[INIT] Iniciando visión y FSM...")

    def run(self) -> None:
        """Bucle de control principal a 50 Hz."""
        # Arrancar SignDetector (corre en su propio hilo)
        self.sign_det.start()

        # Por defecto arranca en AUTONOMOUS (el carro se mueve solo).
        # Con --standby se queda quieto esperando set_mode().
        if not _START_STANDBY:
            self._mode = self.Mode.AUTONOMOUS
            self.fsm.activate()
            print("[MAIN] Modo AUTONOMOUS activado (el carro va a manejar solo).")
            if _DISPLAY:
                print("[MAIN] Teclas en la ventana:  A=Autonomo  S=Stop  Q=Salir")

        print(f"[MAIN] Bucle de control a {LOOP_HZ} Hz iniciado.")
        dt = 1.0 / LOOP_HZ
        t_last = time.monotonic()

        try:
            while self._running:
                now   = time.monotonic()
                dt    = now - t_last
                t_last = now

                self._update_vision()
                self._run_mode(dt)

                # Avanzar blink (si los LEDs estuvieran conectados)
                self.signals.tick()

                elapsed = time.monotonic() - now
                sleep   = max(0.0, (1.0 / LOOP_HZ) - elapsed)
                if sleep > 0:
                    time.sleep(sleep)

                self._metrics["loop_count"] += 1

        finally:
            self._shutdown()

    # ─── Actualización de visión ───────────────────────────────────────────

    def _update_vision(self) -> None:
        """Procesa frame + actualiza FSM con detecciones."""
        frame = self.camera.get_frame()
        if frame is None:
            return

        # Lane detection
        self._last_lane = self.lane_pipe.process(frame)
        self._metrics["frame_count"] += 1
        self._metrics["errors_px"].append(self._last_lane.error_px)

        # Sign detection
        self.sign_det.update_frame(frame)

        # Actualizar FSM
        self.fsm.lane_error   = self._last_lane.error_px
        self.fsm.lane_conf    = self._last_lane.confidence
        self.fsm.lidar_mm     = self.sensor.front_mm
        self.fsm.sign_visible = self.sign_det.has_any_sign()

        if self.sensor.front_mm is not None:
            self._metrics["lidar_readings"].append(self.sensor.front_mm)

        # Distancia a STOP
        closest = self.sign_det.closest_sign("stop_sign")
        if closest is not None and closest.distance_m is not None:
            self.fsm.sign_distance_mm = closest.distance_m * 1000.0

    # ─── Modos de operación ───────────────────────────────────────────────

    def _run_mode(self, dt: float) -> None:
        match self._mode:
            case self.Mode.STANDBY:
                self._do_standby()
            case self.Mode.MANUAL:
                self._do_manual()
            case self.Mode.VISION:
                self._do_vision(dt)
            case self.Mode.AUTONOMOUS:
                self.fsm.update(dt)
                self._metrics["fsm_states"].append(self.fsm.state.name)
                self._metrics["pid_outputs"].append(self.pid.last_output)
                self._metrics["servo_angles"].append(self.steering.current_angle)
                self._metrics["motor_duties"].append(self.motor.current_duty)
                self._log_autonomous()
                if _DISPLAY:
                    self._render_debug_view(mode_label="AUT")

    def _log_autonomous(self) -> None:
        """Status line para modo autónomo."""
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
        """Espera a comando automático (para test)."""
        self.motor.brake()
        self.steering.set_angle(SERVO_CENTER)
        print(f"\r[STANDBY] Esperando comando... ("
              f"ejecuta: sim.set_mode(VehicleSimulator.Mode.AUTONOMOUS))  ",
              end="", flush=True)

    def _do_manual(self) -> None:
        """Modo manual — en PC requiere API, no gamepad."""
        self.motor.brake()
        self.steering.set_angle(SERVO_CENTER)
        print(f"\r[MANUAL] No disponible en simulador (requiere gamepad). "
              f"Cambiando a AUTONOMOUS...", end="", flush=True)
        self._mode = self.Mode.AUTONOMOUS
        self.fsm.activate()

    def _do_vision(self, dt: float) -> None:
        """Modo debug de visión — PID en simulación, motores OFF."""
        self.motor.brake()
        self.steering.set_angle(SERVO_CENTER)

        if self.camera.get_frame() is None:
            return

        self.pid.compute(self._last_lane.error_px, dt)

        if _DISPLAY:
            self._render_debug_view(mode_label="VIS")

        dets = self.sign_det.get_detections()
        sign_txt = ", ".join(f"{d.label}({d.confidence:.0%})" for d in dets) or "—"
        lidar_txt = f"{self.sensor.front_mm:.0f}" if self.sensor.front_mm else "---"
        angle_target = max(
            SERVO_MIN, min(SERVO_MAX, SERVO_CENTER + self.pid.last_output)
        )
        print(f"\r[VIS] err:{self._last_lane.error_px:+.0f}px "
              f"conf:{self._last_lane.confidence:.0%}  "
              f"P:{self.pid.last_p:+5.2f} I:{self.pid.last_i:+5.2f} "
              f"D:{self.pid.last_d:+5.2f}  corr:{self.pid.last_output:+5.2f}d "
              f"angle:{angle_target:5.1f}d  lidar:{lidar_txt}mm  "
              f"signs:{sign_txt}   ",
              end="", flush=True)

    # ─── Render compartido (VISION + AUTONOMOUS) ──────────────────────────

    def _render_debug_view(self, mode_label: str) -> None:
        """Panel de debug multi-información."""
        frame = self.camera.get_frame()
        if frame is None:
            return

        lane  = self._last_lane
        dets  = self.sign_det.get_detections()
        lidar = self.sensor.front_mm

        # Frame con línea central del carril
        vis = self.lane_pipe.draw_debug(frame, lane)

        # ── Mosaico BEV + máscara ──
        if lane.bev_frame is not None and lane.mask_frame is not None:
            vis[0:180, 0:320]   = cv2.resize(lane.bev_frame,  (320, 180))
            vis[0:180, 320:640] = cv2.resize(lane.mask_frame, (320, 180))
            cv2.putText(vis, "BEV", (8, 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            cv2.putText(vis, "HSV Mask", (328, 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

        # ── Bounding boxes YOLO ──
        for d in dets:
            cv2.rectangle(vis, (d.x1, d.y1), (d.x2, d.y2), (0, 0, 255), 2)
            dist_txt = f" {(d.distance_m or 0)*100:.0f}cm" if d.distance_m else ""
            cv2.putText(vis, f"{d.label} {d.confidence:.0%}{dist_txt}",
                        (d.x1, max(d.y1 - 6, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1, cv2.LINE_AA)

        angle_target = max(
            SERVO_MIN, min(SERVO_MAX, SERVO_CENTER + self.pid.last_output)
        )

        # ── Panel PID ──
        self._draw_panel(vis, x=8, y=200, w=320, h=160, lines=[
            f"MODO  : {mode_label}",
            f"err   :{lane.error_px:+7.1f}px  conf:{lane.confidence:.0%}",
            f"P     :{self.pid.last_p:+7.2f}   kp={self.pid.kp:.3f}",
            f"I     :{self.pid.last_i:+7.2f}   ki={self.pid.ki:.3f}",
            f"D     :{self.pid.last_d:+7.2f}   kd={self.pid.kd:.3f}",
            f"corr  :{self.pid.last_output:+7.2f}d -> {angle_target:5.1f}d",
            f"lidar :{lidar:.0f}mm" if lidar else "lidar :---",
        ])

        # ── Panel objetos ──
        if dets:
            obj_lines = ["OBJETOS:"]
            for d in dets[:5]:
                dist = f" @{(d.distance_m or 0)*100:.0f}cm" if d.distance_m else ""
                obj_lines.append(f"- {d.label} {d.confidence:.0%}{dist}")
        else:
            obj_lines = ["OBJETOS:", "- (ninguno)"]
        self._draw_panel(vis, x=336, y=200, w=296, h=160, lines=obj_lines)

        try:    fsm_txt = f"FSM:{self.fsm.state.name}"
        except: fsm_txt = ""
        cv2.putText(vis, f"{mode_label}  {fsm_txt}  duty:{self.motor.current_duty:+.0f}%",
                    (8, CAMERA_H - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 255), 2, cv2.LINE_AA)

        cv2.imshow("TMR 2026 - Simulator Debug", vis)
        # Teclas en la ventana: A=Autonomo  S=Stop  Q=Salir
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self._running = False
        elif key == ord('a'):
            if self._mode != self.Mode.AUTONOMOUS:
                self._mode = self.Mode.AUTONOMOUS
                self.fsm.activate()
        elif key == ord('s'):
            self._mode = self.Mode.STANDBY
            self.fsm.deactivate()

    @staticmethod
    def _draw_panel(img, x: int, y: int, w: int, h: int, lines: list[str]) -> None:
        """Caja de debug con texto."""
        ov = img.copy()
        cv2.rectangle(ov, (x, y), (x + w, y + h), (0, 0, 0), -1)
        cv2.addWeighted(ov, 0.55, img, 0.45, 0, dst=img)
        cv2.rectangle(img, (x, y), (x + w, y + h), (255, 220, 0), 1)
        for i, line in enumerate(lines):
            cv2.putText(img, line, (x + 8, y + 20 + i * 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 220, 0), 1, cv2.LINE_AA)

    # ─── Métodos de control programático ───────────────────────────────────

    def set_mode(self, mode: str) -> None:
        """Cambiar modo de operación."""
        if mode != self._mode:
            print(f"\n[MODE] {self._mode} → {mode}")
            if _DISPLAY and self._mode in (self.Mode.VISION, self.Mode.AUTONOMOUS):
                if mode not in (self.Mode.VISION, self.Mode.AUTONOMOUS):
                    cv2.destroyAllWindows()
            if mode == self.Mode.VISION or self._mode == self.Mode.VISION:
                self.pid.reset()
            if mode == self.Mode.AUTONOMOUS:
                self.fsm.activate()
            else:
                self.fsm.deactivate()
        self._mode = mode

    def run_autonomous_test(self, duration_s: float) -> dict:
        """
        Ejecuta modo AUTONOMOUS durante N segundos y retorna métricas.
        Útil para validación Phase 1.
        """
        print(f"\n[TEST] Iniciando test autónomo ({duration_s}s)...")
        self.set_mode(self.Mode.AUTONOMOUS)
        start = time.monotonic()

        while time.monotonic() - start < duration_s and self._running:
            time.sleep(0.001)

        self.set_mode(self.Mode.STANDBY)
        return self.get_metrics()

    def get_metrics(self) -> dict:
        """Retorna métricas colectadas para análisis."""
        elapsed = time.monotonic() - self._start_time
        return {
            "elapsed_s": elapsed,
            "loop_count": self._metrics["loop_count"],
            "frame_count": self._metrics["frame_count"],
            "avg_loop_hz": self._metrics["loop_count"] / elapsed if elapsed > 0 else 0,
            "errors_px": self._metrics["errors_px"],
            "pid_outputs": self._metrics["pid_outputs"],
            "servo_angles": self._metrics["servo_angles"],
            "motor_duties": self._metrics["motor_duties"],
            "lidar_readings": self._metrics["lidar_readings"],
            "fsm_states": self._metrics["fsm_states"],
        }

    # ─── Apagado limpio ───────────────────────────────────────────────────

    def _handle_signal(self, signum, _frame) -> None:
        print(f"\n[SYS] Señal {signum} recibida → apagando...")
        self._running = False

    def _shutdown(self) -> None:
        print("\n[SYS] Apagando simulador...")
        if _DISPLAY:
            cv2.destroyAllWindows()
        self.fsm.deactivate()
        self.motor.brake()
        time.sleep(0.1)
        self.sign_det.stop()
        self.sim.close()
        print("[SYS] Apagado completado.")
        
        # Guardar métricas a CSV
        metrics = self.get_metrics()
        print(f"\n[METRICS] Loop ejecutado {metrics['loop_count']} veces en {metrics['elapsed_s']:.1f}s")
        print(f"[METRICS] Frames procesados: {metrics['frame_count']}")
        print(f"[METRICS] Lecturas lidar: {len(metrics['lidar_readings'])}")


# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    sim = VehicleSimulator()
    try:
        sim.run()
    except KeyboardInterrupt:
        print("\n[USER] Interrumpido por usuario.")
    except Exception as e:
        print(f"\n[ERROR] {e}")
        import traceback
        traceback.print_exc()
