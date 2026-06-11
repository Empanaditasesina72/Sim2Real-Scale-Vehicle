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
  PARKING    → Estacionamiento en batería (SEARCH→MANEUVER→PARKED)

Botones gamepad (PS4/Xbox genérico):
  Cuadrado / X  → Toggle AUTONOMOUS (activa/desactiva)
  Cruz    / A   → MANUAL
  Círculo / B   → Modo VISION (cámara ON, motores OFF, debug)
  Triángulo / Y → Toggle PARKING (estacionamiento en batería)
  Start         → Emergencia: freno inmediato + MANUAL
"""

import os
import sys
import time
import signal
import subprocess

import cv2

# ── Flag de display (--display para abrir ventana) ────────────────────────────
_DISPLAY = "--display" in sys.argv
if _DISPLAY:
    os.environ.setdefault("DISPLAY", ":0")


# ─────────────────────────────────────────────────────────────────────────────
# Liberar GPIO del servicio systemd antes de inicializar hardware.
# Si carrito_tmr.service está activo, los pines ya están reclamados y
# RPi.GPIO/lgpio fallan con 'GPIO not allocated'. Ejecutamos `systemctl stop`
# automáticamente (passwordless sudo). Si nosotros mismos somos el servicio
# (INVOCATION_ID viene de systemd), no hacemos nada.
# ─────────────────────────────────────────────────────────────────────────────
def _release_gpio_from_systemd() -> None:
    if os.environ.get("INVOCATION_ID"):
        return
    try:
        active = subprocess.run(
            ["systemctl", "is-active", "--quiet", "carrito_tmr"],
            timeout=2,
        )
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return
    if active.returncode != 0:
        return

    print("[SYS] carrito_tmr.service activo — deteniéndolo para liberar GPIO...")
    try:
        subprocess.run(
            ["sudo", "-n", "systemctl", "stop", "carrito_tmr"],
            timeout=10,
            check=True,
        )
    except subprocess.CalledProcessError:
        print("[SYS] No pude detener el servicio (¿sudo sin NOPASSWD?).")
        print("[SYS] Ejecuta:  sudo systemctl stop carrito_tmr")
        sys.exit(1)
    except (subprocess.TimeoutExpired, FileNotFoundError) as e:
        print(f"[SYS] Error deteniendo servicio: {e}")
        sys.exit(1)
    time.sleep(0.5)  # dar tiempo al kernel a liberar los pines
    print("[SYS] Servicio detenido — GPIO libre.")

# ─────────────────────────────────────────────────────────────────────────────
# CONSTANTES HARDWARE — única fuente de verdad: config.py
# (Los buses I²C, canal PCA9685 y pines XSHUT los leen los drivers
#  directamente de config.py; aquí solo se importa lo que usa este módulo.)
# ─────────────────────────────────────────────────────────────────────────────
from config import (
    PIN_MOTOR_RPWM     as MOTOR_RPWM,     # GPIO BCM 18 — PWM avance
    PIN_MOTOR_LPWM     as MOTOR_LPWM,     # GPIO BCM 13 — PWM reversa
    SERVO_CENTER_ANGLE as SERVO_CENTER,   # 90° = ruedas al frente
    SERVO_MIN_ANGLE    as SERVO_MIN,      # tope físico izquierda (58°)
    SERVO_MAX_ANGLE    as SERVO_MAX,      # tope físico derecha  (122°)
    PIN_LED_TURN_LEFT, PIN_LED_TURN_RIGHT, PIN_LED_BRAKE, SIGNAL_BLINK_HZ,
    BTN_MANUAL, BTN_VISION, BTN_AUTONOMOUS, BTN_PARKING, BTN_EMERGENCY,
    AXIS_STEER, AXIS_THROTTLE, AXIS_BRAKE,
    JOYSTICK_DEADBAND as DEADBAND,
)

# ─────────────────────────────────────────────────────────────────────────────
# CONSTANTES DE COMPORTAMIENTO
# ─────────────────────────────────────────────────────────────────────────────
LOOP_HZ           = 50      # Hz del bucle de control principal
CAMERA_W          = 640
CAMERA_H          = 480
CAMERA_FPS        = 30
AWB_WARMUP_S      = 2.0     # segundos estabilización AE/AWB
YOLO_MODEL        = "weights/tmr_signs.pt"
YOLO_CONF         = 0.55      # paridad con el simulador validado (Sim2Real).
                              # Con 0.15 el modelo inventaba señales (verde,
                              # flechas…) y la FSM frenaba sin razón. Si la
                              # señal física no se parece al training set, el
                              # detector por COLOR de SignDetector la respalda.
YOLO_IMGSZ        = 320

# PID de dirección (error en px → corrección en grados)
PID_KP            = 0.08    # Subir si el coche oscila poco → aumentar
PID_KI            = 0.002   # Anti-windup a long-term drift
PID_KD            = 0.025   # Amortiguación → aumentar si el servo vibra
PID_OUT_MIN       = -(SERVO_CENTER - SERVO_MIN)   # −32° (tope físico real)
PID_OUT_MAX       =  (SERVO_MAX - SERVO_CENTER)   # +32°

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
from control.fsm               import AutonomousFSM
from control.parking_fsm       import ParkingFSM, ParkingState


# ─────────────────────────────────────────────────────────────────────────────
# LECTOR DE TECLADO (no bloqueante) — alternativa al gamepad
# ─────────────────────────────────────────────────────────────────────────────
class _KeyboardReader:
    """
    Lee una tecla por iteración desde stdin SIN bloquear el bucle de control.
    Pone el TTY en modo cbreak (cada tecla llega al instante, sin Enter).
    Si stdin no es TTY (e.g. ejecución bajo systemd), queda inerte.
    Restaura los atributos del terminal en close().
    """

    def __init__(self):
        self.enabled = False
        self._old_attrs = None
        try:
            import termios, tty
            if not sys.stdin.isatty():
                return
            self._old_attrs = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
            self.enabled = True
        except Exception as e:
            print(f"[KB] Teclado no disponible: {e}")

    def get_key(self):
        if not self.enabled:
            return None
        import select
        r, _, _ = select.select([sys.stdin], [], [], 0)
        if r:
            return sys.stdin.read(1)
        return None

    def close(self):
        if self._old_attrs is None:
            return
        try:
            import termios
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_attrs)
        except Exception:
            pass
        self._old_attrs = None
        self.enabled = False


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
        PARKING    = "PARKING"     # estacionamiento en batería

    # Modos que usan la ventana de debug (--display)
    DISPLAY_MODES = ("VISION", "AUTONOMOUS", "PARKING")

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
        # FSM de estacionamiento en batería (misma que validó el simulador)
        self.parking = ParkingFSM(self.motor, self.steering)

        # ── Gamepad (pygame) ──────────────────────────────────────
        self._joystick = None
        self._init_gamepad()

        # ── Teclado (alternativa sin gamepad) ─────────────────────
        self._kb = _KeyboardReader()
        if self._kb.enabled:
            print("[KB] Atajos:  A=MANUAL  B=VISION  X=AUTO  P=PARKING  "
                  "Space=EMERG  S=STANDBY  Q=salir")

        # ── Estado interno ────────────────────────────────────────
        self._mode            = self.Mode.STANDBY
        self._running         = True
        self._last_mode_t     = 0.0
        self._last_lane       = LaneResult(error_px=0.0, confidence=0.0)
        self._btn_prev: dict  = {}
        self._sign_action     = ""   # acción de la señal más cercana (telemetría)

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

                self._pump_gamepad_events()
                self._process_gamepad()
                self._poll_keyboard()
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
        """
        Inicializa el subsistema de joystick de pygame. La conexión real al
        primer mando ocurre en `_pump_gamepad_events()` (vía evento
        JOYDEVICEADDED de SDL2), así que el sistema arranca aunque el control
        esté apagado y se enganchará automáticamente cuando se prenda.
        """
        try:
            import pygame
            pygame.init()
            pygame.joystick.init()
            print("[PAD] Esperando mando — se enganchará automáticamente al prenderlo.")
        except Exception as e:
            print(f"[PAD] pygame no disponible: {e}")

    def _pump_gamepad_events(self) -> None:
        """
        Drena la cola de eventos SDL para mantener el estado del joystick
        actualizado y reaccionar a JOYDEVICEADDED/REMOVED.  Se llama una vez
        por iteración del bucle principal.
        """
        try:
            import pygame
        except Exception:
            return
        for event in pygame.event.get():
            if event.type == pygame.JOYDEVICEADDED:
                try:
                    joy = pygame.joystick.Joystick(event.device_index)
                    joy.init()
                    self._joystick = joy
                    self._btn_prev.clear()
                    print(f"[PAD] Mando conectado: {joy.get_name()}")
                except Exception as e:
                    print(f"[PAD] Error al enganchar mando: {e}")
            elif event.type == pygame.JOYDEVICEREMOVED:
                if self._joystick is not None:
                    print("[PAD] Mando desconectado — esperando reconexión...")
                self._joystick = None
                self._btn_prev.clear()

    def _process_gamepad(self) -> None:
        if self._joystick is None:
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

        # Triángulo / Y → Toggle PARKING (estacionamiento en batería)
        if btn(BTN_PARKING):
            if self._mode == self.Mode.PARKING:
                self._set_mode(self.Mode.MANUAL)
            else:
                self.fsm.deactivate()
                self.motor.brake()
                self._set_mode(self.Mode.PARKING)
                self.parking.activate()
            return

    # ─── Teclado (espejo del gamepad) ─────────────────────────────────────────

    def _poll_keyboard(self) -> None:
        if not self._kb.enabled:
            return
        key = self._kb.get_key()
        if key:
            self._process_key(key)

    def _process_key(self, key: str) -> None:
        """
        A=MANUAL  B=VISION  X=AUTO (toggle)  P=PARKING (toggle)
        Space=EMERG  S=STANDBY  Q=salir
        Espacio y Q ignoran el cooldown (parada inmediata).
        """
        k = key.lower()

        if k == "q":
            print("\n[KB] Salida solicitada.")
            self._running = False
            return

        if key == " ":
            print("\n[KB] EMERGENCIA — freno + MANUAL")
            self.motor.brake()
            self.fsm.deactivate()
            self._set_mode(self.Mode.MANUAL)
            return

        if time.monotonic() - self._last_mode_t < self.MODE_COOLDOWN_S:
            return

        if k == "a":
            print("\n[KB] -> MANUAL")
            self.fsm.deactivate()
            self._set_mode(self.Mode.MANUAL)
        elif k == "b":
            if self._mode == self.Mode.VISION:
                print("\n[KB] VISION -> MANUAL")
                self._set_mode(self.Mode.MANUAL)
            else:
                print("\n[KB] -> VISION (preview cámara)")
                self.motor.brake()
                self._set_mode(self.Mode.VISION)
        elif k == "x":
            if self._mode == self.Mode.AUTONOMOUS:
                print("\n[KB] AUTONOMOUS -> MANUAL")
                self.fsm.deactivate()
                self._set_mode(self.Mode.MANUAL)
            else:
                print("\n[KB] -> AUTONOMOUS")
                self.motor.brake()
                self._set_mode(self.Mode.AUTONOMOUS)
                self.fsm.activate()
        elif k == "p":
            if self._mode == self.Mode.PARKING:
                print("\n[KB] PARKING -> MANUAL")
                self._set_mode(self.Mode.MANUAL)
            else:
                print("\n[KB] -> PARKING (estacionamiento en bateria)")
                self.fsm.deactivate()
                self.motor.brake()
                self._set_mode(self.Mode.PARKING)
                self.parking.activate()
        elif k == "s":
            print("\n[KB] -> STANDBY")
            self.fsm.deactivate()
            self.motor.brake()
            self._set_mode(self.Mode.STANDBY)

    def _set_mode(self, mode: str) -> None:
        if mode != self._mode:
            print(f"[MODE] {self._mode} → {mode}")
            # Al salir de PARKING por cualquier vía (botón, teclado,
            # emergencia) la maniobra se cancela y el coche queda frenado.
            if self._mode == self.Mode.PARKING:
                self.parking.deactivate()
            # Cerrar la ventana de debug al salir de un modo que la usa
            # (VISION/AUTONOMOUS/PARKING) hacia uno que no (STANDBY/MANUAL).
            if (_DISPLAY and self._mode in self.DISPLAY_MODES
                    and mode not in self.DISPLAY_MODES):
                cv2.destroyAllWindows()
            # VISION usa el PID solo para preview — al entrar/salir lo
            # reseteamos para que el integrador no se contamine entre modos.
            if mode == self.Mode.VISION or self._mode == self.Mode.VISION:
                self.pid.reset()
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

        # Señales que OBLIGAN a frenar: STOP y semáforo en ROJO.
        # (green/yellow/left/right/straight NO frenan — con has_any_sign()
        #  el carro frenaba hasta con el semáforo en VERDE. Misma lógica
        #  que validó el simulador Sim2Real.)
        stop_like = (self.sign_det.has_sign("stop_sign")
                     or self.sign_det.has_sign("red"))
        self.fsm.sign_visible = stop_like

        # Distancia a la señal de frenado más cercana (STOP o rojo),
        # estimada por bbox — fallback si no hay lidar frontal.
        closest = (self.sign_det.closest_sign("stop_sign")
                   or self.sign_det.closest_sign("red"))
        if closest is not None and closest.distance_m is not None:
            self.fsm.sign_distance_mm = closest.distance_m * 1000.0
        else:
            self.fsm.sign_distance_mm = None

        # Acción según la señal detectada (para overlay + log).
        self._sign_action = self._decide_sign_action()

    # Acciones por tipo de señal (lo que el carro "hace" al verla).
    SIGN_ACTIONS = {
        "stop_sign": "ALTO total (5 s)",
        "red":       "Semaforo ROJO: frenar",
        "green":     "Semaforo VERDE: avanzar",
        "yellow":    "Semaforo AMARILLO: precaucion",
        "left":      "Flecha IZQUIERDA",
        "right":     "Flecha DERECHA",
        "straight":  "Seguir RECTO",
    }

    def _decide_sign_action(self) -> str:
        """Devuelve el texto de acción de la señal más cercana detectada."""
        dets = self.sign_det.get_detections()
        if not dets:
            return ""
        with_dist = [d for d in dets if d.distance_m is not None]
        if not with_dist:
            return self.SIGN_ACTIONS.get(dets[0].label, "")
        closest = min(with_dist, key=lambda d: d.distance_m)
        return self.SIGN_ACTIONS.get(closest.label, closest.label)

    # ─── Modos de operación ───────────────────────────────────────────────────

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
                self._log_autonomous()
                if _DISPLAY:
                    self._render_debug_view(mode_label="AUT")
            case self.Mode.PARKING:
                self._do_parking(dt)

    def _do_parking(self, dt: float) -> None:
        """Estacionamiento en batería — misma ParkingFSM validada en el sim."""
        self.parking.lidar_mm = self.sensor.front_mm
        self.parking.update(dt)

        # Luces: hazard durante toda la maniobra; freno encendido al quedar
        # estacionado (coche detenido = misma señalización que ESPERA).
        self.signals.set_mode(SignalMode.HAZARD)
        if self.parking.state == ParkingState.PARKED:
            self.brake_light.on()
        else:
            self.brake_light.off()

        print(f"\r[PARK] {self.parking.state.name:<16} "
              f"duty:{self.motor.current_duty:+.0f}%  "
              f"angle:{self.steering.current_angle:5.1f}°   ",
              end="", flush=True)
        if _DISPLAY:
            self._render_debug_view(mode_label="PARK")

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
        action = self._sign_action or "—"
        print(f"\r[AUT] {self.fsm.state.name:<10}  "
              f"err:{self._last_lane.error_px:+5.0f}px  "
              f"angle:{self.steering.current_angle:5.1f}°  "
              f"duty:{self.motor.current_duty:+.0f}%  "
              f"lidar:{lidar_txt}mm  signs:{sign_txt}  -> {action}   ",
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

    def _do_vision(self, dt: float) -> None:
        """Debug de visión — motores OFF, muestra pipeline + PID en pantalla."""
        self.motor.brake()
        self.steering.center()
        self.signals.set_mode(SignalMode.OFF)
        self.brake_light.off()

        if self.camera.get_frame() is None:
            return

        # PID — solo simulación. El servo NO se mueve; ya quedó en center().
        # En AUTONOMOUS la FSM calcula el PID; aquí lo hacemos a mano para
        # poder visualizarlo igualmente.
        self.pid.compute(self._last_lane.error_px, dt)

        if _DISPLAY:
            self._render_debug_view(mode_label="VIS")

        # Log a consola siempre (con --display y sin él).
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

    # ─── Render compartido (VISION + AUTONOMOUS + PARKING) ────────────────────

    def _render_debug_view(self, mode_label: str) -> None:
        """
        Dibuja una sola ventana con TODO el debug visual:
          • Frame anotado con la línea central del carril detectado.
          • Mosaico superior: BEV (vista cenital) + máscara HSV.
          • Bounding boxes de YOLO con etiqueta + confianza + distancia.
          • Panel inferior izquierdo: valores PID (P/I/D/corr) y error.
          • Panel inferior derecho: objetos detectados + acción de la señal.

        Llamado desde VISION, AUTONOMOUS y PARKING cuando --display está
        activo. No bloquea: cv2.waitKey(1).
        """
        frame = self.camera.get_frame()
        if frame is None:
            return

        lane  = self._last_lane
        dets  = self.sign_det.get_detections()
        lidar = self.sensor.front_mm

        # Frame con línea central del carril dibujada por el pipeline.
        vis = self.lane_pipe.draw_debug(frame, lane)

        # ── Mosaico BEV + máscara (ojo de águila + filtro) ──
        if lane.bev_frame is not None and lane.mask_frame is not None:
            vis[0:180, 0:320]   = cv2.resize(lane.bev_frame,  (320, 180))
            vis[0:180, 320:640] = cv2.resize(lane.mask_frame, (320, 180))
            cv2.putText(vis, "BEV (ojo de aguila)", (8, 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            cv2.putText(vis, "Mascara HSV blanco",   (328, 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

        angle_target = max(
            SERVO_MIN, min(SERVO_MAX, SERVO_CENTER + self.pid.last_output)
        )

        # ── Panel inferior izquierdo: PID + error ──
        self._draw_panel(vis, x=8, y=200, w=320, h=160, lines=[
            f"MODO  : {mode_label}",
            f"err   :{lane.error_px:+7.1f}px  conf:{lane.confidence:.0%}",
            f"P     :{self.pid.last_p:+7.2f}   kp={self.pid.kp:.3f}",
            f"I     :{self.pid.last_i:+7.2f}   ki={self.pid.ki:.3f}",
            f"D     :{self.pid.last_d:+7.2f}   kd={self.pid.kd:.3f}",
            f"corr  :{self.pid.last_output:+7.2f}d -> servo {angle_target:5.1f}d",
            f"lidar :{lidar:.0f}mm" if lidar else "lidar :---",
        ])

        # ── Panel inferior derecho: objetos detectados + acción ──
        if dets:
            obj_lines = ["OBJETOS DETECTADOS:"]
            for d in dets[:4]:
                dist = f" @{(d.distance_m or 0)*100:.0f}cm" if d.distance_m else ""
                obj_lines.append(f"- {d.label}  {d.confidence:.0%}{dist}")
            if self._sign_action:
                obj_lines.append(f"-> {self._sign_action}")
        else:
            obj_lines = ["OBJETOS DETECTADOS:", "- (ninguno)"]
        self._draw_panel(vis, x=336, y=200, w=296, h=160, lines=obj_lines)

        # ── Estado FSM en la barra inferior (conducción o parking) ──
        try:
            if self._mode == self.Mode.PARKING:
                fsm_txt = f"FSM:{self.parking.state.name}"
            else:
                fsm_txt = f"FSM:{self.fsm.state.name}"
        except Exception:
            fsm_txt = ""
        cv2.putText(vis, f"{mode_label}  {fsm_txt}  duty:{self.motor.current_duty:+.0f}%",
                    (8, CAMERA_H - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 255), 2, cv2.LINE_AA)

        # ── Bounding boxes YOLO (AL FINAL: siempre encima de mosaico + paneles) ──
        for d in dets:
            cv2.rectangle(vis, (d.x1, d.y1), (d.x2, d.y2), (0, 255, 0), 2)
            dist_txt = f" {(d.distance_m or 0)*100:.0f}cm" if d.distance_m else ""
            label_txt = f"{d.label} {d.confidence:.0%}{dist_txt}"
            (tw, th), _ = cv2.getTextSize(label_txt, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            ly = max(d.y1 - 6, th + 4)
            cv2.rectangle(vis, (d.x1, ly - th - 4), (d.x1 + tw + 4, ly + 2),
                          (0, 0, 0), -1)
            cv2.putText(vis, label_txt, (d.x1 + 2, ly - 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

        cv2.imshow("TMR 2026 - Vision Debug", vis)
        cv2.waitKey(1)

    @staticmethod
    def _draw_panel(img, x: int, y: int, w: int, h: int, lines: list[str]) -> None:
        """Caja semitransparente con texto multi-línea."""
        ov = img.copy()
        cv2.rectangle(ov, (x, y), (x + w, y + h), (0, 0, 0), -1)
        cv2.addWeighted(ov, 0.55, img, 0.45, 0, dst=img)
        cv2.rectangle(img, (x, y), (x + w, y + h), (255, 220, 0), 1)
        for i, line in enumerate(lines):
            cv2.putText(img, line, (x + 8, y + 20 + i * 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 220, 0), 1, cv2.LINE_AA)

    # ─── Apagado limpio ───────────────────────────────────────────────────────

    def _handle_signal(self, signum, _frame) -> None:
        print(f"\n[SYS] Señal {signum} recibida → apagando...")
        self._running = False

    def _shutdown(self) -> None:
        print("\n[SYS] Apagando sistema...")
        try:    self._kb.close()
        except: pass
        if _DISPLAY:
            cv2.destroyAllWindows()
        self.fsm.deactivate()
        self.parking.deactivate()
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
    _release_gpio_from_systemd()
    VehicleTMR().run()
