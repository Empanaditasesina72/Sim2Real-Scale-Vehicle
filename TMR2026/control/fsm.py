# -*- coding: utf-8 -*-
"""
fsm.py — Máquina de Estados Finitos autónoma TMR 2026 (5 estados).

Estados y transiciones:
  ┌─────────────┐  YOLO ve señal   ┌────────────────┐
  │   CRUCERO   │ ─────────────── ▶│  PRECAUCIÓN    │
  │  vel=MAX    │                  │  vel=20%       │
  └─────────────┘                  └────────────────┘
        ▲                                  │ lidar ≤ 30 cm
        │                                  ▼
  ┌─────────────┐  ramp completo   ┌────────────────┐
  │  REANUDAR   │ ◀─────────────── │    FRENADO     │
  │  soft-start │                  │  motor = 0     │
  └─────────────┘                  └────────────────┘
        ▲                                  │ inmediato
        │ 5.0 s exactos                    ▼
        └───────────────────── ┌────────────────────┐
                               │      ESPERA        │
                               │  motor = 0  5 s   │
                               └────────────────────┘

Garantías importantes:
  • FRENADO → motor.brake() = duty EXACTAMENTE 0 (sin residual de 5%).
  • ESPERA usa time.monotonic() — no bloquea el hilo de visión.
  • REANUDAR aplica soft-start propio (no depende de la rampa del motor).
  • Cooldown de 3 s tras REANUDAR → ignora la misma señal al rebasarla.
  • La dirección (servo) se actualiza en TODOS los estados sin excepción.
"""

import time
from enum import Enum, auto
from typing import Optional

try:
    from hardware.signals import SignalMode
except Exception:
    # Fallback para que el módulo sea importable en PC de dev sin hardware
    class SignalMode:
        OFF = "OFF"; LEFT = "LEFT"; RIGHT = "RIGHT"; HAZARD = "HAZARD"

try:
    from config import LANE_MIN_CONFIDENCE as _CFG_LANE_MIN_CONF
except ImportError:
    _CFG_LANE_MIN_CONF = 0.20


class FSMState(Enum):
    CRUCERO    = auto()   # Avance normal, PID de dirección
    PRECAUCION = auto()   # Señal detectada a lo lejos — reducir velocidad
    FRENADO    = auto()   # Lidar ≤ 30 cm — motor exactamente a 0
    ESPERA     = auto()   # Parado exacto, contando 5.0 s
    REANUDAR   = auto()   # Soft-start para retomar crucero


class AutonomousFSM:
    """
    Controlador autónomo TMR 2026 — Máquina de Estados Finitos.

    Requiere MotorDriver, SteeringDriver y PIDController.

    Uso::

        fsm = AutonomousFSM(motor, steering, pid)
        fsm.activate()
        while running:
            fsm.lane_error   = lane_result.error_px
            fsm.lane_conf    = lane_result.confidence
            fsm.lidar_mm     = sensor.front_mm
            fsm.sign_visible = sign_detector.has_any_sign()
            fsm.update(dt)   # llamar a 50 Hz
        fsm.deactivate()
    """

    # ── Parámetros de velocidad ───────────────────────────────────────────────
    MAX_AUTO_PWM    = 42.0    # % PWM máximo en modo autónomo (anti voltage-sag)
    PRECAUCION_PWM  = 20.0    # % PWM al detectar señal lejana
    RESUME_STEP_PWM = 1.5     # % por tick al reanudar (50 Hz → ~28 s para llegar a 42%)
    # ^^ soft-start de REANUDAR es intencionalmente lento: tiempo para rebasar la señal

    # ── Umbrales ─────────────────────────────────────────────────────────────
    LIDAR_STOP_MM   = 300     # mm — distancia para pasar de PRECAUCIÓN → FRENADO
    ESPERA_S        = 5.0     # segundos de parada obligatoria (reglamento TMR)
    COOLDOWN_S      = 3.0     # segundos tras REANUDAR en que se ignoran señales
    MIN_LANE_CONF   = _CFG_LANE_MIN_CONF  # confianza mínima carril (config.py: 0.20)

    # ── Ángulo servo ─────────────────────────────────────────────────────────
    SERVO_CENTER    = 90.0
    SERVO_MIN       = 45.0
    SERVO_MAX       = 135.0

    # Umbral en grados desde el centro para activar direccional en CRUCERO/REANUDAR.
    # < SIGNAL_DIR_THRESH_DEG → off (ruedas casi rectas, no vale la pena parpadear).
    SIGNAL_DIR_THRESH_DEG = 12.0

    # Umbral de distancia (mm) calculado por bbox para confirmar parada.
    # Se usa SOLO si el lidar no da lectura (fallback). El valor real al
    # quedar quieto suele ser ~30-50 mm menor por la inercia del soft-cut.
    # Con 320 mm aquí y la calibración pinhole nueva (señal de ~4 cm) el
    # carro queda dentro del rango 240–300 mm del reglamento TMR (270±30).
    SIGN_BBOX_STOP_MM = 320

    def __init__(self, motor, steering, pid, signals=None, brake_light=None):
        """
        Parameters
        ----------
        motor       : MotorDriver
        steering    : SteeringDriver
        pid         : PIDController (setpoint=0, output=ángulo corrección en grados)
        signals     : TurnSignals (opcional) — direccionales / hazard
        brake_light : BrakeLight (opcional) — luz de freno
        """
        self.motor       = motor
        self.steering    = steering
        self.pid         = pid
        self.signals     = signals
        self.brake_light = brake_light

        # ── Entradas (actualizar desde el bucle principal antes de update()) ──
        self.lane_error:      float           = 0.0   # px — del LanePipeline
        self.lane_conf:       float           = 0.0   # [0,1]
        self.lidar_mm:        Optional[float] = None  # mm — del VL53L0X
        self.sign_visible:    bool            = False # del SignDetector (histéresis OK)
        self.sign_distance_mm:Optional[float] = None  # mm — estimado por bbox

        # ── Estado interno ────────────────────────────────────────────────────
        self._state          = FSMState.CRUCERO
        self._espera_start   = 0.0
        self._cooldown_until = 0.0   # timestamp hasta el que ignorar señales
        self._resume_speed   = 0.0   # speed actual en REANUDAR

        self._active = False

    # ─── Ciclo de vida ────────────────────────────────────────────────────────

    def activate(self) -> None:
        """Activa el modo autónomo."""
        self.pid.reset()
        self._state        = FSMState.CRUCERO
        self._resume_speed = 0.0
        self._active       = True
        self._apply_lights()
        print("[FSM] Modo autónomo ACTIVADO")

    def deactivate(self) -> None:
        """Frena y desactiva el modo autónomo."""
        self._active = False
        self.motor.brake()
        self.steering.center()
        if self.signals is not None:
            self.signals.set_mode(SignalMode.OFF)
        if self.brake_light is not None:
            self.brake_light.off()
        print("[FSM] Modo autónomo DESACTIVADO")

    @property
    def state(self) -> FSMState:
        return self._state

    # ─── Tick principal (50 Hz) ───────────────────────────────────────────────

    def update(self, dt: float) -> None:
        """
        Ejecutar UNA VEZ por ciclo del bucle principal (50 Hz recomendado).
        dt: tiempo transcurrido en segundos desde la última llamada.

        SIEMPRE actualiza el servo, aunque el motor esté parado.
        """
        if not self._active:
            # Incluso desactivado, avanzamos el parpadeo para no congelar los LEDs
            if self.signals is not None:
                self.signals.tick()
            return

        # ── 1. Dirección — siempre, en TODOS los estados ──────────────────
        self._apply_steering(dt)

        # ── 2. Máquina de estados ──────────────────────────────────────────
        match self._state:
            case FSMState.CRUCERO:
                self._do_crucero()
            case FSMState.PRECAUCION:
                self._do_precaucion()
            case FSMState.FRENADO:
                self._do_frenado()
            case FSMState.ESPERA:
                self._do_espera()
            case FSMState.REANUDAR:
                self._do_reanudar()

        # ── 3. Refrescar luces según estado + dirección actual ──────────────
        self._apply_lights()

        # ── 4. Avanzar parpadeo de direccionales (no bloquea) ──────────────
        if self.signals is not None:
            self.signals.tick()

    # ─── Sub-estados ──────────────────────────────────────────────────────────

    def _do_crucero(self) -> None:
        # Sin pista visible → frenar y esperar (no avanzar a ciegas)
        if self.lane_conf < self.MIN_LANE_CONF:
            self.motor.brake()
            return

        # Señal detectada + cooldown expirado → PRECAUCIÓN
        if self.sign_visible and time.monotonic() >= self._cooldown_until:
            self._transition(FSMState.PRECAUCION)
            return

        self.motor.set_speed(self.MAX_AUTO_PWM)

    def _do_precaucion(self) -> None:
        # Señal desapareció antes de frenar → volver a CRUCERO
        if not self.sign_visible:
            self._transition(FSMState.CRUCERO)
            return

        # Frenar si CUALQUIERA confirma cercanía:
        #   • Lidar frontal ≤ 30 cm (si está cableado), O
        #   • Distancia estimada por la cámara (bbox) ≤ umbral.
        # En el simulador el lidar apunta al centro y la señal va al lado,
        # así que el lidar nunca baja: la cámara (bbox) es la que frena.
        # En el Pi real (swithout ToF frontal) pasa lo mismo → mismo camino.
        lidar_close = (self.lidar_mm is not None
                       and self.lidar_mm <= self.LIDAR_STOP_MM)
        bbox_close  = (self.sign_distance_mm is not None
                       and self.sign_distance_mm <= self.SIGN_BBOX_STOP_MM)

        if lidar_close or bbox_close:
            origen = "lidar" if lidar_close else f"camara {self.sign_distance_mm:.0f}mm"
            print(f"[FSM] Frenando ({origen})")
            self._transition(FSMState.FRENADO)
            return

        self.motor.set_speed(self.PRECAUCION_PWM)

    def _do_frenado(self) -> None:
        # Corte EXACTO a 0 — sin residual, sin rampa
        self.motor.brake()
        # Pasar inmediatamente a ESPERA (el motor ya está en 0)
        self._transition(FSMState.ESPERA)

    def _do_espera(self) -> None:
        # Mantener motor EXACTAMENTE en 0 (brake reafirma el corte)
        self.motor.brake()

        elapsed = time.monotonic() - self._espera_start
        if elapsed >= self.ESPERA_S:
            self._transition(FSMState.REANUDAR)
            print(f"[FSM] ESPERA completada ({elapsed:.2f} s) → REANUDAR")

    def _do_reanudar(self) -> None:
        # Soft-start propio (independiente del hilo de rampa del motor)
        self._resume_speed = min(
            self._resume_speed + self.RESUME_STEP_PWM,
            self.MAX_AUTO_PWM,
        )
        self.motor.set_speed(self._resume_speed)

        # Rampa completa → CRUCERO
        if self._resume_speed >= self.MAX_AUTO_PWM:
            self._transition(FSMState.CRUCERO)

    # ─── Helpers ──────────────────────────────────────────────────────────────

    def _apply_steering(self, dt: float) -> None:
        """
        Calcula la corrección PID y la aplica al servo.
        Funciona en todos los estados — incluso con el motor parado el
        coche debe apuntar en la dirección correcta.
        """
        if self.lane_conf >= self.MIN_LANE_CONF:
            correction = self.pid.compute(self.lane_error, dt)
        else:
            correction = 0.0   # sin pista → al frente

        angle = self.SERVO_CENTER + correction
        angle = max(self.SERVO_MIN, min(self.SERVO_MAX, angle))
        self.steering.set_angle(angle)

    def _transition(self, new_state: FSMState) -> None:
        old = self._state
        self._state = new_state
        print(f"[FSM] {old.name} → {new_state.name}")

        if new_state == FSMState.FRENADO:
            self.motor.brake()   # Hard cut inmediato

        elif new_state == FSMState.ESPERA:
            self._espera_start = time.monotonic()
            self.motor.brake()

        elif new_state == FSMState.REANUDAR:
            self._resume_speed   = 0.0
            # Cooldown: ignorar señales durante COOLDOWN_S para rebasarla
            self._cooldown_until = time.monotonic() + self.COOLDOWN_S
            print(f"[FSM] Cooldown activo por {self.COOLDOWN_S:.1f} s")
            self.pid.reset()

        elif new_state == FSMState.CRUCERO:
            self.pid.reset()

        # Actualizar luces (direccionales + freno) según el nuevo estado
        self._apply_lights()

    # ─── Luces según estado ───────────────────────────────────────────────────
    def _apply_lights(self) -> None:
        """
        Mapeo estado → luces (se llama en cada tick para que las direccionales
        sigan al ángulo del servo en CRUCERO/REANUDAR):
          CRUCERO    → signals LEFT/RIGHT/OFF según ángulo,  brake OFF
          PRECAUCION → signals HAZARD,                       brake OFF
          FRENADO    → signals HAZARD,                       brake ON
          ESPERA     → signals HAZARD,                       brake ON
          REANUDAR   → signals LEFT/RIGHT/OFF según ángulo,  brake OFF
        """
        if self.signals is not None:
            if self._state in (FSMState.PRECAUCION, FSMState.FRENADO, FSMState.ESPERA):
                self.signals.set_mode(SignalMode.HAZARD)
            elif self._state in (FSMState.CRUCERO, FSMState.REANUDAR):
                deviation = self.steering.current_angle - self.SERVO_CENTER
                if   deviation < -self.SIGNAL_DIR_THRESH_DEG:
                    self.signals.set_mode(SignalMode.LEFT)
                elif deviation > +self.SIGNAL_DIR_THRESH_DEG:
                    self.signals.set_mode(SignalMode.RIGHT)
                else:
                    self.signals.set_mode(SignalMode.OFF)
            else:
                self.signals.set_mode(SignalMode.OFF)

        if self.brake_light is not None:
            if self._state in (FSMState.FRENADO, FSMState.ESPERA):
                self.brake_light.on()
            else:
                self.brake_light.off()
