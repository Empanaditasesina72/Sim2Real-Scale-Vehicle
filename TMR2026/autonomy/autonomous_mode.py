"""TMR 2026 autonomous controller.

States:
  LANE_FOLLOWING    -> normal lane following
  APPROACHING_STOP  -> STOP sign detected, progressive braking
  STOPPED_WAIT      -> stopped at STOP, counting 5 s
  RESUMING          -> gradual acceleration after the pause
  CROSSWALK_STOP    -> crosswalk detected, 3 s pause
  CROSSWALK_RESUME  -> resuming after the crosswalk
  PARKING           -> delegates to ParkingManeuver
  OBSTACLE_HOLD     -> unexpected obstacle via ToF

Distance to the STOP sign:
  Priority 1: front VL53L0X (if available)
  Priority 2: estimate from the bbox size in the image (always available)
  bbox formula: d = (real_height * focal) / height_px
"""

import time
from enum import Enum, auto
from typing import Optional

import lgpio as _lgpio_mod

from config import (
    SPEED_STRAIGHT, SPEED_CURVE, SPEED_APPROACH,
    STOP_BRAKE_START_MM, STOP_TARGET_MM, STOP_TOLERANCE_MM,
    STOP_WAIT_SEC, STOP_LED_BLINK_HZ,
    EMERGENCY_STOP_MM,
    STEER_KP, STEER_KI, STEER_KD,
    VEL_STOP_KP, VEL_STOP_KI, VEL_STOP_KD,
    SERVO_CENTER_ANGLE, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE,
    PIN_LED_STOP, PIN_LED_STATUS,
    CROSSWALK_STOP_SEC,
    STOP_SIGN_REAL_HEIGHT_M, CAMERA_FOCAL_LENGTH_PX,
    LANE_MIN_CONFIDENCE,
    OVERTAKE_LEFT_SEC, OVERTAKE_PASS_SEC, OVERTAKE_RETURN_SEC, OVERTAKE_STEER_DEG,
)
from control.pid_controller import PIDController
from vision.lane_detector import LaneData
from vision.object_detector import ObjectDetector
from autonomy.parking_maneuver import ParkingManeuver, ParkingState


class AutoState(Enum):
    LANE_FOLLOWING    = auto()
    APPROACHING_STOP  = auto()
    STOPPED_WAIT      = auto()
    RESUMING          = auto()
    CROSSWALK_STOP    = auto()
    CROSSWALK_RESUME  = auto()
    PARKING           = auto()
    OBSTACLE_HOLD     = auto()
    OVERTAKING_LEFT   = auto()
    OVERTAKING_PASS   = auto()
    OVERTAKING_RETURN = auto()


class AutonomousController:

    def __init__(self, motor, steering):
        self.motor    = motor
        self.steering = steering

        self._steer_pid = PIDController(
            kp=STEER_KP, ki=STEER_KI, kd=STEER_KD,
            setpoint=0.0,
            output_limits=(-(SERVO_CENTER_ANGLE - 45), (SERVO_CENTER_ANGLE - 45)),
            integral_limits=(-20.0, 20.0),
        )
        self._stop_pid = PIDController(
            kp=VEL_STOP_KP, ki=VEL_STOP_KI, kd=VEL_STOP_KD,
            setpoint=STOP_TARGET_MM,
            output_limits=(0.0, SPEED_APPROACH),
            integral_limits=(-10.0, 10.0),
        )

        self._parking = ParkingManeuver(gap_side="right")
        self._state   = AutoState.LANE_FOLLOWING

        self._stop_wait_start: float  = 0.0
        self._resume_start: float     = 0.0
        self._crosswalk_start: float  = 0.0
        self._overtake_start: float   = 0.0
        self._led_last_toggle: float  = 0.0
        self._led_on: bool            = False

        self._last_stop_bbox: Optional[tuple] = None

    def activate(self):
        self._state = AutoState.LANE_FOLLOWING
        self._steer_pid.reset()
        self._stop_pid.reset()
        self._set_led(PIN_LED_STATUS, True)
        self._set_led(PIN_LED_STOP,   False)
        print("[AUTO] Activado")

    def deactivate(self):
        self.motor.brake()
        self.steering.center()
        self._set_led(PIN_LED_STATUS, False)
        self._set_led(PIN_LED_STOP,   False)
        print("[AUTO] Desactivado")

    def trigger_parking(self):
        if self._state == AutoState.LANE_FOLLOWING:
            self._parking.reset()
            self._parking.start()
            self._transition(AutoState.PARKING)

    @property
    def current_state(self) -> AutoState:
        return self._state

    def update(
        self,
        lane: LaneData,
        obj_result: ObjectDetector.AnalysisResult,
        tof_mm: Optional[float],
        dt: float,
    ):
        """Llamar una vez por ciclo del bucle principal."""

        stop_dist_mm = self._resolve_stop_distance(obj_result, tof_mm)

        if (self._state not in (AutoState.PARKING,)
                and tof_mm is not None
                and tof_mm < EMERGENCY_STOP_MM):
            self._transition(AutoState.OBSTACLE_HOLD)

        match self._state:
            case AutoState.LANE_FOLLOWING:
                self._do_lane_following(lane, obj_result, stop_dist_mm, dt)

            case AutoState.APPROACHING_STOP:
                self._do_approaching_stop(lane, stop_dist_mm, dt)

            case AutoState.STOPPED_WAIT:
                self._do_stopped_wait()

            case AutoState.RESUMING:
                self._do_resuming(lane, dt)

            case AutoState.CROSSWALK_STOP:
                self._do_crosswalk_stop()

            case AutoState.CROSSWALK_RESUME:
                self._do_crosswalk_resume(lane, dt)

            case AutoState.PARKING:
                ps = self._parking.update(tof_mm, self.motor, self.steering, obj_result)
                if ps in (ParkingState.PARKED, ParkingState.ABORTED):
                    self._transition(AutoState.LANE_FOLLOWING)

            case AutoState.OBSTACLE_HOLD:
                self.motor.brake()
                if tof_mm is None or tof_mm >= EMERGENCY_STOP_MM + 50:
                    self._transition(AutoState.LANE_FOLLOWING)

            case AutoState.OVERTAKING_LEFT:
                self._do_overtaking_left(dt)

            case AutoState.OVERTAKING_PASS:
                self._do_overtaking_pass(lane, dt)

            case AutoState.OVERTAKING_RETURN:
                self._do_overtaking_return(lane, dt)

    def _do_lane_following(self, lane, obj_result, stop_dist_mm, dt):
        if lane.confidence < LANE_MIN_CONFIDENCE:
            self.motor.brake()
            self.steering.center()
            print("\r[AUTO] No track -- waiting for the lane...        ", end="", flush=True)
            return

        if obj_result.car_in_lane:
            print("\n[AUTO] Obstacle in lane -> starting overtake")
            self._transition(AutoState.OVERTAKING_LEFT)
            return

        if lane.crosswalk_detected:
            self._transition(AutoState.CROSSWALK_STOP)
            return

        if stop_dist_mm is not None and stop_dist_mm < STOP_BRAKE_START_MM:
            self._transition(AutoState.APPROACHING_STOP)
            return

        if (obj_result.traffic_light is not None
                and obj_result.traffic_light.color == "red"):
            self.motor.brake()
            self._apply_steering(lane, dt)
            return

        self._apply_steering(lane, dt)
        self.motor.set_throttle(lane.suggested_speed)

    def _do_approaching_stop(self, lane, stop_dist_mm, dt):
        """Frena progresivamente hasta quedar a ≤30 cm de la señal."""
        if stop_dist_mm is None:
            self.motor.set_throttle(SPEED_APPROACH * 0.4)
            self._apply_steering(lane, dt)
            return

        error = stop_dist_mm - STOP_TARGET_MM

        if error <= STOP_TOLERANCE_MM:
            self.motor.brake()
            self._transition(AutoState.STOPPED_WAIT)
            return

        speed = self._stop_pid.compute(stop_dist_mm, dt)
        self.motor.set_throttle(max(speed, 5.0))
        self._apply_steering(lane, dt)

    def _do_stopped_wait(self):
        """5 segundos parado con LED parpadeante."""
        self.motor.brake()
        self.steering.center()

        now = time.monotonic()
        blink_interval = 1.0 / (2 * STOP_LED_BLINK_HZ)
        if now - self._led_last_toggle >= blink_interval:
            self._led_on = not self._led_on
            self._set_led(PIN_LED_STOP, self._led_on)
            self._led_last_toggle = now

        if now - self._stop_wait_start >= STOP_WAIT_SEC:
            self._set_led(PIN_LED_STOP, False)
            self._transition(AutoState.RESUMING)
            print("[AUTO] Resuming after STOP.")

    def _do_resuming(self, lane, dt):
        """Aceleración gradual tras STOP."""
        RAMP_TIME = 1.5
        t = min((time.monotonic() - self._resume_start) / RAMP_TIME, 1.0)
        speed = SPEED_CURVE + t * (SPEED_STRAIGHT - SPEED_CURVE)
        self._apply_steering(lane, dt)
        self.motor.set_throttle(speed)
        if t >= 1.0:
            self._transition(AutoState.LANE_FOLLOWING)

    def _do_crosswalk_stop(self):
        """Para en el crucero peatonal y espera CROSSWALK_STOP_SEC."""
        self.motor.brake()
        self.steering.center()
        if time.monotonic() - self._crosswalk_start >= CROSSWALK_STOP_SEC:
            self._transition(AutoState.CROSSWALK_RESUME)
            print("[AUTO] Resuming after crosswalk.")

    def _do_crosswalk_resume(self, lane, dt):
        """Acelera suavemente tras el crucero."""
        RAMP_TIME = 1.0
        t = min((time.monotonic() - self._resume_start) / RAMP_TIME, 1.0)
        speed = SPEED_CURVE + t * (SPEED_STRAIGHT - SPEED_CURVE)
        self._apply_steering(lane, dt)
        self.motor.set_throttle(speed)
        if t >= 1.0:
            self._transition(AutoState.LANE_FOLLOWING)

    def _do_overtaking_left(self, dt):
        """
        Fase 1: giro hacia el carril contrario (izquierda).
        Dura OVERTAKE_LEFT_SEC segundos con ángulo fijo.
        """
        elapsed = time.monotonic() - self._overtake_start
        steer_angle = SERVO_CENTER_ANGLE - OVERTAKE_STEER_DEG
        self.steering.set_angle(steer_angle)
        self.motor.set_throttle(SPEED_CURVE)
        print(f"\r[OVERTAKE] Left {elapsed:.1f}/{OVERTAKE_LEFT_SEC:.1f}s   ", end="", flush=True)
        if elapsed >= OVERTAKE_LEFT_SEC:
            self._transition(AutoState.OVERTAKING_PASS)

    def _do_overtaking_pass(self, lane, dt):
        """
        Fase 2: avanza recto pasando el obstáculo.
        Usa el error de carril para mantenerse en el carril contrario.
        """
        elapsed = time.monotonic() - self._overtake_start
        self._apply_steering(lane, dt)
        self.motor.set_throttle(SPEED_CURVE)
        print(f"\r[OVERTAKE] Passing  {elapsed:.1f}/{OVERTAKE_PASS_SEC:.1f}s   ", end="", flush=True)
        if elapsed >= OVERTAKE_PASS_SEC:
            self._transition(AutoState.OVERTAKING_RETURN)

    def _do_overtaking_return(self, lane, dt):
        """
        Fase 3: regresa al carril propio (derecha).
        Dura OVERTAKE_RETURN_SEC segundos, luego vuelve a LANE_FOLLOWING.
        """
        elapsed = time.monotonic() - self._overtake_start
        steer_angle = SERVO_CENTER_ANGLE + OVERTAKE_STEER_DEG
        self.steering.set_angle(steer_angle)
        self.motor.set_throttle(SPEED_CURVE)
        print(f"\r[OVERTAKE] Return  {elapsed:.1f}/{OVERTAKE_RETURN_SEC:.1f}s  ", end="", flush=True)
        if elapsed >= OVERTAKE_RETURN_SEC:
            print()
            self._transition(AutoState.LANE_FOLLOWING)

    def _resolve_stop_distance(
        self,
        obj_result: ObjectDetector.AnalysisResult,
        tof_mm: Optional[float],
    ) -> Optional[float]:
        """
        Devuelve la mejor estimación de distancia a la señal STOP.
        Prioridad: ToF (preciso) > bbox (siempre disponible).
        """
        if not obj_result.stop_sign_detected:
            self._last_stop_bbox = None
            return None

        if tof_mm is not None and tof_mm < 1000:
            return tof_mm

        bbox = obj_result.stop_sign_bbox
        if bbox:
            self._last_stop_bbox = bbox
            h_px = bbox[3] - bbox[1]
            if h_px > 5:
                dist_m = (STOP_SIGN_REAL_HEIGHT_M * CAMERA_FOCAL_LENGTH_PX) / h_px
                return dist_m * 1000

        return None

    def _apply_steering(self, lane: LaneData, dt: float):
        correction = self._steer_pid.compute(lane.error_px, dt)
        self.steering.set_angle(SERVO_CENTER_ANGLE + correction)

    def _transition(self, new_state: AutoState):
        print(f"[AUTO] {self._state.name} → {new_state.name}")
        self._state = new_state
        now = time.monotonic()

        if new_state == AutoState.STOPPED_WAIT:
            self._stop_wait_start = now
            self._steer_pid.reset()

        elif new_state == AutoState.RESUMING:
            self._resume_start = now
            self._stop_pid.reset()

        elif new_state == AutoState.CROSSWALK_STOP:
            self._crosswalk_start = now
            self.motor.brake()

        elif new_state == AutoState.CROSSWALK_RESUME:
            self._resume_start = now

        elif new_state == AutoState.LANE_FOLLOWING:
            self._steer_pid.reset()

        elif new_state in (AutoState.OVERTAKING_LEFT,
                           AutoState.OVERTAKING_PASS,
                           AutoState.OVERTAKING_RETURN):
            self._overtake_start = now

    @staticmethod
    def _set_led(pin: int, state: bool):
        pass
