"""State sub-machine for perpendicular (battery) parking.

TMR 2026 challenge:
  A 60 cm gap between two static cars.
  The car must enter perpendicularly in reverse (battery parking).

Ackermann kinematics used:
  Turning radius: R = L / tan(delta)  (bicycle model)
  where:
    L     = WHEELBASE (distance between axles)
    delta = central steering angle (servo - 90 deg)

Planned manoeuvre (from a position aligned with the gap):
  SEARCHING          -> drive slowly; the side VL53L0X detects a gap >= 60 cm.
        | gap detected
  POSITIONING        -> drive a bit more to align the rear axle with the gap.
        | forward time elapsed
  REVERSING_LOCK     -> reverse with maximum Ackermann steering into the gap;
                        the curved path tucks the rear into the space.
        | arc time elapsed
  REVERSING_STRAIGHT -> straighten the wheels and keep reversing until centred.
        | time elapsed
  PARKED             -> motor stop; signals completion to the main controller.
"""

import math
import time
from enum import Enum, auto

from config import (
    WHEELBASE, TRACK_WIDTH, MAX_STEERING_ANGLE_DEG,
    SERVO_CENTER_ANGLE, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE,
    PARK_SEARCH_SPEED, PARK_MANEUVER_SPEED,
    PARK_MIN_GAP_MM, PARK_TARGET_GAP_MM,
    PARK_OVERSHOOT_SEC, PARK_REVERSE_LOCK_SEC,
    PARK_REVERSE_STRAIGHT_SEC,
    PARK_GAP_CAMERA_MIN_SEC,
    EMERGENCY_STOP_MM,
)


class ParkingState(Enum):
    IDLE             = auto()
    SEARCHING        = auto()
    POSITIONING      = auto()
    REVERSING_LOCK   = auto()
    REVERSING_STRAIGHT = auto()
    PARKED           = auto()
    ABORTED          = auto()


class ParkingManeuver:
    """
    Battery-parking sub-FSM.

    Geometric parameters (all in config.py):
      PARK_OVERSHOOT_SEC       : extra forward time after detecting the gap
      PARK_REVERSE_LOCK_SEC    : reverse time with maximum steering
      PARK_REVERSE_STRAIGHT_SEC: straight reverse time

    These times are calibrated on the track. Replace with encoder/odometry
    if the car has them.
    """

    def __init__(self, gap_side: str = "right"):
        self.gap_side = gap_side
        self._state     = ParkingState.IDLE
        self._phase_start: float    = 0.0
        self._gap_detected_at: float = 0.0
        self._gap_open_since: float  = 0.0

        if gap_side == "right":
            self._lock_angle = SERVO_MAX_ANGLE
            self._straight_angle = SERVO_CENTER_ANGLE
        else:
            self._lock_angle = SERVO_MIN_ANGLE
            self._straight_angle = SERVO_CENTER_ANGLE

        self._R_turn = self._calc_turning_radius()

    @property
    def state(self) -> ParkingState:
        return self._state

    @property
    def is_active(self) -> bool:
        return self._state not in (ParkingState.IDLE,
                                   ParkingState.PARKED,
                                   ParkingState.ABORTED)

    @property
    def is_complete(self) -> bool:
        return self._state == ParkingState.PARKED

    def start(self):
        """Start searching for the parking gap."""
        self._state = ParkingState.SEARCHING
        self._phase_start = time.monotonic()
        print("[PARKING] Searching for a parking gap...")

    def abort(self):
        """Abort the manoeuvre and return to the IDLE state."""
        self._state = ParkingState.ABORTED
        print("[PARKING] Manoeuvre aborted.")

    def reset(self):
        self._state = ParkingState.IDLE

    def update(
        self,
        tof_distance_mm: float | None,
        motor,
        steering,
        obj_result=None,
    ) -> ParkingState:
        """
        Update the FSM. Must be called every iteration of the main loop.

        Parameters
        ----------
        tof_distance_mm : float | None
            VL53L0X reading (front or side depending on the mount).
        motor : MotorDriver
        steering : SteeringDriver

        Returns
        -------
        current ParkingState
        """
        now = time.monotonic()
        elapsed = now - self._phase_start

        if (self._state in (ParkingState.REVERSING_LOCK,
                             ParkingState.REVERSING_STRAIGHT)
                and tof_distance_mm is not None
                and tof_distance_mm < EMERGENCY_STOP_MM):
            motor.stop()
            steering.center()
            self.abort()
            print("[PARKING] EMERGENCY: obstacle during reverse.")
            return self._state

        match self._state:

            case ParkingState.SEARCHING:
                motor.set_throttle(PARK_SEARCH_SPEED)
                steering.center()

                gap_open = self._detect_gap(tof_distance_mm, obj_result, now)

                if gap_open:
                    self._gap_detected_at = now
                    self._transition(ParkingState.POSITIONING)
                    print(f"[PARKING] Gap detected. Positioning...")

            case ParkingState.POSITIONING:
                motor.set_throttle(PARK_SEARCH_SPEED)
                steering.center()

                if elapsed >= PARK_OVERSHOOT_SEC:
                    self._transition(ParkingState.REVERSING_LOCK)
                    print("[PARKING] Position ready. Starting reverse with steering...")

            case ParkingState.REVERSING_LOCK:
                motor.set_throttle(-PARK_MANEUVER_SPEED)
                steering.set_angle(self._lock_angle)

                arc_time = self._estimate_arc_time()
                if elapsed >= arc_time:
                    self._transition(ParkingState.REVERSING_STRAIGHT)
                    print("[PARKING] Arc complete. Straightening...")

            case ParkingState.REVERSING_STRAIGHT:
                motor.set_throttle(-PARK_MANEUVER_SPEED)
                steering.center()

                if elapsed >= PARK_REVERSE_STRAIGHT_SEC:
                    motor.stop()
                    steering.center()
                    self._state = ParkingState.PARKED
                    print("[PARKING] Parking complete!")

            case ParkingState.PARKED | ParkingState.IDLE | ParkingState.ABORTED:
                pass

        return self._state

    def _calc_turning_radius(self) -> float:
        """
        Turning radius in the REVERSING_LOCK phase using the maximum angle.
        R = L / tan(delta)
        """
        delta = abs(self._lock_angle - SERVO_CENTER_ANGLE)
        delta_rad = math.radians(delta)
        if delta_rad < 0.01:
            return float("inf")
        return WHEELBASE / math.tan(delta_rad)

    def _estimate_arc_time(self) -> float:
        """
        Estimate the time needed to rotate 90 deg along the Ackermann arc at
        the manoeuvre speed. It is only a guide -- the exact time is
        calibrated in PARK_REVERSE_LOCK_SEC (config.py).

        Arc length for 90 deg: s = (pi/2) * R
        Approximate linear speed (map 18% PWM -> ~0.25 m/s at 1:10 scale).
        """
        SPEED_MS_APPROX = 0.20
        arc_length = (math.pi / 2) * self._R_turn
        estimated = arc_length / SPEED_MS_APPROX if SPEED_MS_APPROX > 0 else PARK_REVERSE_LOCK_SEC
        return min(estimated, PARK_REVERSE_LOCK_SEC)

    def _detect_gap(self, tof_mm, obj_result, now: float) -> bool:
        """
        Combine camera and ToF to decide whether there is a parking gap.

        Logic:
        - If obj_result exists -> use the camera as the primary source.
          The gap exists when there is NO AUTO in the right-side zone for at
          least PARK_GAP_CAMERA_MIN_SEC seconds.
        - If there is no obj_result -> fall back to ToF (original behavior).
        """
        if obj_result is not None:
            return self._detect_gap_camera(obj_result, now)

        gap_open = (tof_mm is None or tof_mm >= PARK_MIN_GAP_MM)
        return gap_open and (now - self._phase_start) > 0.3

    def _detect_gap_camera(self, obj_result, now: float) -> bool:
        """
        Detect the gap when the right-side space is clear.

        While the first delimiting car was on the right side and now there is
        no longer an AUTO in that zone = start of the gap.
        Requires PARK_GAP_CAMERA_MIN_SEC consecutive seconds without a side
        AUTO to confirm (avoids false positives from noisy frames).
        """
        lateral_clear = not obj_result.car_in_park_zone

        if lateral_clear:
            if self._gap_open_since == 0.0:
                self._gap_open_since = now
            gap_secs = now - self._gap_open_since
            return gap_secs >= PARK_GAP_CAMERA_MIN_SEC
        else:
            self._gap_open_since = 0.0
            return False

    def _transition(self, new_state: ParkingState):
        self._state = new_state
        self._phase_start = time.monotonic()
        self._gap_open_since = 0.0
