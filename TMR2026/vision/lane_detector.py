"""Lane detection with OpenCV.

TMR track: BLACK surface with WHITE lines.

Algorithm:
  1. ROI: lower strip of the frame (where the nearest lines are).
  2. Grayscale -> blur -> high binary threshold (white on black).
  3. Horizontal sliding window to detect the left and right lines.
  4. Lane center = mean of both lines.
  5. Lane error = lane_center - image_center  (px).
  6. Estimated curvature = error difference between the near and far bands.
  7. Suggested speed based on the curvature.

Output: LaneData with error_px, curvature_rad, is_curve, confidence.
"""

from dataclasses import dataclass
import math
import cv2
import numpy as np

from config import (
    CAMERA_WIDTH, CAMERA_HEIGHT,
    CURVE_THRESHOLD_RAD,
    SPEED_STRAIGHT, SPEED_CURVE,
    LANE_LOST_THRESHOLD_PX,
    CROSSWALK_WHITE_RATIO,
)


@dataclass
class LaneData:
    error_px: float
    curvature_rad: float
    is_curve: bool
    confidence: float
    suggested_speed: float
    crosswalk_detected: bool = False
    debug_image: np.ndarray | None = None


class LaneDetector:
    """
    Vision lane detector for a black track with white lines.

    Calibratable parameters:
      roi_top_ratio   -- fraction of the height (from the top) where the ROI starts
      threshold       -- minimum gray value to count as "white"
      n_windows       -- number of horizontal windows in the ROI
    """

    def __init__(
        self,
        roi_top_ratio: float = 0.55,
        roi_near_ratio: float = 0.80,
        threshold: int = 160,
        n_windows: int = 6,
        debug: bool = False,
    ):
        self.roi_top_ratio  = roi_top_ratio
        self.roi_near_ratio = roi_near_ratio
        self.threshold      = threshold
        self.n_windows      = n_windows
        self.debug          = debug

        self._W = CAMERA_WIDTH
        self._H = CAMERA_HEIGHT
        self._mid = self._W // 2

    def process(self, frame: np.ndarray) -> LaneData:
        """
        Process a BGR frame and return LaneData.

        Parameters
        ----------
        frame : np.ndarray
            BGR888 camera frame (640x480).
        """
        gray   = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        _, binary = cv2.threshold(blurred, self.threshold, 255, cv2.THRESH_BINARY)

        roi_top  = int(self._H * self.roi_top_ratio)
        roi_near = int(self._H * self.roi_near_ratio)

        far_band  = binary[roi_top  : roi_near, :]
        near_band = binary[roi_near :          , :]

        cx_far,  conf_far  = self._find_lane_center(far_band)
        cx_near, conf_near = self._find_lane_center(near_band)

        confidence = (conf_far + conf_near) / 2.0

        error_px = cx_near - self._mid

        far_height  = roi_near - roi_top
        near_height = self._H - roi_near
        dy = far_height + near_height / 2
        curvature_rad = math.atan2(abs(cx_near - cx_far), dy) if dy > 0 else 0.0

        is_curve = curvature_rad > CURVE_THRESHOLD_RAD

        t = min(curvature_rad / CURVE_THRESHOLD_RAD, 1.0) if CURVE_THRESHOLD_RAD > 0 else 0
        suggested_speed = SPEED_STRAIGHT * (1 - t) + SPEED_CURVE * t

        if abs(error_px) > LANE_LOST_THRESHOLD_PX:
            confidence = 0.0
            suggested_speed = SPEED_CURVE

        debug_img = None
        if self.debug:
            debug_img = self._draw_debug(
                frame, binary, roi_top, roi_near,
                cx_near, cx_far, error_px
            )

        crosswalk = self._detect_crosswalk(binary, roi_top)

        return LaneData(
            error_px          = error_px,
            curvature_rad     = curvature_rad,
            is_curve          = is_curve,
            confidence        = confidence,
            suggested_speed   = suggested_speed,
            crosswalk_detected = crosswalk,
            debug_image       = debug_img,
        )

    def _find_lane_center(self, band: np.ndarray) -> tuple[float, float]:
        """
        Locate the left and right lines in `band` using a column-wise sliding
        window and return (center_px, confidence).

        If only one line is detected, estimate the other from the typical lane
        width (half the image = the whole visible lane).
        """
        if band.size == 0:
            return float(self._mid), 0.0

        col_sum = np.sum(band, axis=0).astype(np.float32)

        mid = self._W // 2
        left_half  = col_sum[:mid]
        right_half = col_sum[mid:]

        left_peak  = int(np.argmax(left_half))              if left_half.max()  > 0 else None
        right_peak = int(np.argmax(right_half)) + mid       if right_half.max() > 0 else None

        if left_peak is not None and right_peak is not None:
            center     = (left_peak + right_peak) / 2.0
            confidence = min(
                left_half[left_peak]  / (band.shape[0] * 255),
                right_half[right_peak - mid] / (band.shape[0] * 255)
            )
            confidence = min(confidence * 5.0, 1.0)
        elif left_peak is not None:
            center = left_peak + self._W * 0.35
            confidence = 0.4
        elif right_peak is not None:
            center = right_peak - self._W * 0.35
            confidence = 0.4
        else:
            center = float(self._mid)
            confidence = 0.0

        return center, confidence

    def _detect_crosswalk(self, binary: np.ndarray, roi_top: int) -> bool:
        """
        Detect a crosswalk (wide horizontal white lines).

        Search the middle band of the frame (between the horizon and the lane
        ROI) for rows where more than CROSSWALK_WHITE_RATIO of the pixels are
        white. That indicates a white stripe crossing the full track width.

        At least 3 consecutive rows are required to avoid false positives.
        """
        search_top    = int(self._H * 0.30)
        search_bottom = roi_top

        if search_bottom <= search_top:
            return False

        zone = binary[search_top:search_bottom, :]
        if zone.size == 0:
            return False

        row_ratios = np.sum(zone > 0, axis=1) / self._W

        white_rows = row_ratios > CROSSWALK_WHITE_RATIO

        consecutive = 0
        for w in white_rows:
            if w:
                consecutive += 1
                if consecutive >= 3:
                    return True
            else:
                consecutive = 0
        return False

    def _draw_debug(
        self, frame, binary, roi_top, roi_near,
        cx_near, cx_far, error_px
    ) -> np.ndarray:
        vis = frame.copy()

        cv2.line(vis, (0, roi_top),  (self._W, roi_top),  (0, 200, 200), 1)
        cv2.line(vis, (0, roi_near), (self._W, roi_near), (0, 200, 200), 1)

        cv2.line(vis, (self._mid, roi_top), (self._mid, self._H), (200, 200, 0), 1)

        cy_near = int((self._H + roi_near) / 2)
        cy_far  = int((roi_top + roi_near) / 2)
        cv2.circle(vis, (int(cx_near), cy_near), 6, (0, 255, 0), -1)
        cv2.circle(vis, (int(cx_far),  cy_far),  6, (255, 165, 0), -1)

        cv2.putText(vis, f"Error: {error_px:.1f}px", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        return vis
