"""Pi AI Camera (Sony IMX500) via Picamera2.

NPU strategy:
  The IMX500 runs YOLOv8 inference directly on its integrated neural
  accelerator (on-chip NPU). The Pi 5 receives:
    1. The BGR888 frame as a numpy array (for OpenCV / lane detection).
    2. The model output tensors embedded in the capture metadata
       (imx500.get_outputs). These are parsed into Detection objects.

  Benefit: the Pi 5 CPU does NOT run inference -- it only parses small
  tensors. RAM is not saturated by heavy models loaded in Python.

Requirement on Raspberry Pi OS:
  sudo apt install imx500-all picamera2
"""

import threading
import time
import queue
from dataclasses import dataclass, field
from typing import Optional

import numpy as np
from picamera2 import Picamera2
from picamera2.devices.imx500 import IMX500

from config import (
    IMX500_MODEL_PATH,
    CAMERA_WIDTH,
    CAMERA_HEIGHT,
    CAMERA_FPS,
    CAMERA_AWB_MODE,
    CAMERA_CONTRAST,
    CAMERA_SATURATION,
    CAMERA_SHARPNESS,
    CAMERA_DENOISE,
    CAMERA_BUFFERS,
    DETECTION_CONFIDENCE,
    CLASSES_OF_INTEREST,
    STOP_SIGN_REAL_HEIGHT_M,
    CAMERA_FOCAL_LENGTH_PX,
)


@dataclass
class Detection:
    label: str
    class_id: int
    confidence: float
    x1: int; y1: int
    x2: int; y2: int

    @property
    def width(self)  -> int: return self.x2 - self.x1
    @property
    def height(self) -> int: return self.y2 - self.y1
    @property
    def area(self)   -> int: return self.width * self.height
    @property
    def cx(self)     -> int: return (self.x1 + self.x2) // 2
    @property
    def cy(self)     -> int: return (self.y1 + self.y2) // 2

    def estimated_distance_m(self) -> Optional[float]:
        """
        Estimate the distance using the bounding-box height and the known real
        height of the object (pin-hole perspective method).

        Only accurate for objects with a known real height (STOP sign).
        """
        if self.label != "STOP" or self.height < 5:
            return None
        return (STOP_SIGN_REAL_HEIGHT_M * CAMERA_FOCAL_LENGTH_PX) / self.height


@dataclass
class CameraFrame:
    image: np.ndarray
    detections: list[Detection] = field(default_factory=list)
    timestamp: float = field(default_factory=time.monotonic)


class CameraManager:
    """
    Capture and processing of the Pi AI Camera on a dedicated thread.

    The thread produces CameraFrame objects that the main loop consumes via
    get_latest_frame() -- it always returns the most recent frame and never
    blocks for more than a few ms.
    """

    def __init__(self):
        self._imx500 = IMX500(IMX500_MODEL_PATH)
        self._picam2 = Picamera2(self._imx500.camera_num)

        cfg = self._picam2.create_preview_configuration(
            main={"format": "BGR888", "size": (CAMERA_WIDTH, CAMERA_HEIGHT)},
            controls={
                "FrameRate"          : CAMERA_FPS,
                "AwbEnable"          : True,
                "AwbMode"            : CAMERA_AWB_MODE,
                "AeEnable"           : True,
                "Contrast"           : CAMERA_CONTRAST,
                "Saturation"         : CAMERA_SATURATION,
                "Sharpness"          : CAMERA_SHARPNESS,
                "NoiseReductionMode" : CAMERA_DENOISE,
            },
            buffer_count=CAMERA_BUFFERS,
        )
        self._picam2.configure(cfg)

        self._queue: queue.Queue[CameraFrame] = queue.Queue(maxsize=1)

        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._last_frame: CameraFrame | None = None
        self._frame_lock = threading.Lock()

        self._class_map: dict[int, str] = {}

    def start(self):
        self._picam2.start()
        time.sleep(3.0)

        try:
            self._picam2.set_controls({
                "AfMode"  : 2,
                "AfSpeed" : 1,
            })
            print("[CAM] Autoenfoque continuo activado")
        except Exception:
            print("[CAM] Autoenfoque no disponible — enfoque fijo")

        self._build_class_map()

        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._capture_loop,
            name="Camera-IMX500",
            daemon=True,
        )
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=3.0)
        self._picam2.stop()

    def get_latest_frame(self) -> Optional[CameraFrame]:
        """
        Return the most recent frame without blocking.
        Returns None if none has been captured yet.
        """
        with self._frame_lock:
            return self._last_frame

    def get_frame_blocking(self, timeout: float = 0.1) -> Optional[CameraFrame]:
        """Espera hasta `timeout` segundos por un frame nuevo."""
        try:
            return self._queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def _capture_loop(self):
        while not self._stop_event.is_set():
            try:
                request = self._picam2.capture_request()
                image = request.make_array("main")

                metadata = request.get_metadata()
                request.release()

                detections = self._parse_npu_output(metadata, image.shape)

                frame = CameraFrame(image=image, detections=detections)

                with self._frame_lock:
                    self._last_frame = frame

                try:
                    self._queue.put_nowait(frame)
                except queue.Full:
                    try:
                        self._queue.get_nowait()
                        self._queue.put_nowait(frame)
                    except queue.Empty:
                        pass

            except Exception as e:
                time.sleep(0.01)

    def _parse_npu_output(
        self, metadata: dict, img_shape: tuple
    ) -> list[Detection]:
        """
        Extract detections from the IMX500 output tensor.

        EfficientDet Lite0 _pp exports 4 tensors (TensorFlow/COCO format):
          [0] boxes   : (1, N, 4)  [ymin, xmin, ymax, xmax] normalized 0-1
          [1] classes : (1, N)     class ID (float)
          [2] scores  : (1, N)     confidence
          [3] count   : (1,)       number of valid detections

        NOTE: the boxes/classes/scores order varies by compiled model.
        The code tries the EfficientDet format and falls back to the YOLOv8
        format if the first fails, to be robust to alternative models.
        """
        np_outputs = self._imx500.get_outputs(metadata, add_batch=True)
        if np_outputs is None:
            return []

        ih, iw = img_shape[:2]

        try:
            if len(np_outputs) >= 4:
                boxes   = np_outputs[0][0]
                classes = np_outputs[1][0]
                scores  = np_outputs[2][0]
                count   = int(np_outputs[3][0])
                tf_format = True
            else:
                boxes   = np_outputs[0][0]
                scores  = np_outputs[1][0]
                classes = np_outputs[2][0]
                count   = len(scores)
                tf_format = False
        except (IndexError, TypeError):
            return []

        detections: list[Detection] = []
        for i in range(min(count, len(scores))):
            score  = float(scores[i])
            cls_id = int(classes[i])

            if score < DETECTION_CONFIDENCE:
                continue

            label = self._class_map.get(cls_id, f"cls_{cls_id}")
            if label not in CLASSES_OF_INTEREST.values():
                continue

            box = boxes[i]
            if tf_format:
                y1 = int(box[0] * ih)
                x1 = int(box[1] * iw)
                y2 = int(box[2] * ih)
                x2 = int(box[3] * iw)
            else:
                x1 = int(box[0] * iw)
                y1 = int(box[1] * ih)
                x2 = int(box[2] * iw)
                y2 = int(box[3] * ih)

            x1, x2 = sorted([max(0, x1), min(iw - 1, x2)])
            y1, y2 = sorted([max(0, y1), min(ih - 1, y2)])

            detections.append(Detection(
                label=label,
                class_id=cls_id,
                confidence=score,
                x1=x1, y1=y1, x2=x2, y2=y2,
            ))

        return detections

    def _build_class_map(self):
        """
        Build the class_id -> name map using the intrinsics of the model
        loaded in the IMX500, then filter with CLASSES_OF_INTEREST.
        """
        try:
            intrinsics = self._imx500.network_intrinsics
            if intrinsics and intrinsics.labels:
                for idx, raw_label in enumerate(intrinsics.labels):
                    label_lower = raw_label.strip().lower()
                    for known, friendly in CLASSES_OF_INTEREST.items():
                        if known in label_lower:
                            self._class_map[idx] = friendly
                            break
        except Exception:
            pass

    @property
    def resolution(self) -> tuple[int, int]:
        return CAMERA_WIDTH, CAMERA_HEIGHT
