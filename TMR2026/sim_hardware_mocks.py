"""Hardware mocks for Sim2Real (PC <-> Raspberry Pi).

Replaces the Pi dependencies (lgpio, picamera2, smbus2) with TCP/UDP sockets,
so the TMR2026 code can run on a PC connected to the Unity simulator.

Usage:
    from sim_hardware_mocks import SimulatorClient

    sim = SimulatorClient(host='127.0.0.1', port=5005)
    sim.motor.set_speed(50.0)     # % PWM
    sim.steering.set_angle(90.0)  # degrees
    distance_mm = sim.distance.front_mm
    frame = sim.camera.get_latest_frame()
"""

import socket
import threading
import time
import sys
import numpy as np
import cv2
from typing import Optional

try:
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")
except Exception:
    pass


class MockMotorDriver:
    """Replaces: hardware/motor.py"""

    def __init__(self, socket_client):
        self.socket = socket_client
        self.current_duty = 0.0
        self._lock = threading.Lock()

    def set_speed(self, duty_percent: float):
        """
        Send a PWM command to the simulator.
        duty_percent: [-100, 100] (negative = reverse, positive = forward)
        """
        duty_percent = max(-100, min(100, duty_percent))
        with self._lock:
            self.current_duty = duty_percent

        try:
            msg = f"MOTOR:{duty_percent:.2f}\n".encode()
            self.socket.sendall(msg)
        except Exception as e:
            print(f"[MockMotor] Error sending PWM: {e}")

    def brake(self):
        """Instantaneous brake (duty = 0)."""
        self.set_speed(0.0)

    def stop(self):
        """Alias for brake()."""
        self.brake()


class MockSteeringDriver:
    """Replaces: hardware/steering_driver.py.

    Replicates the Pi's physical inversion: the car's servo is mounted
    reversed, and config.STEERING_INVERTED compensates for it. So the
    simulator behaves EXACTLY like the real car, we apply the same inversion
    before sending the angle to Unity. current_angle is still the LOGICAL one."""

    STEERING_INVERTED = True
    SERVO_CENTER = 90.0

    def __init__(self, socket_client):
        self.socket = socket_client
        self.current_angle = 90.0
        self._lock = threading.Lock()

    def set_angle(self, angle_deg: float):
        """
        LOGICAL angle_deg: [0,180] (90=straight, <90=left, >90=right).
        The PHYSICAL (inverted) angle is sent to Unity, as on the real car.
        """
        angle_deg = max(0, min(180, angle_deg))
        with self._lock:
            self.current_angle = angle_deg

        physical = (2 * self.SERVO_CENTER - angle_deg
                    if self.STEERING_INVERTED else angle_deg)
        try:
            msg = f"SERVO:{physical:.2f}\n".encode()
            self.socket.sendall(msg)
        except Exception as e:
            print(f"[MockSteering] Error sending angle: {e}")

    def center(self):
        """Centre the servo (90 deg). Used by the FSM in deactivate()."""
        self.set_angle(90.0)

    @property
    def current_angle_deg(self):
        return self.current_angle


class MockDistanceSensor:
    """Replaces: hardware/distance_sensor.py (2x VL53L0X).
    Does NOT create its own thread; data is injected by the unified receiver."""

    def __init__(self, socket_client):
        self.socket = socket_client
        self.front_mm = None
        self.rear_mm = None
        self._lock = threading.Lock()

    def _update(self, front_mm: int, rear_mm: int):
        with self._lock:
            self.front_mm = front_mm
            self.rear_mm = rear_mm

    def stop(self):
        pass


class MockCameraStream:
    """Replaces: vision/camera_stream.py.
    Does NOT create its own thread; frames are injected by the unified receiver."""

    def __init__(self, socket_client, width=640, height=480):
        self.socket = socket_client
        self.width = width
        self.height = height
        self.latest_frame = None
        self.frame_lock = threading.Lock()

    def _update(self, frame):
        with self.frame_lock:
            self.latest_frame = frame

    def get_latest_frame(self) -> Optional[np.ndarray]:
        """Return the latest received frame (BGR)."""
        with self.frame_lock:
            return self.latest_frame.copy() if self.latest_frame is not None else None

    def stop(self):
        pass


class SimulatorClient:
    """
    Client connecting Python <-> Unity simulator over TCP/IP.
    Provides interfaces identical to the real hardware.
    """

    def __init__(self, host='127.0.0.1', port=5005, timeout=5.0):
        """
        Connect to the simulator.

        Args:
            host: simulator IP (127.0.0.1 = localhost)
            port: TCP port (default 5005)
            timeout: socket timeout in seconds
        """
        print(f"[SimClient] Connecting to {host}:{port}...")

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.settimeout(timeout)

        try:
            self.socket.connect((host, port))
            print("[SimClient] Connected to the simulator")
        except Exception as e:
            print(f"[SimClient] ERROR connecting: {e}")
            print(f"[SimClient] Is Unity running on {host}:{port}?")
            raise

        self.motor = MockMotorDriver(self.socket)
        self.steering = MockSteeringDriver(self.socket)
        self.distance = MockDistanceSensor(self.socket)
        self.camera = MockCameraStream(self.socket, width=640, height=480)

        self._listening = True
        self._recv_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._recv_thread.start()

    def _receive_loop(self):
        """
        Read from the socket and demultiplex messages:
          - Text:   "TOF:front,rear\n"  -> MockDistanceSensor
          - Binary: [4 bytes BE size][JPEG] -> MockCameraStream
        """
        buffer = b""
        while self._listening:
            try:
                data = self.socket.recv(65536)
                if not data:
                    print("[SimClient] Server closed the connection.")
                    self._listening = False
                    break
                buffer += data

                while True:
                    if len(buffer) == 0:
                        break

                    first = buffer[0:1]

                    if first == b'T':
                        nl = buffer.find(b'\n')
                        if nl == -1:
                            break
                        line = buffer[:nl].decode(errors='ignore').strip()
                        buffer = buffer[nl + 1:]
                        if line.startswith("TOF:"):
                            try:
                                parts = line[4:].split(",")
                                if len(parts) >= 2:
                                    self.distance._update(int(parts[0]), int(parts[1]))
                            except (ValueError, IndexError):
                                pass

                    else:
                        if len(buffer) < 4:
                            break
                        size = int.from_bytes(buffer[:4], 'big')
                        if size <= 0 or size > 5_000_000:
                            buffer = buffer[1:]
                            continue
                        if len(buffer) < 4 + size:
                            break
                        jpeg_bytes = buffer[4:4 + size]
                        buffer = buffer[4 + size:]
                        try:
                            arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
                            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                            if frame is not None:
                                self.camera._update(frame)
                        except Exception as e:
                            print(f"[SimClient] JPEG decode error: {e}")

            except socket.timeout:
                continue
            except OSError as e:
                if self._listening:
                    print(f"[SimClient] OSError in recv: {e}")
                break
            except Exception as e:
                if self._listening:
                    print(f"[SimClient] recv error: {e}")
                time.sleep(0.01)

    def close(self):
        """Close the connection to the simulator."""
        self._listening = False
        try:
            self.motor.brake()
            self.steering.set_angle(90.0)
        except Exception:
            pass
        self.distance.stop()
        self.camera.stop()

        try:
            self.socket.shutdown(socket.SHUT_RDWR)
        except Exception:
            pass
        try:
            self.socket.close()
            print("[SimClient] Disconnected")
        except Exception:
            pass

    def is_connected(self) -> bool:
        """Check whether the connection is active."""
        try:
            self.socket.sendall(b"PING")
            return True
        except:
            return False

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()


if __name__ == "__main__":
    print("=" * 70)
    print("SIM2REAL HARDWARE MOCKS - TEST")
    print("=" * 70)

    try:
        with SimulatorClient(host='127.0.0.1', port=5005) as sim:
            print("\n[TEST] Starting tests...")

            print("\n[TEST] Testing motor...")
            for pwm in [10, 25, 50, 0]:
                sim.motor.set_speed(pwm)
                time.sleep(0.2)
                print(f"  Motor PWM: {pwm}%")

            print("\n[TEST] Testing servo...")
            for angle in [45, 90, 135, 90]:
                sim.steering.set_angle(angle)
                time.sleep(0.2)
                print(f"  Servo angle: {angle} deg")

            print("\n[TEST] Reading sensors (10 seconds)...")
            start = time.time()
            while time.time() - start < 10:
                if sim.distance.front_mm is not None:
                    print(f"  ToF: Front={sim.distance.front_mm}mm, Rear={sim.distance.rear_mm}mm", end='\r')

                frame = sim.camera.get_latest_frame()
                if frame is not None:
                    print(f"  Camera: {frame.shape} (frame received)")

                time.sleep(0.1)

            print("\n[TEST] Tests complete")

    except Exception as e:
        print(f"\n[TEST] Error: {e}")
        import traceback
        traceback.print_exc()
