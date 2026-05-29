# -*- coding: utf-8 -*-
"""
sim_hardware_mocks.py — Mocks de Hardware para Sim2Real (PC ↔ Raspberry Pi)

Reemplaza dependencias de Pi (lgpio, picamera2, smbus2) con sockets TCP/UDP.
Permite que el código TMR2026 corra en PC conectado a simulador Unity.

Uso:
    from sim_hardware_mocks import SimulatorClient

    sim = SimulatorClient(host='127.0.0.1', port=5005)
    sim.motor.set_speed(50.0)  # % PWM
    sim.steering.set_angle(90.0)  # grados
    distance_mm = sim.distance.front_mm
    frame = sim.camera.get_latest_frame()
"""

import socket
import threading
import time
import sys
import numpy as np
import cv2
import io
from typing import Optional
from collections import deque

# Forzar UTF-8 en la consola (Windows cp1252 crashea con ✓ ✗ → etc.)
try:
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")
except Exception:
    pass

# ============================================================================
# MOCKS DE HARDWARE
# ============================================================================

class MockMotorDriver:
    """Reemplaza: hardware/motor.py"""

    def __init__(self, socket_client):
        self.socket = socket_client
        self.current_duty = 0.0
        self._lock = threading.Lock()

    def set_speed(self, duty_percent: float):
        """
        Envía comando PWM al simulador.
        duty_percent: [-100, 100] (negativo = reversa, positivo = avance)
        """
        duty_percent = max(-100, min(100, duty_percent))
        with self._lock:
            self.current_duty = duty_percent

        try:
            msg = f"MOTOR:{duty_percent:.2f}\n".encode()
            self.socket.sendall(msg)
        except Exception as e:
            print(f"[MockMotor] Error enviando PWM: {e}")

    def brake(self):
        """Freno inmediato (duty = 0)"""
        self.set_speed(0.0)

    def stop(self):
        """Alias de brake()"""
        self.brake()


class MockSteeringDriver:
    """Reemplaza: hardware/steering_driver.py"""

    def __init__(self, socket_client):
        self.socket = socket_client
        self.current_angle = 90.0
        self._lock = threading.Lock()

    def set_angle(self, angle_deg: float):
        """
        Envía ángulo del servo al simulador.
        angle_deg: [0, 180] (90 = centro, <90 = izquierda, >90 = derecha)
        """
        angle_deg = max(0, min(180, angle_deg))
        with self._lock:
            self.current_angle = angle_deg

        try:
            msg = f"SERVO:{angle_deg:.2f}\n".encode()
            self.socket.sendall(msg)
        except Exception as e:
            print(f"[MockSteering] Error enviando ángulo: {e}")

    def center(self):
        """Centra el servo (90°). Lo usa el FSM en deactivate()."""
        self.set_angle(90.0)

    @property
    def current_angle_deg(self):
        return self.current_angle


class MockDistanceSensor:
    """Reemplaza: hardware/distance_sensor.py (VL53L0X x2).
    NO crea su propio thread; los datos los inyecta el receptor unificado."""

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
    """Reemplaza: vision/camera_stream.py.
    NO crea su propio thread; los frames los inyecta el receptor unificado."""

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
        """Retorna último frame recibido (BGR)"""
        with self.frame_lock:
            return self.latest_frame.copy() if self.latest_frame is not None else None

    def stop(self):
        pass


# ============================================================================
# CLIENTE SIMULADOR
# ============================================================================

class SimulatorClient:
    """
    Cliente que conecta Python ↔ Simulador Unity via TCP/IP.
    Proporciona interfaces idénticas a hardware real.
    """

    def __init__(self, host='127.0.0.1', port=5005, timeout=5.0):
        """
        Conectar al simulador.

        Args:
            host: IP del simulador (127.0.0.1 = localhost)
            port: Puerto TCP (defecto 5005)
            timeout: Timeout de socket en segundos
        """
        print(f"[SimClient] Conectando a {host}:{port}...")

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.settimeout(timeout)

        try:
            self.socket.connect((host, port))
            print(f"[SimClient] ✓ Conectado a simulador")
        except Exception as e:
            print(f"[SimClient] ✗ ERROR conectando: {e}")
            print(f"[SimClient] ¿Unity está corriendo en {host}:{port}?")
            raise

        # Crear mocks
        self.motor = MockMotorDriver(self.socket)
        self.steering = MockSteeringDriver(self.socket)
        self.distance = MockDistanceSensor(self.socket)
        self.camera = MockCameraStream(self.socket, width=640, height=480)

        # Receptor unificado (UN solo thread leyendo el socket)
        self._listening = True
        self._recv_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._recv_thread.start()

    def _receive_loop(self):
        """
        Lee del socket y demultiplexa mensajes:
          - Texto:   "TOF:front,rear\n"  -> MockDistanceSensor
          - Binario: [4 bytes BE size][JPEG] -> MockCameraStream
        """
        buffer = b""
        while self._listening:
            try:
                data = self.socket.recv(65536)
                if not data:
                    # peer cerró el socket
                    print("[SimClient] Servidor cerró la conexión.")
                    self._listening = False
                    break
                buffer += data

                # Procesar todo lo que se pueda del buffer
                while True:
                    if len(buffer) == 0:
                        break

                    first = buffer[0:1]

                    # --- Mensaje TOF (texto) ---
                    if first == b'T':
                        nl = buffer.find(b'\n')
                        if nl == -1:
                            break  # mensaje incompleto, esperar más bytes
                        line = buffer[:nl].decode(errors='ignore').strip()
                        buffer = buffer[nl + 1:]
                        if line.startswith("TOF:"):
                            try:
                                parts = line[4:].split(",")
                                if len(parts) >= 2:
                                    self.distance._update(int(parts[0]), int(parts[1]))
                            except (ValueError, IndexError):
                                pass

                    # --- Frame JPEG (binario) ---
                    else:
                        if len(buffer) < 4:
                            break  # header incompleto
                        size = int.from_bytes(buffer[:4], 'big')
                        # Sanidad: descartar tamaños absurdos (>5 MB)
                        if size <= 0 or size > 5_000_000:
                            # Algo se desincronizó: descartar 1 byte y reintentar
                            buffer = buffer[1:]
                            continue
                        if len(buffer) < 4 + size:
                            break  # JPEG incompleto
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
                    print(f"[SimClient] OSError en recv: {e}")
                break
            except Exception as e:
                if self._listening:
                    print(f"[SimClient] recv error: {e}")
                time.sleep(0.01)

    def close(self):
        """Cerrar conexión con simulador"""
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
            print("[SimClient] ✓ Desconectado")
        except Exception:
            pass

    def is_connected(self) -> bool:
        """Verificar si conexión está activa"""
        try:
            self.socket.sendall(b"PING")
            return True
        except:
            return False

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()


# ============================================================================
# SCRIPT DE PRUEBA
# ============================================================================

if __name__ == "__main__":
    print("=" * 70)
    print("SIM2REAL HARDWARE MOCKS - TEST")
    print("=" * 70)

    try:
        with SimulatorClient(host='127.0.0.1', port=5005) as sim:
            print("\n[TEST] Iniciando pruebas...")

            # Test 1: Motor
            print("\n[TEST] Probando motor...")
            for pwm in [10, 25, 50, 0]:
                sim.motor.set_speed(pwm)
                time.sleep(0.2)
                print(f"  Motor PWM: {pwm}%")

            # Test 2: Servo
            print("\n[TEST] Probando servo...")
            for angle in [45, 90, 135, 90]:
                sim.steering.set_angle(angle)
                time.sleep(0.2)
                print(f"  Servo ángulo: {angle}°")

            # Test 3: Sensores
            print("\n[TEST] Leyendo sensores (10 segundos)...")
            start = time.time()
            while time.time() - start < 10:
                if sim.distance.front_mm is not None:
                    print(f"  ToF: Front={sim.distance.front_mm}mm, Rear={sim.distance.rear_mm}mm", end='\r')

                frame = sim.camera.get_latest_frame()
                if frame is not None:
                    print(f"  Cámara: {frame.shape} (Frame recibido)")

                time.sleep(0.1)

            print("\n[TEST] ✓ Pruebas completadas")

    except Exception as e:
        print(f"\n[TEST] ✗ Error: {e}")
        import traceback
        traceback.print_exc()
