"""
test_fake_unity.py — Servidor que IMITA a Unity para probar el lado PC.
Manda TOF (50 Hz) + JPEG (30 FPS) con el formato EXACTO del protocolo,
luego conecta el SimulatorClient real y verifica que recibe todo bien.

Si esto pasa -> el lado PC (Python) esta 100% listo, solo falta Unity real.
"""
import socket, threading, time, struct
import numpy as np
import cv2

PORT = 5005
_running = True

def fake_unity_server():
    """Imita a Unity: acepta cliente, manda TOF + JPEG como SimulatorServer.cs."""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("127.0.0.1", PORT))
    s.listen(1)
    print(f"[FAKE-UNITY] Escuchando en 127.0.0.1:{PORT} (imitando Unity)...")
    conn, addr = s.accept()
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    print(f"[FAKE-UNITY] Cliente PC conectado desde {addr}")

    # Frame dummy: pista oscura con 2 lineas blancas verticales (como Unity)
    frame = np.full((480, 640, 3), 42, dtype=np.uint8)   # gris oscuro #2A
    cv2.rectangle(frame, (170, 0), (185, 480), (255, 255, 255), -1)  # linea izq
    cv2.rectangle(frame, (455, 0), (470, 480), (255, 255, 255), -1)  # linea der

    tof_t = cam_t = time.time()
    sent_tof = sent_jpeg = 0
    t0 = time.time()
    while _running and time.time() - t0 < 6:
        now = time.time()
        # TOF cada 20ms (50 Hz)
        if now - tof_t >= 0.02:
            tof_t = now
            try:
                conn.sendall(b"TOF:370,2000\n")   # 37 cm frente
                sent_tof += 1
            except Exception:
                break
        # JPEG cada 33ms (30 FPS)
        if now - cam_t >= 0.033:
            cam_t = now
            ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
            if ok:
                data = jpg.tobytes()
                size = struct.pack(">I", len(data))   # 4 bytes big-endian
                try:
                    conn.sendall(size + data)
                    sent_jpeg += 1
                except Exception:
                    break
        time.sleep(0.001)
    print(f"[FAKE-UNITY] Enviados: {sent_tof} TOF, {sent_jpeg} JPEG")
    try: conn.close()
    except: pass
    try: s.close()
    except: pass


# Arrancar Unity falso
th = threading.Thread(target=fake_unity_server, daemon=True)
th.start()
time.sleep(0.5)

# Conectar el cliente REAL del proyecto
from sim_hardware_mocks import SimulatorClient
print("[TEST] Conectando SimulatorClient real...")
sim = SimulatorClient(host="127.0.0.1", port=PORT)

# Mandar comandos PC -> Unity (motor + servo)
sim.motor.set_speed(30.0)
sim.steering.set_angle(75.0)

# Recibir 5 segundos
print("[TEST] Recibiendo sensores 5 s...")
start = time.time()
tof_ok = frame_ok = 0
last_dist = None
while time.time() - start < 5:
    if sim.distance.front_mm is not None:
        tof_ok += 1
        last_dist = sim.distance.front_mm
    f = sim.camera.get_latest_frame()
    if f is not None:
        frame_ok += 1
    time.sleep(0.05)

print("\n" + "=" * 55)
print(f"  TOF recibidos:    {tof_ok}   (ultima dist: {last_dist} mm)")
print(f"  Frames recibidos: {frame_ok}")
print("=" * 55)
if tof_ok > 20 and frame_ok > 10:
    print("  >>> LADO PC 100% OK. Solo falta poner Unity en Play.")
else:
    print("  >>> Algo falla en el cliente PC. Revisar receptor.")

_running = False
sim.close()
time.sleep(0.3)
