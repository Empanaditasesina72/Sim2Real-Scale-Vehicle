"""
test_python_loopback.py — Verifica si TCP localhost funciona en esta PC.
Crea un servidor Python que envia datos y un cliente Python que los recibe.
NADA de Unity involucrado.

Si esto recibe bytes -> TCP loopback funciona, problema es Unity.
Si esto NO recibe bytes -> problema de firewall/antivirus en localhost.
"""
import socket
import threading
import time

PORT = 5006  # uso 5006 para no chocar con Unity en 5005

def server_thread():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('127.0.0.1', PORT))
    s.listen(1)
    print(f"[SERVER] Escuchando en 127.0.0.1:{PORT}...")
    conn, addr = s.accept()
    print(f"[SERVER] Cliente conectado desde {addr}")
    # Enviar 30 mensajes rapidos como hace Unity
    for i in range(30):
        msg = f"TOF:{500 + i},{2000 - i}\n".encode()
        conn.sendall(msg)
        time.sleep(0.02)
    print("[SERVER] Envio 30 mensajes, cerrando")
    conn.close()
    s.close()

# Lanzar servidor en thread
t = threading.Thread(target=server_thread, daemon=True)
t.start()
time.sleep(0.5)  # dar tiempo a que arranque

# Cliente
print(f"[CLIENT] Conectando a 127.0.0.1:{PORT}...")
c = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
c.settimeout(3.0)
c.connect(('127.0.0.1', PORT))
print("[CLIENT] Conectado")

total_bytes = 0
recv_count = 0
start = time.time()
while time.time() - start < 5:
    try:
        data = c.recv(4096)
        if not data:
            print("[CLIENT] Servidor cerro la conexion")
            break
        recv_count += 1
        total_bytes += len(data)
        if recv_count <= 3:
            print(f"[CLIENT] recv #{recv_count}: {data!r}")
    except socket.timeout:
        print("[CLIENT] timeout")
        break

c.close()
print()
print("=" * 60)
print(f"RESULTADO LOOPBACK PYTHON:")
print(f"  recv() calls:  {recv_count}")
print(f"  Bytes totales: {total_bytes}")
print("=" * 60)

if total_bytes > 0:
    print()
    print(">>> TCP loopback FUNCIONA en esta PC.")
    print(">>> El problema entonces es algo especifico de Unity")
    print("    (probablemente su socket Write esta fallando silenciosamente)")
else:
    print()
    print(">>> TCP loopback NO FUNCIONA en esta PC.")
    print(">>> Es 100% firewall/antivirus interceptando localhost.")
