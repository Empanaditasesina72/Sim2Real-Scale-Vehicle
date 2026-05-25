"""
test_raw_socket.py — Test minimo de conectividad TCP con Unity.
Solo conecta al socket y muestra los bytes que llegan.
Si esto no recibe nada en 5s, hay un problema externo (firewall, antivirus).
"""
import socket
import time

HOST = '127.0.0.1'
PORT = 5005

print(f"[RAW] Conectando a {HOST}:{PORT}...")
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.settimeout(2.0)

try:
    s.connect((HOST, PORT))
    print("[RAW] ✓ Conectado")
except Exception as e:
    print(f"[RAW] ✗ ERROR conectando: {e}")
    exit(1)

# Enviar un comando para asegurar que Unity sabe que estamos aqui
s.sendall(b"MOTOR:0\n")
print("[RAW] Enviado: MOTOR:0\\n")

print("[RAW] Escuchando 10 segundos por bytes entrantes...")
start = time.time()
total_bytes = 0
total_recv_calls = 0
timeouts = 0

while time.time() - start < 10:
    try:
        data = s.recv(65536)
        total_recv_calls += 1
        if not data:
            print("[RAW] Socket cerrado por servidor")
            break
        total_bytes += len(data)
        # Mostrar los primeros 20 bytes del primer chunk
        if total_recv_calls <= 3:
            preview = data[:30]
            print(f"[RAW] recv #{total_recv_calls}: +{len(data)} bytes | preview: {preview!r}")
    except socket.timeout:
        timeouts += 1
        print(f"[RAW] timeout #{timeouts} (no llegaron datos en 2s)")

elapsed = time.time() - start
print()
print("=" * 60)
print(f"[RAW] RESULTADO:")
print(f"  Tiempo:        {elapsed:.1f}s")
print(f"  recv() calls:  {total_recv_calls}")
print(f"  Bytes totales: {total_bytes}")
print(f"  Timeouts:      {timeouts}")
print("=" * 60)

if total_bytes == 0:
    print()
    print(">>> DIAGNOSTICO: Unity envia pero Python NO recibe.")
    print(">>> Causas probables:")
    print("    1. Firewall de Windows bloquea puerto 5005")
    print("    2. Antivirus interfiere con loopback")
    print("    3. Algun software intercepta TCP localhost")
else:
    print()
    print(">>> Unity SI envia y Python SI recibe.")
    print(">>> El bug esta en el parser de sim_hardware_mocks.py")

s.close()
