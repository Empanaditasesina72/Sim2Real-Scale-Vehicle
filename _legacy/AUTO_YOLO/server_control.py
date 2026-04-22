import socket
import subprocess
import os
import sys

HOST = '0.0.0.0'
PORT = 5000

proceso = None

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind((HOST, PORT))
server.listen(1)

print("📡 Servidor listo...")

while True:
    conn, addr = server.accept()
    print(f"Conectado desde {addr}")

    data = conn.recv(1024).decode().strip()
    print("Comando:", data)

    # =========================
    # START
    # =========================
    if data == "START":
        if proceso is None:
            print("🚀 Iniciando main.py")
            proceso = subprocess.Popen(["python3", "cam.py"])
        else:
            print("⚠️ Ya está corriendo")

    # =========================
    # KILL TOTAL (❌)
    # =========================
    elif data == "KILL":

        print("💀 Cerrando TODO...")

        # cerrar main si existe
        if proceso:
            proceso.kill()
            proceso = None

        # cerrar servidor
        conn.close()
        server.close()

        # matar proceso actual
        os._exit(0)

    conn.close()
