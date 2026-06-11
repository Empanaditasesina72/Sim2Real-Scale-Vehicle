# Setup — Carrito TMR 2026

## 1. Instalar dependencias del sistema

```bash
sudo apt update && sudo apt install -y \
  python3-picamera2 python3-libcamera imx500-all \
  python3-pygame python3-rpi.gpio \
  bluetooth bluez python3-dbus \
  python3-smbus2

pip3 install --break-system-packages \
  adafruit-circuitpython-pca9685 \
  adafruit-circuitpython-motor \
  adafruit-circuitpython-vl53l0x \
  adafruit-blinka \
  opencv-python-headless
```

## 2. Habilitar el bus I²C alternativo (GPIO 0 / GPIO 1)

Agregar al final de `/boot/firmware/config.txt`:

```
dtoverlay=i2c-gpio,bus=3,i2c_gpio_sda=0,i2c_gpio_scl=1,i2c_gpio_delay_us=2
```

Reiniciar y verificar: `ls /dev/i2c-*` → debe aparecer `/dev/i2c-3`

## 3. Cargar el modelo YOLOv8 en el IMX500

```bash
# Verificar modelos disponibles
ls /usr/share/imx500-models/

# El archivo esperado es:
# /usr/share/imx500-models/imx500_network_yolov8n_pp.rpk
# Si tiene otro nombre, actualizar IMX500_MODEL_PATH en config.py
```

## 4. Configurar Bluetooth del mando

```bash
bluetoothctl
  power on
  agent on
  scan on
  # Poner el mando en modo pairing (mantener PS/Xbox + Share)
  pair XX:XX:XX:XX:XX:XX
  trust XX:XX:XX:XX:XX:XX
  connect XX:XX:XX:XX:XX:XX
  exit
```

Para que se conecte automáticamente al encender la Pi, el mando debe
estar en la lista de dispositivos "trusted".

## 5. Instalar el servicio systemd

```bash
# Copiar el archivo al directorio de systemd
sudo cp /home/pi/carrito_tmr/systemd/carrito_tmr.service \
        /etc/systemd/system/

# Recargar, habilitar y arrancar
sudo systemctl daemon-reload
sudo systemctl enable carrito_tmr
sudo systemctl start  carrito_tmr

# Ver logs en tiempo real
journalctl -u carrito_tmr -f
```

## 6. Comandos útiles en operación

```bash
# Ver estado
sudo systemctl status carrito_tmr

# Detener manualmente (útil para depurar)
sudo systemctl stop carrito_tmr

# Ejecutar manualmente sin servicio (para ver prints en terminal)
cd /home/pi/carrito_tmr
python3 main.py

# Solo debug de visión sin arrancar servicio
python3 -c "
from hardware.camera_manager import CameraManager
from vision.lane_detector import LaneDetector
import time, cv2
cam = CameraManager(); cam.start(); ld = LaneDetector(debug=True)
while True:
    f = cam.get_latest_frame()
    if f:
        data = ld.process(f.image)
        cv2.imwrite('/tmp/debug.jpg', data.debug_image)
        print(data)
    time.sleep(0.1)
"
```

## 7. Calibración en pista

| Parámetro | Archivo | Qué ajustar |
|-----------|---------|-------------|
| `CAMERA_FOCAL_LENGTH_PX` | config.py | Medir con tablero de ajedrez o señal STOP a distancia conocida |
| `STEER_KP/KI/KD` | config.py | Aumentar Kp hasta que oscile, luego dividir a la mitad |
| `PARK_OVERSHOOT_SEC` | config.py | Cronometrar cuánto tarda en pasar el espacio de 60 cm |
| `PARK_REVERSE_LOCK_SEC` | config.py | Ajustar hasta que el arco sea de ~90° |
| `SPEED_STRAIGHT / SPEED_CURVE` | config.py | Velocidad máxima sin que las imágenes salgan movidas |
| `threshold` en LaneDetector | lane_detector.py | Ajustar según iluminación de la pista |

## 8. Estructura del proyecto

```
TMR2026/
├── main.py                   ← Punto de entrada (Raspberry Pi, bucle 50 Hz)
├── main_simulator.py         ← Gemelo digital (Unity / Sim2Real)
├── config.py                 ← TODOS los parámetros
├── requirements.txt
├── docs/                     ← SETUP, Sim2Real, calibración, entregas
├── hardware/
│   ├── motor.py              ← IBT-2 soft-start (ACTIVO; RPWM=18, LPWM=13)
│   ├── steering_driver.py    ← PCA9685 + MG90s + geometría Ackermann
│   ├── distance_sensor.py    ← VL53L0X hilo dedicado
│   ├── signals.py            ← Direccionales / hazard (2 Hz)
│   └── brake_light.py        ← Luz de freno
├── control/
│   ├── fsm.py                ← FSM de conducción (5 estados)
│   ├── parking_fsm.py        ← Estacionamiento en batería
│   └── pid_controller.py     ← PID genérico con anti-windup
├── vision/
│   ├── camera_stream.py      ← Picamera2 RGB→BGR (hilo)
│   ├── lane_pipeline.py      ← BEV + HSV + sliding windows (ACTIVO)
│   └── sign_detector.py      ← YOLOv8n + respaldo por color (ACTIVO)
├── autonomy/                 ← Implementaciones alternativas (no conectadas)
├── tests/                    ← pytest (FSM, señales, histéresis)
├── tools/                    ← test_camera.py, captura, evaluación YOLO
└── systemd/
    └── carrito_tmr.service   ← Auto-arranque en boot
```
