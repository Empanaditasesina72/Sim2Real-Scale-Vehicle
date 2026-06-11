# Detección en el NPU de la cámara (Sony IMX500)

La Pi AI Camera no es solo una cámara: el sensor IMX500 trae un **acelerador
neuronal integrado**. Con el modelo cargado ahí, la inferencia ocurre *dentro
de la cámara* y la Pi recibe, junto a cada frame, los tensores de salida en la
metadata. Resultado: **detección a la velocidad de la cámara con ~0 % de CPU**,
dejando los 4 núcleos para el carril, la FSM y el bucle de control de 50 Hz.

## Cadena de respaldo (no hay forma de quedarse sin detector)

```
NPU IMX500 (.rpk)  →  CPU NCNN  →  CPU PyTorch (.pt)  →  detector por COLOR
     on-chip           ~25 ms          ~120 ms              solo STOP
```

`main.py:_build_vision()` elige solo: si `config.py:USE_IMX500_NPU=True` y
existe `weights/tmr_signs_imx500.rpk`, usa el NPU; cualquier falla cae al
camino CPU **sin interrumpir el arranque**.

## Generar el .rpk (una sola vez, EN LA PI)

El converter de Sony solo existe en Linux, así que este paso se hace en la
Raspberry (a diferencia del export NCNN, que ya viene hecho en el repo).

```bash
# 1. Prerequisitos (una vez)
sudo apt install -y imx500-all imx500-tools default-jre
pip3 install --break-system-packages model-compression-toolkit "imx500-converter[pt]"

# 2. Exportar (cuantización INT8 calibrada con traffic_lights/)
cd ~/Carrito/TMR2026
python tools/export_imx500.py            # 15-60 min en la Pi 5 — una vez
```

El script deja `weights/tmr_signs_imx500.rpk` + `weights/tmr_signs_imx500_labels.txt`
y en el siguiente `python main.py` verás:

```
[NPU] Cargando modelo en el IMX500: weights/tmr_signs_imx500.rpk
[VISION] Backend: NPU IMX500 (inferencia on-chip)
```

> 💾 Opcional: respalda el .rpk en GitHub desde la Pi —
> `git add weights/tmr_signs_imx500* && git commit -m "rpk IMX500" && git push`.

## Qué cambia y qué NO cambia

| | Camino CPU (NCNN) | NPU IMX500 |
|---|---|---|
| Dónde corre el modelo | CPU ARM (hilo a 15 Hz) | Dentro del sensor (~30 Hz) |
| CPU usada por la detección | ~30-40 % de un núcleo | ~0 % (solo parseo de tensores) |
| Precisión | FP16 (= al .pt) | INT8 cuantizado (≈, calibrar conf) |
| Gating de la FSM | solo `stop_sign`/`red` frenan | **idéntico** |
| Histéresis 3 frames | sí | **idéntica** |
| Respaldo por color (STOP) | sí | **idéntico** |
| Distancia pinhole por clase | sí | **idéntica** |
| `LanePipeline` / PID / FSM | — | **sin cambios** (mismo frame BGR) |

La FSM y `main.py` no distinguen el backend: `IMX500CameraStream` expone la
misma API que `CameraStream` + `SignDetector` (`get_frame`, `get_detections`,
`has_sign`, `closest_sign`, …).

## Calibración en pista

- **`config.py:IMX500_CONF`** (default 0.55) — la cuantización INT8 puede
  mover las confianzas respecto al `.pt`. Si el NPU no ve la señal de lejos,
  baja hacia 0.40; si inventa señales, sube hacia 0.65.
- Verifica en modo **VISION** (`--display`): los bboxes y el panel de objetos
  salen igual que con el camino CPU.

## Volver al camino CPU

```python
# config.py
USE_IMX500_NPU = False
```
(o simplemente borra/renombra el .rpk — el fallback es automático).

## Problemas comunes

| Síntoma | Causa / solución |
|---|---|
| `imxconv-pt: command not found` | `pip3 install "imx500-converter[pt]"` |
| El converter pide Java | `sudo apt install default-jre` |
| Export no produce .rpk | falta `imx500-tools` (apt) |
| `[VISION] NPU no disponible (...)` al arrancar | revisa el mensaje — el sistema ya siguió con CPU; el carro funciona igual |
| Confianzas raras tras cuantizar | recalibra `IMX500_CONF`; si no alcanza, re-exporta con `--fraction 1.0` |
