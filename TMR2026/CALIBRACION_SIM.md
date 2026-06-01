# Calibración del Simulador Sim2Real (Unity ↔ PC)

Configuración del **simulador** (actualizada al 2026-06-01). Cuando llegue la
calibración del **carro físico real**, comparar y ajustar para que Unity replique
el comportamiento real.

---

## 1. Cámara del vehículo (Unity — `VehicleBuilder.cs`)

| Parámetro | Valor actual (sim) | Notas |
|-----------|--------------------|-------|
| Posición local | `(0, 0.22, 0.30)` | Adelante del carro y a 22 cm (= altura real del Pi) |
| Rotación | `Euler(10, 0, 0)` | 10° hacia abajo |
| FOV | `60` | Igual que la cámara del Pi físico |
| Near / Far | `0.01 / 20` | |
| Fondo | gris `#8A8A8A` | No blanco (evita "pantalla blanca") |
| RenderTexture | `320×240` | Se reescala a 640×480 en el PC |

**Aprendizaje clave:** el modelo 3D del carro tapaba la cámara. Solución: cámara
ADELANTE del carro, mirando ligeramente hacia abajo (si va horizontal ve el cielo).

---

## 2. Pista (Unity — `SceneBuilder.cs`)

| Elemento | Valor |
|----------|-------|
| Largo de pista | `60 m` (centro en `z = 30`) |
| Ancho de carril (línea a línea) | `54 cm` → líneas en `x = ±0.27` |
| Suelo | gris `#6E6E6E` (oscuro, contrasta con las líneas blancas) |
| Líneas (izq/der) | **blancas** `#FFFFFF`, 4 cm de ancho |
| Central punteada | blanca, segmentos cada `0.8 m` |
| STOP | en `z = 12 m`, derecha (`x = 0.32`), **panel rojo sólido** (sin imagen) |
| Zona de estacionamiento | en `z = 30 m` (autos + hueco) |

---

## 3. Filtro de carril HSV (`vision/lane_pipeline.py`) — configurable

```python
# Simulador Unity (líneas MUY brillantes sobre fondo oscuro):
hsv_white_lo = [0,  0, 200]
hsv_white_hi = [179, 40, 255]

# Pi físico (default de clase, luz media-baja):
HSV_WHITE_LO = [0,  0, 130]
HSV_WHITE_HI = [179, 60, 255]
```
> El HSV es un parámetro del constructor: cada cámara usa su propio umbral.

## 4. Bird's-Eye View (BEV) — SOLO simulador (`main_simulator.py`)

```python
roi_frac = 0.30        # el BEV mira más lejos (se ven los 2 carriles)
# bev_src_ratio NO se sobreescribe → usa el trapecio por defecto de LanePipeline
```

## 5. Sesgo de carril

```python
right_bias = 0.75   # sim (0.5=centro, 1.0=línea derecha). Pi default = 0.70.
```

## 6. Dirección (servo) — INVERSIÓN

El servo del carro físico está montado al revés. El simulador lo replica:
```python
# MockSteeringDriver
STEERING_INVERTED = True
physical = 2*90 - angle_logico   # se envía a Unity el ángulo físico
```
**Sin esta inversión el carro gira al lado equivocado** (se va a la línea izquierda
en vez del carril derecho).

## 7. PID de dirección (error de carril → ángulo servo)

```python
PID_KP = 0.08
PID_KI = 0.002
PID_KD = 0.025
```

## 8. FSM — parada en STOP

| Parámetro | Valor |
|-----------|-------|
| Freno por cámara | ~320 mm (frena cuando la señal está cerca) |
| `STOP_TARGET_MM` | 270 mm (regla TMR: 270 ± 30) |
| `ESPERA_S` | 5.0 s |
| Altura real octágono STOP | `0.04 m` (4 cm) |
| `CAMERA_FOCAL_LENGTH_PX` | 490 |

---

## CÓMO REPLICAR EL CARRO FÍSICO EN UNITY

Cuando tengas la foto/datos del carro físico:
1. **Cámara**: medir altura real (cm) y ángulo de inclinación → ajustar
   `cameraMount.localPosition.y` y `Euler(x,...)`.
2. **Carril**: confirmar ancho real (¿54 cm?) → ajustar la `x` de las líneas.
3. **Vista BEV**: comparar la vista del Pi vs la de Unity; ajustar `roi_frac`/BEV
   hasta que las líneas queden verticales en el ojo de águila.
4. **Colores**: igualar el gris de la pista y el brillo de las líneas al real.
5. Verificar que `err` (px) tenga el mismo signo y magnitud en sim y real.
