# DriveNet — plan de captura de datos (paso a paso)

> El detector de señales ya se entrena con su dataset (`traffic_lights/`).
> DriveNet (el **manejo**) NO tiene datos todavía: es behavioral cloning, así que
> sin pares `(imagen → error_px)` no hay nada que entrenar. Este documento es el
> plan concreto para generar esos datos. Referencia técnica completa:
> [`DRIVE_NET.md`](DRIVE_NET.md).

## Decisión: de dónde salen los datos

Hay dos fuentes. **No** son excluyentes — lo ideal es mezclar ambas.

| Fuente | Quién etiqueta | Requiere | Cuándo usarla |
|---|---|---|---|
| **A. Pista física** (cámara Pi) | el pipeline clásico (`LanePipeline`) como "experto" | la Pi + la pista armada | **Tu camino principal** (no hay gap sim2real) |
| **B. Simulador Unity** | autopiloto del sim + pipeline clásico | Unity corriendo en la PC | Cuando puedas correr el sim (mucha variedad gratis) |

Como ahora **no** puedes correr el simulador, el plan base es **A (pista física)**.
La C (sintética) ya está validada y se usa solo como relleno/regularización.

---

## Camino A — pista física (el principal)

Todo esto se hace **EN LA PI** (es la que tiene cámara). El servicio systemd debe
estar parado para liberar la cámara (`sudo systemctl stop carrito_tmr`).

### A1. Capturar fotos variadas de la pista

```bash
cd ~/Carrito
sudo systemctl stop carrito_tmr           # liberar la cámara
python TMR2026/tools/capture_track.py --auto 0.5   # 1 foto cada 0.5 s
```

Mientras captura, **mueve el carro a mano por la pista** cubriendo variedad — esto
es lo que decide si el modelo generaliza:

- **Posiciones laterales:** centrado, pegado a la izquierda, pegado a la derecha
  (para que aprenda a corregir, no solo el caso perfecto).
- **Curvas** a ambos lados, además de rectas.
- **Iluminación:** con luz de día, con lámpara, con linterna de celular, sombras.
- **Apunta a ~1500–3000 fotos** en total. Más variedad > más cantidad.

Salida: `TMR2026/tools/captures/track_*.jpg`. Para separar por condición, puedes
mover lotes a carpetas (`captures_dia/`, `captures_curvas/`, …) y etiquetar cada
una por separado en el siguiente paso.

> Verifica antes que el pipeline clásico VE la línea en esas fotos
> (`python TMR2026/tools/test_camera.py --no-yolo`). Si la máscara HSV no agarra la
> línea, ajústala (ver "Vision tuning notes" en CLAUDE.md) ANTES de etiquetar —
> el experto solo etiqueta bien lo que detecta.

### A2. Convertir las fotos en un "tub" etiquetado

```bash
python TMR2026/tools/record_driving.py --source images \
    --path TMR2026/tools/captures --out TMR2026/datasets/track_real
```

Esto corre el pipeline clásico sobre cada foto y escribe
`datasets/track_real/{frames/, labels.csv, tub.json}` con el `error_px` experto.

### A3. Mover el dataset a la PC para entrenar

Los `datasets/` están en `.gitignore` (pueden ser grandes), así que **no** van por
git. Pásalos por USB, `scp` o carpeta compartida:

```bash
# desde la PC, jalando de la Pi por scp (ejemplo)
scp -r angel01@<IP_PI>:~/Carrito/TMR2026/datasets/track_real \
       C:/Users/Angel/Documents/GitHub/Carrito/TMR2026/datasets/
```

---

## Camino B — simulador (cuando puedas correrlo)

En la PC, con Unity escuchando en `127.0.0.1:5005` (igual que `main_simulator.py`).
El autopiloto maneja solo y etiqueta solo:

```bash
python TMR2026/tools/record_driving.py --source sim --max 5000 --throttle 18 \
    --out TMR2026/datasets/sim_trackA
# repite en varias pistas/luces: sim_trackB, sim_trackC, ...
```

---

## Entrenar (EN LA PC, GPU) — igual para A, B o mezcla

```bash
python TMR2026/tools/train_drive.py \
    --data TMR2026/datasets/track_real,TMR2026/datasets/drive_synth \
    --val  TMR2026/datasets/track_real_val \
    --epochs 60 --batch 64 --workers 4 --device 0
```

- Mezclar el tub sintético (`drive_synth`) regulariza contra sobreajustar una sola
  pista.
- Genera `weights/drive_net.pt` + `.json` + `drive_train.png`. **Mira que el
  `val_RMSE` en px baje.**
- Añade `--workers 4`: `train_drive` está limitado por carga de datos (la red es
  diminuta), ahí está la ganancia, no en la GPU.

## Evaluar

```bash
python TMR2026/tools/test_drive_net.py --tub TMR2026/datasets/track_real_val
# genera RMSE + un montaje anotado para revisar a ojo
```

## Exportar y desplegar a la Pi

```bash
# PC: export portable
python TMR2026/tools/export_drive.py            # TorchScript (+ NCNN con pnnx)
# commit del .pt/.json -> tú haces push -> Pi hace pull
```

En `config.py` (cuando el RMSE convenza):
```python
USE_DRIVE_NET = True
```
`main.py` / `main_simulator.py` cambian `LanePipeline` por `DriveNet`
automáticamente (con fallback si faltan pesos). DriveNet corre en la **CPU de la
Pi** (~1 MB); la NPU IMX500 sigue dedicada a las señales.

---

## Checklist rápido

- [ ] A1 — capturar ≥1500 fotos variadas (`capture_track.py --auto 0.5`)
- [ ] A1.5 — verificar que el HSV ve la línea (`test_camera.py --no-yolo`)
- [ ] A2 — etiquetar a tub (`record_driving.py --source images`)
- [ ] (opcional) separar un `track_real_val` para validación
- [ ] A3 — pasar `datasets/` a la PC
- [ ] Entrenar en GPU (`train_drive.py --device 0 --workers 4`)
- [ ] Evaluar RMSE (`test_drive_net.py`)
- [ ] Exportar (`export_drive.py`) → commit → push → Pi pull
- [ ] `USE_DRIVE_NET = True` y probar en AUTONOMOUS con `--display`
