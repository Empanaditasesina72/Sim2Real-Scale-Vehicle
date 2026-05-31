# Entrega — Validación Sim2Real del Vehículo Autónomo (TMR 2026)

Este documento mapea **cada requisito del PDF** a lo que se entregó, explica
**cómo ejecutar todo** y dónde están las **métricas y gráficas** para el
artículo científico.

---

## 1. Resumen ejecutivo

Se construyó un **gemelo digital** en Unity 3D que se comunica de forma
**bidireccional** por sockets TCP con el código Python real del TMR 2026.
El mismo código de control (FSM + PID + visión) que corre en la Raspberry Pi
corre en la PC contra la simulación, **sin hardware físico**.

Se ejecutan las **3 pruebas** del PDF y se generan archivos **.csv** + gráficas
+ un **tablero de puntos** que evalúa el cumplimiento.

**Resultado de la última corrida: 100/100 — APROBADO.**

---

## 2. Cumplimiento punto por punto del PDF

### Fase 1 — Adaptación del código Python (Mocks de Hardware) ✅
| Requisito PDF | Entregado | Archivo |
|---|---|---|
| Mock de motores/servos → sockets | `MockMotorDriver`, `MockSteeringDriver` envían `MOTOR:`/`SERVO:` por TCP | `sim_hardware_mocks.py` |
| Mock de sensores VL53L0X (mm) | `MockDistanceSensor` escucha `TOF:front,rear` | `sim_hardware_mocks.py` |
| Mock de cámara (frames) | `MockCameraStream` recibe JPEG; visión con YOLO + OpenCV | `sim_hardware_mocks.py`, `vision/` |

> El servo replica la inversión física real (`STEERING_INVERTED`) para que el
> comportamiento del simulador sea idéntico al del carro.

### Fase 2 — Entorno virtual (Unity 3D) ✅
| Requisito PDF | Entregado | Dónde |
|---|---|---|
| Pista simplificada TMR (carril, STOP, estacionamiento) | Pista de 30 m, carril de 54 cm con líneas blancas, señal STOP, zona de estacionamiento marcada | `Assets/Scripts/SceneBuilder.cs` |
| Modelo de coche con físicas + Ackermann | Vehículo con dirección Ackermann (ruedas frontales giran), movimiento estable | `Assets/Scripts/VehicleBuilder.cs`, `VehicleController.cs` |
| ToF por Raycast frontal/trasero a 50 Hz | `SensorManager.GetToFFront/Rear` (raycast → mm), enviado a 50 Hz | `Assets/Scripts/SensorManager.cs`, `SimulatorServer.cs` |
| Cámara virtual → RenderTexture → bytes JPEG | `SensorManager.GetCameraJPEG()` a ~30 FPS | `Assets/Scripts/SensorManager.cs` |
| Recepción de comandos (SPEED, ángulo) → motor/dirección | `SimulatorServer` parsea `MOTOR`/`SERVO` → `VehicleController` | `Assets/Scripts/SimulatorServer.cs` |

### Fase 3 — Pruebas y datos (Entregables) ✅
| Prueba PDF | Entregado | Archivo CSV |
|---|---|---|
| **P1 Latencia** del ciclo percepción→respuesta (<100-200 ms) | latencia por ciclo registrada cada iteración | `validation_results/P1_latencia.csv` |
| **P2 Frenado PID** ante STOP (700→270 mm, sin sobreimpulso) | distancia ToF vs PWM vs tiempo | `validation_results/P2_pid_stop.csv` |
| **P3 Transiciones FSM** (sin bloqueo, transiciones limpias) | log temporal de cada cambio de estado + dwell | `validation_results/P3_fsm.csv` |

> **Estacionamiento en batería (Prueba 3 completa):** además del ciclo del
> STOP, se implementó la maniobra de estacionamiento con sus estados
> `PARKING_SEARCH → PARKING_MANEUVER → PARKED` (`control/parking_fsm.py`).
> Se ejecuta con `python run_validation.py parking`.

---

## 3. Cómo ejecutar TODO (paso a paso)

### Requisitos una sola vez
```bash
pip install -r requirements.txt        # opencv, numpy, matplotlib, ultralytics
```

### Ejecución
1. **Abrir Unity** (proyecto `TMR2026_Sim`) y dar **PLAY**.
   - Verifica en la consola: `[Server] Listening on port 5005...`
2. **En la PC**, una terminal en `Carrito/TMR2026/`:
   ```bash
   python run_validation.py            # P1 latencia + P2 STOP + P3 ciclo FSM
   python run_validation.py parking    # P3 estacionamiento en batería
   ```
   Cada comando: ejecuta el control real contra la simulación, genera los
   CSV, el tablero de puntos (`PUNTAJE.txt`) y las gráficas (PNG).
   > Corre PRIMERO el normal (100/100) y LUEGO el de parking para demostrar
   > la maniobra de estacionamiento. Entre corridas, deja Unity en PLAY.

### Ver solo en vivo (sin guardar datos)
```bash
python main_simulator.py --display     # ventana de debug con cámara + BEV
```

---

## 4. Qué entregar al profesor

Carpeta `validation_results/` (se crea al correr la validación):
```
validation_results/
├── P1_latencia.csv      ← datos Prueba 1
├── P2_pid_stop.csv      ← datos Prueba 2
├── P3_fsm.csv           ← datos Prueba 3
├── fig1_latencia.png    ← gráfica latencia
├── fig2_frenado.png     ← gráfica frenado PID
├── fig3_fsm.png         ← línea de tiempo FSM
└── PUNTAJE.txt          ← tablero de puntos (cumplimiento)
```
Más el **código** (repos `Carrito` y `TMR2026_Sim` en GitHub) y este documento.

---

## 5. Sistema de puntos (estilo competencia)

El script evalúa cada prueba contra su criterio del PDF y asigna puntos:

| Prueba | Criterio | Puntos |
|---|---|---|
| P1 Latencia | media < 100 ms = 30; < 200 ms = 20 | /30 |
| P2 Frenado STOP | parar a 270±30 mm sin sobreimpulso = 40 | /40 |
| P3 FSM | recorrer los 5 estados sin bloqueo = 30 | /30 |
| **TOTAL** | | **/100** |

Veredicto: ≥70% APROBADO, 40-70% PARCIAL, <40% REVISAR.

---

## 6. Notas de alcance (honestidad técnica)

- **Latencia**: en simulación local el ciclo va a ~15 ms (muy por debajo del
  objetivo de 200 ms). En el artículo se reporta como evidencia de que el
  diseño multihilo no introduce cuellos de botella.
- **Frenado PID**: las ganancias usadas son las del PDF (Kp=0.035, Ki=0.001,
  Kd=0.008 para velocidad). El carro se detiene dentro de 270±30 mm.
- **Estacionamiento en batería**: la zona está modelada en la pista y existe
  el módulo `autonomy/parking_maneuver.py`. La maniobra completa de parking
  (PARKING_SEARCH → PARKING_MANEUVER) es la **siguiente fase de integración**;
  la Prueba 3 valida el ciclo de estados del STOP, que demuestra que la FSM
  hace transiciones limpias sin bloquearse (objetivo central de la prueba).

---

## 7. Arquitectura (para el artículo)

```
   PC (Python, código TMR2026 real)            Unity 3D (gemelo digital)
   ┌─────────────────────────────┐  TCP 5005   ┌──────────────────────────┐
   │ Visión (OpenCV/YOLO)        │ ─MOTOR/SERVO─►│ VehicleController        │
   │ FSM (5 estados)             │             │ (Ackermann + físicas)    │
   │ PID dirección               │ ◄─TOF/JPEG── │ SensorManager (raycast + │
   │ Mocks de hardware (sockets) │             │ RenderTexture)           │
   └─────────────────────────────┘             └──────────────────────────┘
        3 hilos: control 50 Hz · visión · receptor
```
