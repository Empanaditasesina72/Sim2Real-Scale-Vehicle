# Sistema Autónomo de Vehículo - Arquitectura Refactorizada

## Estructura del Sistema

```
CAMARA/
   │
   ├── YOLO/
   │     ├── objeto/
   │     │     └── objeto_detector.py      # Detección de objetos (stop, señales, etc.)
   │     │
   │     ├── semaforo/
   │     │     └── semaforo_detector.py    # Detección y clasificación de semáforos
   │     │
   │     └── señal/
   │           └── senal_detector.py       # Detección de señales de tráfico
   │
   └── lane_detection/
         ├── lane_center/
         │     └── lane_center_detector.py # Detección del centro del carril
         │
         └── lane_curvature/
               └── lane_curvature_detector.py # Cálculo de curvatura del carril

PERCEPTION_OUTPUT/
   └── perception_output.py                # Agregación de toda la información de percepción

STATE_MACHINE/
   └── state_machine.py                    # Máquina de estados con Ackerman steering

CONTROL/
   └── vehicle_controller.py               # Controladores PID para velocidad y dirección
```

## Funcionalidades Implementadas

### 1. Percepción (CAMARA)
- **YOLO Object Detection**: Detecta objetos, semáforos y señales
- **Lane Detection**: Detecta líneas de carril y calcula centro y curvatura
- **Color Classification**: Clasifica colores de semáforos por HSV

### 2. Salida de Percepción (PERCEPTION_OUTPUT)
- `distancia_obj`: Distancia al objeto más cercano
- `tipo_obj`: Tipo de objeto detectado
- `lane_error`: Error de posicionamiento en el carril
- `semaforo`: Estado del semáforo
- `interseccion`: Indicador de intersección

### 3. Máquina de Estados (STATE_MACHINE)
Estados implementados:
- `RUTA_LIBRE`: Navegación normal
- `STOP_LEJOS/CERCA`: Frenado por obstáculos
- `EMERGENCIA`: Stop inmediato
- `SEMAFORO_ROJO/AMARILLO`: Control por semáforos
- `PARKING_SEARCH/MANEUVER`: Estacionamiento autónomo
- `INTERSECTION_STOP`: Stop en intersecciones
- `LANE_CHANGE_LEFT/RIGHT`: Cambio de carril
- `OBSTACLE_AVOIDANCE`: Evasión de obstáculos

### 4. Control (CONTROL)
- **PID Velocity Control**: Control de velocidad
- **Ackerman Steering**: Dirección con geometría de Ackerman
- **Adaptive Control**: Ajuste dinámico de ganancias PID

## Ecuación de Ackerman

La dirección implementa la geometría de Ackerman para un giro preciso:

```
tan(δ_outer) = L / (L/tan(δ_center) + W/2)
tan(δ_inner) = L / (L/tan(δ_center) - W/2)
```

Donde:
- `L`: Distancia entre ejes (wheelbase)
- `W`: Ancho de vía (track width)
- `δ`: Ángulo de dirección

## Instalación y Uso

1. Instalar dependencias:
```bash
pip install ultralytics opencv-python numpy matplotlib
```

2. Ejecutar el sistema:
```bash
python main_refactored.py
```

## Parámetros Configurables

### Vehículo
- `wheelbase`: 0.25m (distancia entre ejes)
- `track_width`: 0.18m (ancho de vía)
- `max_steering_angle`: 30° (ángulo máximo de dirección)

### Control PID
- **Velocidad**: Kp=0.6, Ki=0.2, Kd=0.05
- **Dirección**: Kp=0.8, Ki=0.1, Kd=0.2

### Umbrales de Seguridad
- `STOP_CERCA`: 25cm
- `STOP_LEJOS`: 50cm
- `TOF_FRENO_DELANTE`: 300mm
- `TOF_ALERTA_ATRAS`: 150mm

## Estados de Parking

El sistema incluye funcionalidad de estacionamiento autónomo:

1. `PARKING_SEARCH`: Busca espacio de estacionamiento
2. `PARKING_MANEUVER`: Ejecuta maniobra de estacionamiento con Ackerman steering

## Extensiones Futuras

- Integración con GPS para navegación
- Control de crucero adaptativo (ACC)
- Sistema de advertencia de colisión frontal (FCW)
- Asistente de mantenimiento de carril (LKA)
- Detección de peatones y ciclistas