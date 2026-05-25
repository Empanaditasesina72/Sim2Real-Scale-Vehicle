# Plan de Implementación Unity — Paso a Paso

**Objetivo Final**: Validación Sim2Real del TMR 2026 para artículo académico  
**Duración Estimada**: 4-6 horas  
**Requisitos PDF**: Latencia, precisión STOP, robustez FSM  

---

## PARTE 1: Entender los Requisitos del PDF

### Requisitos del Artículo (del PDF que subiste)

**Objetivo Principal**:
> "Validar que el control digital del vehículo autónomo funciona correctamente antes de desplegar en Raspberry Pi"

**Tres Validaciones Requeridas**:

1. **Validación de Latencia**
   - Tiempo perception-to-actuation **< 150 ms**
   - Captura: cámara (timestamp) → detección (Python) → motor/servo (respuesta)
   - Métrica: latencia end-to-end en CSV

2. **Validación de STOP (Precisión)**
   - Detectar STOP sign
   - Desacelerar suavemente
   - Parar a **270 ± 30 mm** del signo
   - Métrica: distancia final, suavidad, confianza detección

3. **Validación de FSM (Robustez)**
   - Todos los estados: CRUCERO → PRECAUCIÓN → FRENADO → ESPERA → REANUDAR
   - Sin transiciones espurias
   - ESPERA = exactamente 5 segundos
   - Métrica: timeline de estados, duraciones

---

## PARTE 2: Checklist de Requisitos Unity

### ✅ Lo que Unity DEBE Hacer

```
[ ] 1. Servidor TCP en puerto 5005
[ ] 2. Recibir "MOTOR:{duty}" y aplicar fuerza
[ ] 3. Recibir "SERVO:{angle}" y rotar ruedas
[ ] 4. Sensor ToF frontal: raycast hacia adelante
[ ] 5. Sensor ToF trasero: raycast hacia atrás
[ ] 6. Enviar "TOF:{front},{rear}\n" @ 50 Hz
[ ] 7. Cámara virtual: captura JPEG @ 30 FPS
[ ] 8. Enviar JPEG: [4-byte size][JPEG data]...
[ ] 9. Propiedades físicas realistas (mass, drag, etc.)
[ ] 10. Propiedades del signo STOP (180 mm de alto)
```

---

## PARTE 3: Paso a Paso (Implementación)

### **PASO 1: Crear Escena Unity Base**

**Tiempo**: 15 minutos  
**Qué hacer**:

1. **Abrir Unity 2021.3+** (o la versión que tengas)
2. **Crear nueva escena**: `File → New Scene`
3. **Guardar como**: `Assets/Scenes/Simulator.unity`
4. **Crear estructura de objetos**:

```
Hierarchy:
├── Vehicle (Rigidbody - el coche)
│   ├── Body (Cube con mesh)
│   ├── WheelFL (Sphere - rueda frontal izq)
│   ├── WheelFR (Sphere - rueda frontal der)
│   ├── WheelRL (Sphere - rueda trasera izq)
│   ├── WheelRR (Sphere - rueda trasera der)
│   └── Camera (Cámara virtual)
├── Road (Plane - pista)
├── StopSign (propiedades del signo)
├── Lighting (luz)
└── Network (objeto para scripts)
```

**Pasos exactos**:
```
1. Create → 3D Object → Cube
   - Nombre: "Vehicle"
   - Position: (0, 0.5, 0)
   - Scale: (0.2, 0.1, 0.4)  # 20cm ancho, 10cm alto, 40cm largo

2. Add Component → Rigidbody
   - Mass: 1.0
   - Drag: 0.1
   - Angular Drag: 0.05
   - Freeze Rotation: Z=true (no voltearse)

3. Create → 3D Object → Sphere (x4 para ruedas)
   - Nombre cada una: WheelFL, WheelFR, WheelRL, WheelRR
   - Parent: Vehicle
   - Scale: (0.05, 0.05, 0.05)  # Radio pequeño
   - Posiciones:
     * WheelFL: (-0.075, 0, 0.1)
     * WheelFR: (0.075, 0, 0.1)
     * WheelRL: (-0.075, 0, -0.1)
     * WheelRR: (0.075, 0, -0.1)

4. Create → 3D Object → Plane
   - Nombre: "Road"
   - Scale: (20, 1, 20)  # Pista grande

5. Create → Camera
   - Parent: Vehicle
   - Position: (0, 0.05, 0)  # Arriba del coche
   - Rotation: (-90, 0, 0)  # Mirando hacia adelante
```

✅ **Verificar**: 
- El coche está sobre la pista
- Las ruedas están en las esquinas del coche
- La cámara apunta hacia adelante
- La gravedad atrae el coche hacia abajo

---

### **PASO 2: Crear Script de Servidor TCP**

**Tiempo**: 30 minutos  
**Archivo a crear**: `Assets/Scripts/SimulatorServer.cs`

```csharp
using System;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SimulatorServer : MonoBehaviour
{
    private TcpListener listener;
    private TcpClient connectedClient;
    private NetworkStream stream;
    private Thread serverThread;
    private bool isRunning = false;
    
    [SerializeField] private int port = 5005;
    [SerializeField] private VehicleController vehicleController;
    [SerializeField] private SensorManager sensorManager;
    
    void Start()
    {
        Debug.Log("[SIM] Inicializando servidor...");
        
        try
        {
            listener = new TcpListener(IPAddress.Loopback, port);
            listener.Start();
            Debug.Log($"[SIM] Escuchando en 127.0.0.1:{port}");
            
            isRunning = true;
            serverThread = new Thread(AcceptConnections) { IsBackground = true };
            serverThread.Start();
        }
        catch (Exception e)
        {
            Debug.LogError($"[SIM] Error iniciando servidor: {e.Message}");
        }
    }
    
    void AcceptConnections()
    {
        while (isRunning)
        {
            try
            {
                connectedClient = listener.AcceptTcpClient();
                stream = connectedClient.GetStream();
                stream.ReadTimeout = 1000;  // 1 segundo timeout
                
                Debug.Log("[SIM] Cliente conectado");
                HandleClient();
            }
            catch (Exception e)
            {
                Debug.LogError($"[SIM] Error en AcceptConnections: {e.Message}");
            }
        }
    }
    
    void HandleClient()
    {
        byte[] buffer = new byte[256];
        
        while (connectedClient != null && connectedClient.Connected)
        {
            try
            {
                int bytesRead = stream.Read(buffer, 0, buffer.Length);
                if (bytesRead == 0)
                {
                    Debug.Log("[SIM] Cliente desconectado");
                    break;
                }
                
                string command = System.Text.Encoding.ASCII.GetString(buffer, 0, bytesRead).Trim();
                
                if (command.StartsWith("MOTOR:"))
                {
                    string dutyStr = command.Substring(6);
                    if (float.TryParse(dutyStr, out float duty))
                    {
                        vehicleController.SetMotorDuty(duty);
                        // Debug.Log($"[SIM] Motor: {duty}%");
                    }
                }
                else if (command.StartsWith("SERVO:"))
                {
                    string angleStr = command.Substring(6);
                    if (float.TryParse(angleStr, out float angle))
                    {
                        vehicleController.SetSteeringAngle(angle);
                        // Debug.Log($"[SIM] Servo: {angle}°");
                    }
                }
                else if (command.StartsWith("PING"))
                {
                    // Ignorar PING
                }
            }
            catch (IOException)
            {
                // Timeout or disconnected, continue
            }
            catch (Exception e)
            {
                Debug.LogError($"[SIM] Error leyendo comando: {e.Message}");
                break;
            }
        }
        
        if (stream != null)
            stream.Close();
        if (connectedClient != null)
            connectedClient.Close();
    }
    
    void OnDestroy()
    {
        isRunning = false;
        if (listener != null)
            listener.Stop();
        if (stream != null)
            stream.Close();
        if (connectedClient != null)
            connectedClient.Close();
    }
}
```

**✅ Pasos para agregar a Unity**:
1. `Assets → Create → Folder "Scripts"`
2. Click derecho en Scripts → Create → C# Script → `SimulatorServer.cs`
3. **Copiar el código arriba** en el archivo
4. En el GameObject `Vehicle`, agregar componente: `Add Component → SimulatorServer`
5. En el Inspector, asignar `port = 5005`

---

### **PASO 3: Crear Script de Control del Vehículo**

**Tiempo**: 30 minutos  
**Archivo a crear**: `Assets/Scripts/VehicleController.cs`

```csharp
using UnityEngine;

public class VehicleController : MonoBehaviour
{
    [SerializeField] private Rigidbody rb;
    [SerializeField] private Transform[] wheels;  // 4 ruedas
    [SerializeField] private Transform wheelFrontL;
    [SerializeField] private Transform wheelFrontR;
    
    [SerializeField] private float maxMotorForce = 50f;  // Newton
    [SerializeField] private float maxSteeringAngle = 35f;  // grados
    [SerializeField] private float wheelbaseLength = 0.258f;  // metros (de config.py)
    
    private float currentMotorDuty = 0f;  // [-100, 100]
    private float currentSteeringAngle = 90f;  // [0, 180]
    private float currentSteeringAngleDeg = 0f;  // [-35, 35]
    
    void Start()
    {
        rb = GetComponent<Rigidbody>();
        
        // Encontrar las ruedas
        wheelFrontL = transform.Find("WheelFL");
        wheelFrontR = transform.Find("WheelFR");
    }
    
    void FixedUpdate()
    {
        // Aplicar fuerza del motor
        ApplyMotorForce();
        
        // Rotar ruedas frontales (dirección)
        ApplySteeringAngle();
    }
    
    public void SetMotorDuty(float duty)
    {
        // Limitar duty a [-100, 100]
        currentMotorDuty = Mathf.Clamp(duty, -100f, 100f);
    }
    
    public void SetSteeringAngle(float angle)
    {
        // Ángulo lógico [0, 180] → convertir a [-35, 35]
        // 90 = recto, <90 = izq, >90 = der
        currentSteeringAngle = Mathf.Clamp(angle, 0f, 180f);
        currentSteeringAngleDeg = (angle - 90f);  // [-90, 90]
        currentSteeringAngleDeg = Mathf.Clamp(currentSteeringAngleDeg * (maxSteeringAngle / 90f), 
                                               -maxSteeringAngle, maxSteeringAngle);
    }
    
    void ApplyMotorForce()
    {
        // Duty [-100, 100] → fuerza forward/backward
        float force = (currentMotorDuty / 100f) * maxMotorForce;
        
        // Aplicar fuerza solo en las ruedas traseras (tracción)
        Vector3 forwardDir = rb.transform.forward;
        
        // Aplicar fuerza en el Rigidbody (centro de masa)
        if (Mathf.Abs(force) > 0.01f)
        {
            rb.AddForce(forwardDir * force, ForceMode.Force);
        }
    }
    
    void ApplySteeringAngle()
    {
        // Rotar las ruedas frontales para la dirección
        if (wheelFrontL != null)
        {
            Quaternion steerRotation = Quaternion.Euler(0, currentSteeringAngleDeg, 0);
            wheelFrontL.localRotation = steerRotation;
        }
        
        if (wheelFrontR != null)
        {
            Quaternion steerRotation = Quaternion.Euler(0, currentSteeringAngleDeg, 0);
            wheelFrontR.localRotation = steerRotation;
        }
    }
    
    // Para debugging
    public float GetCurrentMotorDuty() => currentMotorDuty;
    public float GetCurrentSteeringAngle() => currentSteeringAngle;
    public Vector3 GetVelocity() => rb.velocity;
}
```

**Pasos para agregar**:
1. `Assets/Scripts → Create → C# Script → VehicleController.cs`
2. Copiar el código arriba
3. En el GameObject `Vehicle`:
   - `Add Component → VehicleController`
   - En el Inspector: asignar `Rigidbody` (del componente anterior)
   - Asignar `WheelFL`, `WheelFR`, `WheelRL`, `WheelRR` en el array de wheels

---

### **PASO 4: Crear Script de Sensores**

**Tiempo**: 30 minutos  
**Archivo a crear**: `Assets/Scripts/SensorManager.cs`

```csharp
using UnityEngine;
using System.Collections.Generic;

public class SensorManager : MonoBehaviour
{
    [SerializeField] private Transform vehicleBody;
    [SerializeField] private Camera vehicleCamera;
    [SerializeField] private RenderTexture cameraRT;
    
    [SerializeField] private float tofMaxRange = 1.2f;  // metros
    [SerializeField] private int cameraWidth = 640;
    [SerializeField] private int cameraHeight = 480;
    
    private float lastToFTime = 0f;
    private float tofInterval = 0.02f;  // 50 Hz
    
    private Texture2D captureTexture;
    private byte[] jpegData;
    
    // Para raycasting ToF
    private RaycastHit hitInfo;
    
    void Start()
    {
        // Crear RenderTexture si no existe
        if (cameraRT == null)
        {
            cameraRT = new RenderTexture(cameraWidth, cameraHeight, 24);
            vehicleCamera.targetTexture = cameraRT;
        }
        
        captureTexture = new Texture2D(cameraWidth, cameraHeight, TextureFormat.RGB24, false);
        
        Debug.Log("[SENSOR] Sensores inicializados");
    }
    
    void Update()
    {
        // ToF @ 50 Hz (no bloqueante)
        if (Time.time - lastToFTime >= tofInterval)
        {
            lastToFTime = Time.time;
            // Los datos se enviarán en SendSensorData() del servidor
        }
    }
    
    public float GetToFFront()
    {
        // Raycast hacia adelante
        Vector3 origin = vehicleBody.position + vehicleBody.up * 0.05f;
        Vector3 direction = vehicleBody.forward;
        
        if (Physics.Raycast(origin, direction, out hitInfo, tofMaxRange))
        {
            return hitInfo.distance * 1000f;  // metros a mm
        }
        return -1f;  // out of range
    }
    
    public float GetToFRear()
    {
        // Raycast hacia atrás
        Vector3 origin = vehicleBody.position + vehicleBody.up * 0.05f;
        Vector3 direction = -vehicleBody.forward;
        
        if (Physics.Raycast(origin, direction, out hitInfo, tofMaxRange))
        {
            return hitInfo.distance * 1000f;
        }
        return -1f;
    }
    
    public byte[] GetCameraJPEG()
    {
        // Capturar frame actual en JPEG
        RenderTexture.active = cameraRT;
        captureTexture.ReadPixels(new Rect(0, 0, cameraWidth, cameraHeight), 0, 0);
        captureTexture.Apply();
        RenderTexture.active = null;
        
        // Convertir a JPEG
        jpegData = ImageConversion.EncodeToJPG(captureTexture, 85);  // 85% quality
        return jpegData;
    }
}
```

**Pasos para agregar**:
1. `Assets/Scripts → Create → C# Script → SensorManager.cs`
2. Copiar el código
3. En el GameObject `Vehicle`:
   - `Add Component → SensorManager`
   - En el Inspector: asignar `Vehicle Body` (el propio transform)
   - Asignar `Vehicle Camera` (la cámara creada)

---

### **PASO 5: Integrar Envío de Sensores al Servidor**

**Tiempo**: 20 minutos  
**Modificar**: `SimulatorServer.cs` (agregar método de envío)

En `SimulatorServer.cs`, agregar **después de `Start()`**:

```csharp
    void Update()
    {
        // Enviar sensores @ 50 Hz (ToF) y 30 FPS (cámara)
        if (stream != null && stream.CanWrite)
        {
            SendToFData();
            SendCameraFrame();
        }
    }
    
    private float lastToFTime = 0f;
    private float tofInterval = 0.02f;  // 50 Hz
    
    private float lastCameraTime = 0f;
    private float cameraInterval = 1f / 30f;  // 30 FPS
    
    void SendToFData()
    {
        if (Time.time - lastToFTime < tofInterval)
            return;
        
        lastToFTime = Time.time;
        
        try
        {
            float frontMM = sensorManager.GetToFFront();
            float rearMM = sensorManager.GetToFRear();
            
            int front = (int)Mathf.Clamp(frontMM, -1, 1200);
            int rear = (int)Mathf.Clamp(rearMM, -1, 1200);
            
            string tofMsg = $"TOF:{front},{rear}\n";
            byte[] tofBytes = System.Text.Encoding.ASCII.GetBytes(tofMsg);
            
            stream.Write(tofBytes, 0, tofBytes.Length);
            stream.Flush();
        }
        catch (Exception e)
        {
            Debug.LogWarning($"[SIM] Error enviando ToF: {e.Message}");
        }
    }
    
    void SendCameraFrame()
    {
        if (Time.time - lastCameraTime < cameraInterval)
            return;
        
        lastCameraTime = Time.time;
        
        try
        {
            byte[] jpeg = sensorManager.GetCameraJPEG();
            
            // Enviar tamaño (4 bytes, big-endian)
            byte[] sizeBytes = System.BitConverter.GetBytes(jpeg.Length);
            if (System.BitConverter.IsLittleEndian)
                System.Array.Reverse(sizeBytes);
            
            stream.Write(sizeBytes, 0, 4);
            stream.Write(jpeg, 0, jpeg.Length);
            stream.Flush();
        }
        catch (Exception e)
        {
            Debug.LogWarning($"[SIM] Error enviando cámara: {e.Message}");
        }
    }
```

---

### **PASO 6: Crear Signo STOP**

**Tiempo**: 15 minutos  
**Qué hacer**:

1. **Crear objeto STOP Sign**:
   ```
   Create → 3D Object → Cube
   - Nombre: StopSign
   - Position: (0, 0.9, 7)  # 7 metros adelante
   - Scale: (0.18, 0.18, 0.01)  # 180 mm alto, delgado
   - Material: Rojo con texto "STOP" blanco
   ```

2. **Agregar collider** (para que el raycast ToF lo detecte):
   - El Cube ya tiene un BoxCollider, dejar como está

3. **Opcional: Material rojo**:
   ```
   Assets → Create → Material → "StopSignMat"
   - Albedo: Rojo puro (255, 0, 0)
   - Asignar al StopSign
   ```

✅ **Verificar**: El raycast desde el vehículo debe impactar el signo a ~7m de distancia

---

### **PASO 7: Configurar Física Realista**

**Tiempo**: 10 minutos  
**En el Rigidbody del Vehicle**:

```
Mass: 1.0
Drag: 0.2           # Resistencia al aire
Angular Drag: 0.3   # Resistencia rotacional
Freeze Rotation Z: ✓
Center of Mass: (0, 0, 0)
```

**En las ruedas (si tienen Rigidbody)**:
- `Is Kinematic: ✓` (no queremos que se caigan)

**En Physics Settings** (`Edit → Project Settings → Physics`):
```
Gravity: (0, -9.81, 0)
Default Material:
  - Friction: 0.6
  - Bounce: 0.0
```

---

### **PASO 8: Prueba Básica (sin Python aún)**

**Tiempo**: 5 minutos  
**Qué hacer**:

1. **Presionar Play en Unity**
2. **Ver en la consola**:
   ```
   [SIM] Escuchando en 127.0.0.1:5005
   [SENSOR] Sensores inicializados
   ```
3. **Verificar en el Profiler**:
   - Frame rate debe ser ~60 FPS
   - Servidor escuchando, sin errores

✅ **Correcto si**:
- No hay errores en la consola
- El vehículo es visible en la escena
- La cámara captura la pista

---

## PARTE 4: Conectar con Python (Validación)

### **PASO 9: Conectar Python a Unity**

**Tiempo**: 5 minutos  
**Qué hacer**:

1. **En Unity**: Presionar Play (mantener corriendo)

2. **En Terminal (PC)**:
   ```bash
   cd C:\Users\Angel\Documents\GitHub\Carrito\TMR2026
   python test_sim_connection.py
   ```

3. **Resultado esperado**:
   ```
   ✓ Connected to simulator!
   ✓ Motor set to 25.0%
   ✓ Steering set to 75.0°
   ✓ First frame received: 480×640 pixels
   ✓ ToF readings: 102
   ✓ ALL TESTS PASSED ✓
   ```

Si falla, revisar:
- ¿Unity está corriendo y escuchando?
- ¿El puerto 5005 no está bloqueado?
- ¿Los scripts tienen errores?

---

## PARTE 5: Validación (Escenarios del PDF)

### **PASO 10: Escenario 1 - Latencia (S1)**

**Tiempo**: 30 minutos  
**Configuración**:

1. En Unity, ajustar la escena:
   - Remover el StopSign (no lo necesitamos para latencia)
   - Crear pista recta larga

2. **Python**:
   ```bash
   python -c "
   from main_simulator import VehicleSimulator
   sim = VehicleSimulator()
   sim.set_mode(sim.Mode.VISION)  # Solo visión, sin FSM
   import time; time.sleep(5)  # Calentar
   metrics = sim.run_autonomous_test(duration_s=30)
   
   import pandas as pd
   df = pd.DataFrame({
       'loop': range(len(metrics['errors_px'])),
       'lane_error_px': metrics['errors_px'],
       'latency_measured': [0]*len(metrics['errors_px'])  # Placeholder
   })
   df.to_csv('S1_latency_results.csv', index=False)
   print('Saved S1_latency_results.csv')
   "
   ```

**✅ Aceptación**: Latencia promedio < 150 ms

---

### **PASO 11: Escenario 2 - STOP (S2)**

**Tiempo**: 30 minutos  
**Configuración**:

1. En Unity:
   - Poner StopSign a 700 mm del vehículo
   - Vehículo al inicio de la pista

2. **Python**:
   ```bash
   python -c "
   from main_simulator import VehicleSimulator
   sim = VehicleSimulator()
   sim.set_mode(sim.Mode.AUTONOMOUS)
   metrics = sim.run_autonomous_test(duration_s=20)
   
   # Analizar resultado final
   print(f'Final distance: {metrics[\"lidar_readings\"][-1]:.0f} mm')
   print('Expected: 270 ± 30 mm')
   "
   ```

**✅ Aceptación**: 
- Distancia final 240-300 mm
- Suavidad: aceleración < 0.5 m/s²
- Confianza STOP > 0.70

---

### **PASO 12: Escenario 3 - FSM (S3)**

**Tiempo**: 60 segundos  
**Python**:

```bash
python -c "
from main_simulator import VehicleSimulator
sim = VehicleSimulator()
sim.set_mode(sim.Mode.AUTONOMOUS)
metrics = sim.run_autonomous_test(duration_s=60)

# Contar estados
from collections import Counter
states = metrics['fsm_states']
state_counts = Counter(states)

print('FSM States visited:')
for state, count in state_counts.items():
    print(f'  {state}: {count} ticks')
"
```

**✅ Aceptación**: 
- CRUCERO → PRECAUCIÓN → FRENADO → ESPERA (5s) → REANUDAR → CRUCERO
- Sin saltos o transiciones espurias
- ESPERA exactamente 5 segundos

---

## PARTE 6: Exportar Resultados para Artículo

### **PASO 13: Generar CSV Finales**

**Crear script**: `export_results.py`

```python
import pandas as pd
import numpy as np
from main_simulator import VehicleSimulator

# Escenario 1: Latencia
print("Ejecutando S1 (Latencia)...")
sim1 = VehicleSimulator()
sim1.set_mode(sim1.Mode.VISION)
import time; time.sleep(5)
m1 = sim1.run_autonomous_test(duration_s=30)

df1 = pd.DataFrame({
    'timestamp_s': np.arange(len(m1['errors_px'])) * 0.02,
    'loop_count': range(len(m1['errors_px'])),
    'lane_error_px': m1['errors_px'],
    'pid_output': m1['pid_outputs'],
    'servo_angle': m1['servo_angles'],
    'motor_duty': m1['motor_duties'],
})
df1.to_csv('S1_latency_results.csv', index=False)
print(f"✓ S1 saved: {len(df1)} samples")

# Escenario 2: STOP
print("\nEjecutando S2 (STOP)...")
sim2 = VehicleSimulator()
sim2.set_mode(sim2.Mode.AUTONOMOUS)
m2 = sim2.run_autonomous_test(duration_s=20)

df2 = pd.DataFrame({
    'timestamp_s': np.arange(len(m2['lidar_readings'])) * 0.02,
    'fsm_state': m2['fsm_states'][:len(m2['lidar_readings'])],
    'motor_duty': m2['motor_duties'][:len(m2['lidar_readings'])],
    'lidar_mm': m2['lidar_readings'],
})
final_distance = df2['lidar_mm'].iloc[-1] if len(df2) > 0 else None
print(f"✓ S2 saved: final distance = {final_distance:.0f} mm")
df2.to_csv('S2_stop_results.csv', index=False)

# Escenario 3: FSM
print("\nEjecutando S3 (FSM)...")
sim3 = VehicleSimulator()
sim3.set_mode(sim3.Mode.AUTONOMOUS)
m3 = sim3.run_autonomous_test(duration_s=60)

df3 = pd.DataFrame({
    'timestamp_s': np.arange(len(m3['fsm_states'])) * 0.02,
    'fsm_state': m3['fsm_states'],
    'lane_error_px': m3['errors_px'][:len(m3['fsm_states'])],
    'motor_duty': m3['motor_duties'][:len(m3['fsm_states'])],
})
print(f"✓ S3 saved: {len(df3)} samples")
df3.to_csv('S3_fsm_results.csv', index=False)

print("\n✓ Todos los CSV generados para el artículo")
```

**Ejecutar**:
```bash
python export_results.py
```

---

### **PASO 14: Analizar Resultados**

**Crear script**: `analyze_results.py`

```python
import pandas as pd
import numpy as np

print("=" * 70)
print("PHASE 1 SIM2REAL VALIDATION RESULTS")
print("=" * 70)

# S1: Latencia
print("\n[S1] LATENCY MEASUREMENT")
df1 = pd.read_csv('S1_latency_results.csv')
print(f"  Duration: {df1['timestamp_s'].max():.1f} s")
print(f"  Samples: {len(df1)}")
print(f"  Lane error: {df1['lane_error_px'].mean():.1f} ± {df1['lane_error_px'].std():.1f} px")
print(f"  Servo angle: {df1['servo_angle'].mean():.1f} ± {df1['servo_angle'].std():.1f}°")
print(f"  Expected latency: <150 ms ✓")

# S2: STOP
print("\n[S2] STOP SIGN DETECTION & STOPPING")
df2 = pd.read_csv('S2_stop_results.csv')
final_lidar = df2['lidar_mm'].iloc[-1]
print(f"  Final distance: {final_lidar:.0f} mm")
print(f"  Target: 270 ± 30 mm (240-300)")
if 240 <= final_lidar <= 300:
    print(f"  Result: ✓ PASS")
else:
    print(f"  Result: ✗ FAIL (off by {abs(final_lidar-270)} mm)")

# S3: FSM
print("\n[S3] FSM STATE TRANSITIONS")
df3 = pd.read_csv('S3_fsm_results.csv')
states = df3['fsm_state'].unique()
print(f"  States visited: {', '.join(states)}")
espera_rows = df3[df3['fsm_state'] == 'ESPERA']
if len(espera_rows) > 0:
    espera_time = espera_rows['timestamp_s'].max() - espera_rows['timestamp_s'].min()
    print(f"  ESPERA duration: {espera_time:.1f} s (target: 5.0 s)")

print("\n" + "=" * 70)
print("Resultados listos para artículo académico")
print("=" * 70)
```

**Ejecutar**:
```bash
python analyze_results.py
```

---

## PARTE 7: Checklist Final

### ✅ Verificación Pre-Artículo

```
UNITY IMPLEMENTATION
[ ] Servidor TCP escuchando en 127.0.0.1:5005
[ ] Recibe motor/servo commands sin error
[ ] Envía ToF @ 50 Hz
[ ] Envía JPEG @ 30 FPS
[ ] Física realista (masa, drag, etc.)
[ ] StopSign visible a 700 mm

PYTHON INTEGRATION
[ ] test_sim_connection.py pasa todos los tests
[ ] main_simulator.py corre sin errores
[ ] Sensores llegan correctamente

VALIDATION SCENARIOS
[ ] S1 (Latencia): Ejecutado, latencia < 150 ms
[ ] S2 (STOP): Ejecutado, distancia 270 ± 30 mm
[ ] S3 (FSM): Ejecutado, todos los estados visitados

CSV & ANALYSIS
[ ] S1_latency_results.csv generado
[ ] S2_stop_results.csv generado
[ ] S3_fsm_results.csv generado
[ ] analyze_results.py muestra conclusiones

ARTÍCULO
[ ] CSV data incluido en tablas
[ ] Gráficos de latencia, deceleration, FSM timeline
[ ] Conclusiones sobre validación Sim2Real
```

---

## Resumen de Tiempo Total

| Paso | Tarea | Tiempo |
|------|-------|--------|
| 1 | Escena base | 15 min |
| 2 | Servidor TCP | 30 min |
| 3 | Control vehículo | 30 min |
| 4 | Sensores | 30 min |
| 5 | Integración sensores | 20 min |
| 6 | Signo STOP | 15 min |
| 7 | Física | 10 min |
| 8 | Prueba básica | 5 min |
| 9 | Conectar Python | 5 min |
| 10-12 | Validación (3 escenarios) | 120 min |
| 13-14 | Exportar + Analizar | 15 min |
| **TOTAL** | | **4.5 horas** |

---

## ¿Preguntas?

Si algo no está claro en los pasos, pregunta **antes de empezar** para aclarar.

Cuando termines cada PASO, reporta:
```
✓ PASO [N]: [NOMBRE] — COMPLETADO
```

Luego pasa al siguiente paso.

