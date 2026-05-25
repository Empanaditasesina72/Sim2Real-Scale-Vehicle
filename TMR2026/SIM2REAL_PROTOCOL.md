# Sim2Real Socket Protocol Specification (Phase 1)

## Overview

Bidirectional TCP/IP communication between:
- **Client (Python)**: `main_simulator.py` running on PC
- **Server (Unity)**: Physics simulator + virtual sensors on same PC or remote

**Connection**:
- Host: `127.0.0.1` (localhost) or remote IP
- Port: `5005` (configurable in `main_simulator.py`)
- Protocol: TCP/IP, blocking I/O with timeouts

---

## Message Format

### Client → Server (Commands)

Commands are **text-based**, one per `sendall()`:

#### 1. Motor Control
```
MOTOR:{duty_percent}\n
```
- `duty_percent`: float [-100.0, 100.0]
  - Negative = reversa (backwards)
  - Positive = avance (forward)
  - Zero = neutra (coast)
- Example: `MOTOR:-45.50` → 45.5% PWM reversa
- Frequency: ~50 Hz (main loop)
- **Must be instantaneous** (no ramp in protocol; soft-start happens in hardware driver)

#### 2. Steering Control
```
SERVO:{angle_deg}\n
```
- `angle_deg`: float [0.0, 180.0]
  - 90.0 = center (straight)
  - < 90 = left turn
  - > 90 = right turn
- Example: `SERVO:75.35` → 75.35° servo position
- Frequency: ~50 Hz (main loop)
- **No inversion in protocol** — servo receives logical angle (inversion happens in hardware driver on Pi)

#### 3. Healthcheck (optional)
```
PING\n
```
- Server responds: (ignored — just used to verify connection)
- Used by `is_connected()` in SimulatorClient

---

### Server → Client (Telemetry)

Server sends telemetry asynchronously. Messages arrive in a continuous stream on the same socket.

#### 1. Time-of-Flight Sensor Data
```
TOF:{front_mm},{rear_mm}\n
```
- `front_mm`: int, millimeters [0, 1200] or -1 if out of range
- `rear_mm`: int, millimeters [0, 1200] or -1 if out of range
- Example: `TOF:425,189` → front sensor reads 425 mm, rear reads 189 mm
- Frequency: **50 Hz** (must match `TOF_POLL_INTERVAL_S = 0.020` in config.py)
- **CRITICAL**: If one sensor is disabled/unavailable, send `-1` for that value

#### 2. Camera Frames (JPEG)
```
[4-byte size header (big-endian)][JPEG data][4-byte size][JPEG data]...
```
- Binary format (not text)
- Size header: `int.from_bytes(data[:4], 'big')` → frame size in bytes
- JPEG data: Raw JPEG bytes (can be decoded with `cv2.imdecode()`)
- Example binary sequence:
  ```
  00 00 02 A0              # Frame 1 size = 672 bytes
  FF D8 FF E0 ... (672 bytes of JPEG) ... FF D9
  00 00 01 F4              # Frame 2 size = 500 bytes
  FF D8 FF E0 ... (500 bytes of JPEG) ... FF D9
  ```
- Frequency: **30 FPS** (matches `CAMERA_FPS = 30` in config.py)
- Resolution: **640×480 pixels** (matches `CAMERA_W/H`)
- Color space: **BGR** (Python reads RGB from camera, but internally stores BGR for OpenCV)
  - If Unity sends RGB JPEG, Python's `cv2.imdecode()` will interpret as-is (no conversion needed in reception)
  - Vision pipeline assumes BGR, so Unity must ensure frames are compatible

---

## Buffering & Frame Synchronization

### Python Reception

The `MockDistanceSensor._receive_loop()` and `MockCameraStream._receive_loop()` threads handle buffering:

1. **TOF messages**:
   - Searched for by `b"TOF:"` prefix
   - Buffer accumulates data until complete message (ends with `\n` or next `TOF:`)
   - Multiple messages can arrive in one `recv()` call
   - Frame loss is acceptable — latest reading replaces old one

2. **JPEG frames**:
   - Size header is read first (4 bytes, big-endian)
   - Then exactly that many bytes are read for the JPEG
   - Buffering handles partial frames (waits for full frame before decoding)
   - Old frames are overwritten — only latest is kept in `latest_frame`

### Socket Configuration

```python
self.socket.settimeout(1.0)  # 1 second read timeout
```
- Prevents main loop from hanging if server stalls
- `socket.timeout` exception is caught and ignored (allows graceful degradation)

---

## Synchronization & Latency

### Open-Loop vs Closed-Loop

**Current design (Phase 1A)**: Open-loop command-response

- Python: Send motor/servo commands at 50 Hz
- Python: Receive sensor data asynchronously at server's rate
- **No ACK/NAK handshake** — fire-and-forget commands
- Latency = network delay + server processing + Python polling rate

**Example timeline** (best case, all messages on same TCP frame):
```
t=0ms    Python sends MOTOR:25.0
t=0.5ms  Network TX → Server received
t=0.5ms  Server applies physics, reads virtual sensors
t=1ms    Server sends TOF:425,189 + JPEG frame
t=1.5ms Server sends next TOF + JPEG
t=2ms    Python receives TOF (in recv thread) + updates lidar_mm
t=20ms   Python next reads camera frame (30 FPS = 33 ms max latency)
```

**Measured latency** (to validate in Phase 1):
- Perception-to-actuation latency: time from camera frame capture → motor command response
- Expected: 80–120 ms (3–4 frames at 30 FPS)

### Message Ordering

Messages sent from Unity **must maintain order**:
1. Send motor/servo updates (if changed)
2. Send TOF sensor readings
3. Send JPEG camera frame

**Do NOT reorder** or interleave — Python's parser assumes TOF lines stay grouped and JPEG frames arrive intact.

---

## Error Handling

### Recoverable Errors

- **TOF parse failure** (malformed numbers) → silently skip, keep previous value
- **JPEG decode failure** → silently skip, keep previous frame
- **Partial JPEG in buffer** → wait for more data (don't drop frame)
- **Socket timeout (1s)** → continue, assume sensor data not available

### Fatal Errors

- **Connection refused** → `SimulatorClient.__init__()` raises, program exits
- **Unexpected EOF on socket** → threads stop, Python sees stale data (should detect via status checks)

### Graceful Degradation

If **TOF sensor dies**: `front_mm = None` → Python checks `if self.sensor.front_mm is not None` before using

If **Camera dies**: `latest_frame = None` → Python skips vision pipeline until frame arrives

If **Network lag**: Python still executes local FSM/PID at 50 Hz, but with stale sensor data (acceptable for short delays)

---

## Performance Requirements

| Metric | Min | Target | Max |
|--------|-----|--------|-----|
| Main loop rate (Python) | 40 Hz | 50 Hz | 60 Hz |
| TOF sensor rate | 40 Hz | 50 Hz | 60 Hz |
| Camera FPS | 25 | 30 | 35 |
| Command latency (motor/servo) | — | <5 ms | <20 ms |
| TOF→Python latency | — | <10 ms | <30 ms |
| Full frame latency (camera capture → display) | — | <80 ms | <150 ms |
| Buffer memory (max) | — | <50 MB | 100 MB |

---

## Testing Checklist

- [ ] Unity sends MOTOR/SERVO commands reliably
- [ ] Python receives motor commands and passes to wheels/steering
- [ ] Python receives TOF data and updates `distance.front_mm` / `distance.rear_mm`
- [ ] Python receives JPEG frames at 30 FPS, decodes correctly
- [ ] Network latency measured (<150 ms round-trip)
- [ ] Graceful handling when simulator is slow/disconnected
- [ ] Lane detection works on Unity-generated frames (calibrate threshold if needed)
- [ ] STOP sign detection triggers FSM state transitions
- [ ] PID steering correction responds to lane error signal

---

## Connection States

### State Machine (Python)

```
DISCONNECTED ─┬→ CONNECTING → CONNECTED → RUNNING
              └→ [init] tries connect, raises on failure
                   ↓ success
                RUNNING (main loop pumps events)
                   ↓ ctrl-c / signal
                SHUTDOWN (closes socket, stops threads)
```

### Server (Unity) Expectations

- **Always listening** on port 5005
- **Accept** new connections without dropping old ones
- **Send** TOF at 50 Hz (timer-based, independent of client heartbeat)
- **Send** JPEG at 30 FPS
- **React to motor/servo** commands within <5 ms
- **Gracefully handle** client disconnects (stop sending, ready for next client)

---

## Example Python Recv Loop (Pseudo-code)

```python
def _receive_loop(self):
    buffer = b""
    while self._listening:
        data = self.socket.recv(1024)  # May block up to 1s
        buffer += data
        
        # Process TOF messages
        while b"TOF:" in buffer:
            idx = buffer.find(b"TOF:")
            end_idx = buffer.find(b"\n", idx)
            if end_idx == -1:
                break  # Wait for more data
            msg = buffer[idx:end_idx].decode()
            parts = msg.split(":")
            if len(parts) == 2:
                front, rear = map(int, parts[1].split(","))
                with self._lock:
                    self.front_mm = front
                    self.rear_mm = rear
            buffer = buffer[end_idx+1:]
        
        # Process JPEG frames
        while len(buffer) >= 4:
            size = int.from_bytes(buffer[:4], 'big')
            if len(buffer) < 4 + size:
                break  # Wait for full frame
            frame_data = buffer[4:4+size]
            buffer = buffer[4+size:]
            
            frame = cv2.imdecode(np.frombuffer(frame_data, np.uint8), 
                                 cv2.IMREAD_COLOR)
            if frame is not None:
                with self.frame_lock:
                    self.latest_frame = frame
```

---

## Unity Implementation Notes

### C# Socket Server (Pseudo-code)

```csharp
TcpListener listener = new TcpListener(IPAddress.Loopback, 5005);
listener.Start();

while (gameRunning) {
    if (listener.Pending()) {
        client = listener.AcceptTcpClient();
        // Handle client in separate thread
        HandleClient(client);
    }
}

void HandleClient(TcpClient client) {
    var reader = new StreamReader(client.GetStream());
    var writer = new BinaryWriter(client.GetStream());
    
    // Send TOF @ 50 Hz
    tofTimer = new Timer(() => {
        float front = raycast_distance_front();
        int front_mm = (int)(front * 1000) % 1200;
        string msg = $"TOF:{front_mm},{rear_mm}\n";
        writer.Write(msg);
        writer.Flush();
    }, null, 0, 20);  // 20 ms = 50 Hz
    
    // Send JPEG @ 30 FPS
    cameraTimer = new Timer(() => {
        Texture2D frame = Render(simulationCamera);
        byte[] jpeg = frame.EncodeToJPG();
        byte[] size = BitConverter.GetBytes(jpeg.Length);  // big-endian!
        Array.Reverse(size);
        writer.Write(size);
        writer.Write(jpeg);
        writer.Flush();
    }, null, 0, 33);  // 33 ms ≈ 30 FPS
    
    // Handle incoming commands
    while (client.Connected) {
        string line = reader.ReadLine();
        if (line.StartsWith("MOTOR:")) {
            float duty = float.Parse(line.Substring(6));
            ApplyMotorForce(duty / 100.0f);
        }
        else if (line.StartsWith("SERVO:")) {
            float angle = float.Parse(line.Substring(6));
            RotateWheels(angle);
        }
    }
}
```

---

## Validation Metrics (CSV Output)

Each second of operation, log to CSV:

```
timestamp_s,loop_count,frame_count,motor_duty,servo_angle,lidar_front_mm,lane_error_px,pid_output,fsm_state
0.02,1,0,0.0,90.0,-1,-0.5,0.0,CRUCERO
0.04,2,0,0.0,90.0,425,2.3,0.18,CRUCERO
0.06,3,1,10.0,92.5,420,5.1,0.41,CRUCERO
```

This enables:
- Latency analysis (e.g., time between command and effect)
- PID tuning verification
- FSM state distribution
- Vision pipeline performance
- End-to-end validation of control logic

