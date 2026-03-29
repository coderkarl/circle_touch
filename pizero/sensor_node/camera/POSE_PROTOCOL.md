# UART Pose Protocol: Pi Zero → 3pi+

## Summary

The Pi Zero 2W sends ArUco marker pose estimates to the Pololu 3pi+ 2040 over a UART
serial link at **115200 baud, 8N1, no flow control**.

Connection:
```
Pi Zero 2W            3pi+ 2040
──────────            ─────────
GPIO 14 (TXD)  ───►  UART RX (e.g. GP1)
GPIO 15 (RXD)  ◄───  UART TX (e.g. GP0)
GND            ───── GND
```
Both sides are 3.3 V — no level shifter needed.

---

## Message Types

### POSE — marker pose in robot frame

One line per detection, terminated by `\n` (LF):

```
POSE,<id>,<ts_ms>,<x_m>,<y_m>,<yaw_deg>,<dist_m>,<qual>\n
```

| Field | Type | Units | Description |
|---|---|---|---|
| `POSE` | literal | — | Message type identifier |
| `id` | int | — | ArUco marker ID |
| `ts_ms` | int | ms | Milliseconds since Pi Zero aruco_robot_pose.py started |
| `x_m` | float (4dp) | m | Marker x position in robot frame (forward is positive) |
| `y_m` | float (4dp) | m | Marker y position in robot frame (left is positive) |
| `yaw_deg` | float (2dp) | deg | Marker heading in robot frame, CCW from robot +x, range [-180, 180] |
| `dist_m` | float (4dp) | m | Camera-to-marker distance (camera z-axis, approximate depth) |
| `qual` | int 0-9 | — | Detection quality (9 = large/clear marker; 0 = small/marginal) |

**Example:**
```
POSE,3,4821,0.2847,-0.0123,91.45,0.3012,7
POSE,7,4821,1.1053,0.0512,-0.82,1.1200,4
```

---

### HEARTBEAT — periodic alive signal (optional)

```
HB,<ts_ms>\n
```

Sent every ~5 seconds when no markers are detected. Lets the 3pi+ detect Pi Zero failure.

**Example:**
```
HB,5000
HB,10003
```

---

## Robot Frame Convention

```
        x_r (forward)
          ↑
          │
y_r ◄────┤  (robot center)
  (left)  │
```

- `x_m > 0`: marker is in front of the robot
- `x_m < 0`: marker is behind the robot (unusual)
- `y_m > 0`: marker is to the left of the robot
- `y_m < 0`: marker is to the right of the robot
- `yaw_deg = 0`: marker x-axis points in robot forward direction
- `yaw_deg = 90`: marker x-axis points in robot left direction (rotated 90° CCW)

---

## 3pi+ Receiver

See `camera_pose_serial.py` for the MicroPython UART receiver.

Key behaviors:
- Non-blocking: 3pi+ main loop calls `camera_pose_serial.get_latest()` to retrieve the newest detection.
- Timeout: if no valid POSE message arrives within `stale_ms` milliseconds, `is_fresh()` returns False.
- Multiple IDs: all received IDs are stored; caller can choose by ID or by calling `get_nearest()`.
- Malformed lines are silently discarded.

---

## Timing

| Parameter | Value |
|---|---|
| Baud rate | 115200 |
| Max message length | ~60 chars |
| Transmission time | ~5.2 ms per message |
| Typical Pi Zero output rate | 5–10 Hz |
| Typical 3pi+ loop period | 50 ms |

With a 50 ms loop on the 3pi+ and 5–10 Hz messages, the robot will typically see a fresh
measurement every 1–2 control cycles.

---

## Parsing on 3pi+ (MicroPython)

```python
# Quick inline parse reference:
import camera_pose_serial
cam = camera_pose_serial.CameraPoseSerial(uart_id=0, baudrate=115200, stale_ms=500)

# In main loop (call every iteration):
cam.update()

# Read latest detection for a specific ID:
pose = cam.get_by_id(3)
if pose and cam.is_fresh(pose):
    x, y, yaw_deg = pose["x_m"], pose["y_m"], pose["yaw_deg"]
```
