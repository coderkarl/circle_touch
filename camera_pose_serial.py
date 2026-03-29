# camera_pose_serial.py — MicroPython UART receiver for camera ArUco pose messages.
#
# Deploy this file to the Pololu 3pi+ 2040 alongside camera_circle_touch.py.
#
# Protocol: see pizero/sensor_node/camera/POSE_PROTOCOL.md
# Message format:  POSE,<id>,<ts_ms>,<x_m>,<y_m>,<yaw_deg>,<dist_m>,<qual>\n
#
# Usage example (inside camera_circle_touch.py or main loop):
#
#   import camera_pose_serial
#   cam = camera_pose_serial.CameraPoseSerial(uart_id=0, baudrate=115200, stale_ms=500)
#
#   # In the main 50ms loop — call every iteration:
#   cam.update()
#
#   # Check if any fresh detection is available:
#   if cam.has_fresh():
#       nearest = cam.get_nearest()   # closest marker by dist_m
#       if nearest:
#           x, y, yaw_deg = nearest["x_m"], nearest["y_m"], nearest["yaw_deg"]
#
#   # Or look up by marker ID:
#   pose = cam.get_by_id(3)
#   if pose and cam.is_pose_fresh(pose):
#       ...

import machine
import time


class CameraPoseSerial:
    """
    Non-blocking UART receiver for Pi Zero ArUco pose messages.

    Call update() every control cycle.  The internal line buffer accumulates
    UART bytes; complete lines are parsed and stored keyed by marker ID.

    Attributes:
        uart          : machine.UART instance
        stale_ms      : age in ms after which a detection is considered stale
        _buf          : byte accumulation buffer (bytearray)
        _detections   : dict mapping marker_id (int) → detection dict
    """

    # Detection keys: id, ts_local_ms, ts_cam_ms, x_m, y_m, yaw_deg, dist_m, qual
    MAX_BUF = 256  # guard against runaway buffer

    def __init__(self, uart_id=0, baudrate=115200, stale_ms=500,
                 tx_pin=None, rx_pin=None):
        """
        uart_id    : UART peripheral index (0 or 1, default 0 = GP0/GP1 on 3pi+)
        baudrate   : must match Pi Zero aruco_robot_pose.py / aruco_config.json
        stale_ms   : detection age threshold in milliseconds
        tx_pin     : override TX pin (machine.Pin), or None for default
        rx_pin     : override RX pin (machine.Pin), or None for default
        """
        self.stale_ms = stale_ms
        self._buf = bytearray()
        self._detections = {}   # {int(marker_id): detection_dict}
        self._last_hb_ms = 0    # timestamp of last HB message
        self._parse_errors = 0

        if tx_pin is not None and rx_pin is not None:
            self.uart = machine.UART(uart_id, baudrate=baudrate,
                                     tx=tx_pin, rx=rx_pin)
        else:
            self.uart = machine.UART(uart_id, baudrate=baudrate)

    # ── Public poll method ────────────────────────────────────────────────

    def update(self):
        """
        Read available UART bytes and parse any complete lines.
        Call this every control loop iteration (e.g. every 50ms).
        """
        n = self.uart.any()
        if n <= 0:
            return

        chunk = self.uart.read(n)
        if chunk is None:
            return

        self._buf.extend(chunk)

        # Guard against runaway buffer (malformed input, no newlines)
        if len(self._buf) > self.MAX_BUF:
            # Discard everything up to the last newline, or all if none found
            idx = self._buf.rfind(b'\n')
            if idx >= 0:
                self._buf = self._buf[idx + 1:]
            else:
                self._buf = bytearray()
            self._parse_errors += 1
            return

        # Process all complete lines in buffer
        while True:
            idx = self._buf.find(b'\n')
            if idx < 0:
                break
            line_bytes = self._buf[:idx]
            self._buf = self._buf[idx + 1:]
            line = line_bytes.decode("ascii", "ignore").strip()
            if line:
                self._parse_line(line)

    # ── Query methods ─────────────────────────────────────────────────────

    def has_fresh(self):
        """Return True if at least one fresh (non-stale) detection exists."""
        now = time.ticks_ms()
        for d in self._detections.values():
            if time.ticks_diff(now, d["ts_local_ms"]) < self.stale_ms:
                return True
        return False

    def get_by_id(self, marker_id):
        """
        Return the latest detection dict for the given marker ID, or None.
        Does not check freshness — use is_pose_fresh() separately.
        """
        return self._detections.get(int(marker_id), None)

    def get_nearest(self, fresh_only=True):
        """
        Return the detection with smallest dist_m.
        If fresh_only (default True), only consider non-stale detections.
        Returns None if no suitable detection exists.
        """
        now = time.ticks_ms()
        best = None
        best_dist = 1e9
        for d in self._detections.values():
            if fresh_only:
                age = time.ticks_diff(now, d["ts_local_ms"])
                if age >= self.stale_ms:
                    continue
            if d["dist_m"] < best_dist:
                best_dist = d["dist_m"]
                best = d
        return best

    def get_all_fresh(self):
        """Return list of all non-stale detections (may be empty list)."""
        now = time.ticks_ms()
        result = []
        for d in self._detections.values():
            if time.ticks_diff(now, d["ts_local_ms"]) < self.stale_ms:
                result.append(d)
        return result

    def is_pose_fresh(self, detection):
        """Return True if a detection dict (returned by get_by_id etc.) is non-stale."""
        if detection is None:
            return False
        age = time.ticks_diff(time.ticks_ms(), detection["ts_local_ms"])
        return age < self.stale_ms

    def pi_alive(self, heartbeat_timeout_ms=10000):
        """
        Return True if a HB or POSE message was received within heartbeat_timeout_ms.
        Useful for detecting Pi Zero failure.
        """
        if self._last_hb_ms == 0:
            return False
        age = time.ticks_diff(time.ticks_ms(), self._last_hb_ms)
        return age < heartbeat_timeout_ms

    def parse_errors(self):
        """Return cumulative parse/overflow error count."""
        return self._parse_errors

    def clear(self):
        """Clear all stored detections."""
        self._detections.clear()

    # ── Internal ─────────────────────────────────────────────────────────

    def _parse_line(self, line):
        """Parse a single ASCII line and update internal state."""
        if line.startswith("POSE,"):
            self._parse_pose(line)
        elif line.startswith("HB,"):
            self._last_hb_ms = time.ticks_ms()

    def _parse_pose(self, line):
        """
        Parse:  POSE,<id>,<ts_ms>,<x_m>,<y_m>,<yaw_deg>,<dist_m>,<qual>
        On any parse failure, increment error count and return silently.
        """
        parts = line.split(",")
        if len(parts) != 8:
            self._parse_errors += 1
            return
        try:
            marker_id  = int(parts[1])
            ts_cam_ms  = int(parts[2])
            x_m        = float(parts[3])
            y_m        = float(parts[4])
            yaw_deg    = float(parts[5])
            dist_m     = float(parts[6])
            qual       = int(parts[7])
        except (ValueError, IndexError):
            self._parse_errors += 1
            return

        self._detections[marker_id] = {
            "id":          marker_id,
            "ts_local_ms": time.ticks_ms(),   # 3pi+ local time of receipt
            "ts_cam_ms":   ts_cam_ms,          # Pi Zero timestamp
            "x_m":         x_m,
            "y_m":         y_m,
            "yaw_deg":     yaw_deg,
            "dist_m":      dist_m,
            "qual":        qual,
        }
        # Update heartbeat tracker on any successful message
        self._last_hb_ms = time.ticks_ms()
