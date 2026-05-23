"""
LD19 LiDAR Test Script — D300 Kit / RPi5
-----------------------------------------
Reads raw serial data from the LD19, parses packets, and prints
a live summary of each completed 360° scan to the terminal.

Requirements:
    pip install pyserial

Run:
    python3 lidar_test.py

If the port is wrong, list available ports with:
    ls /dev/ttyUSB*
"""

import serial
import struct
import time

# ── Configuration ────────────────────────────────────────────────────────────
PORT      = '/dev/ttyUSB0'   # change if needed (try ttyUSB1 etc.)
BAUD      = 230400
# ─────────────────────────────────────────────────────────────────────────────

HEADER           = 0x54
POINTS_PER_PKT   = 12
# 1(hdr)+1(verlen)+2(speed)+2(start_angle)+36(12*3 data)+2(end_angle)+2(ts)+1(crc)
PACKET_LEN       = 47

CRC_TABLE = [
    0x00,0x4d,0x9a,0xd7,0x79,0x34,0xe3,0xae,0xf2,0xbf,0x68,0x25,0x8b,0xc6,0x11,0x5c,
    0xa9,0xe4,0x33,0x7e,0xd0,0x9d,0x4a,0x07,0x5b,0x16,0xc1,0x8c,0x22,0x6f,0xb8,0xf5,
    0x1f,0x52,0x85,0xc8,0x66,0x2b,0xfc,0xb1,0xed,0xa0,0x77,0x3a,0x94,0xd9,0x0e,0x43,
    0xb6,0xfb,0x2c,0x61,0xcf,0x82,0x55,0x18,0x44,0x09,0xde,0x93,0x3d,0x70,0xa7,0xea,
    0x3e,0x73,0xa4,0xe9,0x47,0x0a,0xdd,0x90,0xcc,0x81,0x56,0x1b,0xb5,0xf8,0x2f,0x62,
    0x97,0xda,0x0d,0x40,0xee,0xa3,0x74,0x39,0x65,0x28,0xff,0xb2,0x1c,0x51,0x86,0xcb,
    0x21,0x6c,0xbb,0xf6,0x58,0x15,0xc2,0x8f,0xd3,0x9e,0x49,0x04,0xaa,0xe7,0x30,0x7d,
    0x88,0xc5,0x12,0x5f,0xf1,0xbc,0x6b,0x26,0x7a,0x37,0xe0,0xad,0x03,0x4e,0x99,0xd4,
    0x7c,0x31,0xe6,0xab,0x05,0x48,0x9f,0xd2,0x8e,0xc3,0x14,0x59,0xf7,0xba,0x6d,0x20,
    0xd5,0x98,0x4f,0x02,0xac,0xe1,0x36,0x7b,0x27,0x6a,0xbd,0xf0,0x5e,0x13,0xc4,0x89,
    0x63,0x2e,0xf9,0xb4,0x1a,0x57,0x80,0xcd,0x91,0xdc,0x0b,0x46,0xe8,0xa5,0x72,0x3f,
    0xca,0x87,0x50,0x1d,0xb3,0xfe,0x29,0x64,0x38,0x75,0xa2,0xef,0x41,0x0c,0xdb,0x96,
    0x42,0x0f,0xd8,0x95,0x3b,0x76,0xa1,0xec,0xb0,0xfd,0x2a,0x67,0xc9,0x84,0x53,0x1e,
    0xeb,0xa6,0x71,0x3c,0x92,0xdf,0x08,0x45,0x19,0x54,0x83,0xce,0x60,0x2d,0xfa,0xb7,
    0x5d,0x10,0xc7,0x8a,0x24,0x69,0xbe,0xf3,0xaf,0xe2,0x35,0x78,0xd6,0x9b,0x4c,0x01,
    0xf4,0xb9,0x6e,0x23,0x8d,0xc0,0x17,0x5a,0x06,0x4b,0x9c,0xd1,0x7f,0x32,0xe5,0xa8,
]


def calc_crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = CRC_TABLE[(crc ^ b) & 0xFF]
    return crc


def parse_packet(raw: bytes):
    """
    Parse one 47-byte LD19 packet.
    Returns list of dicts {angle, distance_mm, intensity} or None on error.
    """
    if len(raw) != PACKET_LEN:
        return None
    if raw[0] != HEADER:
        return None
    if calc_crc8(raw[:-1]) != raw[-1]:
        return None

    start_angle = struct.unpack_from('<H', raw, 4)[0] / 100.0   # degrees
    end_angle   = struct.unpack_from('<H', raw, 4 + 2 + POINTS_PER_PKT * 3)[0] / 100.0

    # Handle angle wrap-around (e.g. start=358°, end=2°)
    if end_angle < start_angle:
        end_angle += 360.0

    step = (end_angle - start_angle) / (POINTS_PER_PKT - 1)

    points = []
    for i in range(POINTS_PER_PKT):
        offset    = 6 + i * 3
        dist_mm   = struct.unpack_from('<H', raw, offset)[0]
        intensity = raw[offset + 2]
        angle     = (start_angle + step * i) % 360.0

        if dist_mm > 0:   # 0 = invalid / no return
            points.append({
                'angle':      round(angle, 2),
                'distance':   dist_mm,
                'intensity':  intensity,
            })
    return points


def print_scan_summary(scan: list, scan_count: int, elapsed: float):
    """Print a compact one-line summary plus closest/furthest points."""
    if not scan:
        print(f"Scan #{scan_count}  — no valid points")
        return

    distances = [p['distance'] for p in scan]
    closest  = min(scan, key=lambda p: p['distance'])
    furthest = max(scan, key=lambda p: p['distance'])

    print(
        f"Scan #{scan_count:4d} | "
        f"Points: {len(scan):3d} | "
        f"Min: {closest['distance']:5d} mm @ {closest['angle']:6.1f}° | "
        f"Max: {furthest['distance']:5d} mm @ {furthest['angle']:6.1f}° | "
        f"Rate: {scan_count/elapsed:4.1f} scans/s"
    )


def print_compass(scan: list):
    """
    Print a simple 8-direction distance table every 10 scans
    so you can physically verify the sensor by pointing it at walls.

    Directions use LD19 convention: 0°=forward, clockwise positive.
    """
    directions = {
        'F  (  0°)': (345, 15),
        'FR ( 45°)': (30,  60),
        'R  ( 90°)': (75,  105),
        'BR (135°)': (120, 150),
        'B  (180°)': (165, 195),
        'BL (225°)': (210, 240),
        'L  (270°)': (255, 285),
        'FL (315°)': (300, 330),
    }
    print("\n  ── Compass snapshot ──────────────────────")
    for label, (a_min, a_max) in directions.items():
        sector = [p for p in scan if a_min <= p['angle'] < a_max]
        if sector:
            avg = int(sum(p['distance'] for p in sector) / len(sector))
            bar = '█' * min(20, avg // 100)
            print(f"  {label}: {avg:5d} mm  {bar}")
        else:
            print(f"  {label}:  no return")
    print()


def main():
    print(f"\nLD19 LiDAR Test — opening {PORT} at {BAUD} baud")
    print("Press Ctrl+C to stop.\n")

    try:
        ser = serial.Serial(PORT, BAUD, timeout=1.0)
    except serial.SerialException as e:
        print(f"\nERROR: Could not open {PORT}")
        print(f"  {e}")
        print("\nTry:  ls /dev/ttyUSB*  to find the correct port.")
        print("      sudo chmod 777 /dev/ttyUSB0  if permission denied.\n")
        return

    print(f"Port open. Waiting for data...\n")

    buf        = bytearray()
    scan       = []
    last_angle = None
    scan_count = 0
    crc_errors = 0
    start_time = time.time()

    try:
        while True:
            # Read a chunk; serial timeout means this won't block forever
            chunk = ser.read(128)
            if not chunk:
                print("  (no data received — is the LiDAR spinning?)")
                continue
            buf.extend(chunk)

            # Process every complete packet in the buffer
            while len(buf) >= PACKET_LEN:
                # Find the next header byte
                idx = buf.find(HEADER)
                if idx == -1:
                    buf.clear()
                    break
                if idx > 0:
                    buf = buf[idx:]         # discard garbage before header

                if len(buf) < PACKET_LEN:
                    break                   # wait for more bytes

                pkt    = bytes(buf[:PACKET_LEN])
                buf    = buf[PACKET_LEN:]

                points = parse_packet(pkt)

                if points is None:
                    crc_errors += 1
                    buf = bytearray(pkt[1:]) + buf
                    continue
                if not points:
                    continue




                # Detect start of a new 360° rotation
                current_angle = points[0]['angle']
                if last_angle is not None and current_angle < last_angle:
                    # Full scan complete — report it
                    scan_count += 1
                    elapsed = time.time() - start_time

                    print_scan_summary(scan, scan_count, elapsed)

                    if scan_count % 10 == 0:
                        print_compass(scan)
                        if crc_errors:
                            print(f"  CRC errors so far: {crc_errors}")

                    scan = []   # start fresh

                last_angle = current_angle
                scan.extend(points)

    except KeyboardInterrupt:
        elapsed = time.time() - start_time
        print(f"\n── Stopped ──────────────────────────────────")
        print(f"  Total scans    : {scan_count}")
        print(f"  Runtime        : {elapsed:.1f} s")
        print(f"  Avg scan rate  : {scan_count/elapsed:.1f} Hz" if elapsed > 0 else "")
        print(f"  CRC errors     : {crc_errors}")
        print()
    finally:
        ser.close()


if __name__ == '__main__':
    main()
