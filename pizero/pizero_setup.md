# Raspberry Pi Zero 2W Setup (Pi Camera + ArUco Pose + 3pi+ Integration)

## Goal

Set up a **stable, lightweight vision node** on Raspberry Pi Zero 2W that can:

- Capture frames from the Pi Camera (Picamera2)
- Detect ArUco markers and compute 2D pose (x, y, yaw) of each marker in the robot frame
- Stream pose estimates to the Pololu 3pi+ 2040 over UART serial
- Support camera intrinsic and extrinsic calibration workflows

---

## Platform Decision

- **Hardware:** Raspberry Pi Zero 2W (ARMv8 quad-core, 512 MB RAM)
- **Use OS:** Raspberry Pi OS Lite (32-bit, Bookworm) — headless, SSH only
- **Do not run ROS/ROS2 on Pi Zero** as the primary stack
- **Use custom Python on-device** for camera, ArUco detection, and serial output
- **HDMI display is optional** — only needed for live camera preview during extrinsic setup; all calibration solving can be done on a laptop

### Why Raspberry Pi OS Lite (Bookworm, 32-bit)

- Matches the existing Pi Zero scaffold in this repo
- Supports `python3-picamera2` and `python3-opencv` via `apt`
- 32-bit keeps memory footprint lower (512 MB RAM is tight for 64-bit)
- Headless SSH is sufficient for all development and deployment tasks
- ROS 2 on a Zero-class device adds unnecessary overhead for this application

### Why Pi Zero 2W over Pi Zero W v1.1

- Quad-core ARMv8 vs single-core ARMv6 — roughly 4× faster for OpenCV/ArUco
- Still 512 MB RAM; same form factor and Pi Camera connector
- Sufficient for 5–10 fps ArUco detection at 640×480 or 1280×720

---

## System Architecture

```
Pi Zero 2W (Linux Python)                  3pi+ 2040 (MicroPython)
─────────────────────────────              ─────────────────────────
aruco_robot_pose.py                        camera_circle_touch.py
  ├─ Picamera2 → grab frame                  ├─ read UART → latest pose msg
  ├─ cv2.aruco detect markers                ├─ odom.update_odom()
  ├─ estimatePoseSingleMarkers               ├─ waypoint tracking
  ├─ transform to robot frame                └─ apply camera pose correction
  └─ UART TX → JSON line/msg                      at CIRCLE waypoints
         │
	[UART TX]  ──────────────────────────  [UART RX]  /dev/serial0 ↔ 3pi+ GP0/GP1
```

Pi Zero UART: `/dev/serial0` at 115200 baud (GP0=TX, GP1=RX on Pi Zero header).
3pi+ UART: `machine.UART(0, 115200)` on its default pins.

Processes on Pi Zero:
1. `aruco_robot_pose.py` — camera grab, ArUco detection, pose transform, UART TX
2. Optional `camera_cal_data.py` — calibration image capture (run once during setup)
3. `systemd` unit for auto-start and restart on failure

---

## SD Card / Imaging Checklist

1. Flash **Raspberry Pi OS Lite (32-bit)** to SD card.
2. In Raspberry Pi Imager advanced options:
	 - Set hostname (example: `pizero-sensor`)
	 - Enable SSH
	 - Set username/password
	 - Configure Wi-Fi + locale/timezone
3. Boot Pi Zero and SSH in.

---

## Headless Networking Across Multiple Locations (No SSH Required)

If the Pi may boot where your laptop is not already on the same network, pre-provision multiple Wi-Fi options before deployment.

### Why the long `90-NM-<UUID>.yaml` names

- These files are NetworkManager-backed netplan connection profiles.
- UUID is the stable unique ID for each profile; filenames like `wlan0.yaml` are ambiguous when multiple profiles exist.
- For this setup, treat `/etc/netplan/90-NM-*.yaml` as the source of truth for network profiles.

### Option A (recommended): one Wi-Fi profile with multiple SSIDs

Edit the existing Wi-Fi file on the Pi (or by mounting the SD card on another machine):

`/etc/netplan/90-NM-<UUID1>.yaml`

```yaml
network:
	version: 2
	wifis:
		wlan0:
			renderer: NetworkManager
			dhcp4: true
			access-points:
				HOME_SSID:
					auth:
						key-management: "psk"
						password: "HOME_PASSWORD"
				LAB_SSID:
					auth:
						key-management: "psk"
						password: "LAB_PASSWORD"
			networkmanager:
				uuid: "<UUID1>"
				name: "netplan-wlan0"
```

Notes:
- Keep indentation exact (2 spaces per level).
- Quote passwords, especially if they contain special characters.
- You can keep or remove `match: {}`; it is not required.
- On next boot, NetworkManager should connect to whichever SSID is available.

### Option B: separate Wi-Fi profile per SSID

Create another file such as `/etc/netplan/90-NM-<UUID3>.yaml` with the same `wlan0` structure but a different `networkmanager.uuid` and a single `access-points` entry for the second SSID.

This is useful if you want per-network priorities and easier enable/disable behavior.

### Safe workflow when editing offline

1. Power down Pi and remove SD card.
2. Mount Linux root partition on your laptop.
3. Edit `/etc/netplan/90-NM-*.yaml`.
4. Save and unmount cleanly.
5. Boot Pi at the target site.

If the Pi still does not join Wi-Fi, connect HDMI/USB keyboard once and run:

```bash
sudo netplan generate
sudo netplan apply
sudo journalctl -u NetworkManager -b --no-pager | tail -n 200
```

### Optional: boot-time auto-connect script using `nmcli`

Use this only if you specifically want scripted retries. In most cases, Option A is simpler and more robust.

`/usr/local/bin/wifi-fallback.sh`

```bash
#!/usr/bin/env bash
set -euo pipefail

# SSID:PSK pairs (edit)
NETWORKS=(
	"HOME_SSID:HOME_PASSWORD"
	"LAB_SSID:LAB_PASSWORD"
)

nmcli radio wifi on || true
sleep 3

for entry in "${NETWORKS[@]}"; do
	ssid="${entry%%:*}"
	psk="${entry#*:}"
	if nmcli -t -f SSID dev wifi list ifname wlan0 | grep -Fxq "$ssid"; then
		nmcli dev wifi connect "$ssid" password "$psk" ifname wlan0 && exit 0
	fi
done

exit 1
```

`/etc/systemd/system/wifi-fallback.service`

```ini
[Unit]
Description=WiFi fallback connector
After=NetworkManager.service
Wants=NetworkManager.service

[Service]
Type=oneshot
ExecStart=/usr/local/bin/wifi-fallback.sh
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
```

Enable it once:

```bash
sudo chmod 700 /usr/local/bin/wifi-fallback.sh
sudo systemctl daemon-reload
sudo systemctl enable wifi-fallback.service
```

---

## First SSH Session: Base Setup

Run these commands after first login:

```bash
sudo dpkg-reconfigure locales
# Select en_US.UTF-8 UTF-8
# Choose en_US.UTF-8 as default
sudo apt update
sudo apt full-upgrade -y
sudo apt install -y git vim tmux htop i2c-tools v4l-utils usbutils \
    python3-pip python3-venv python3-serial python3-numpy
sudo reboot
```

After reboot, SSH back in.

---

## Camera Setup and Smoke Tests

### 1) Install camera and OpenCV stack

```bash
sudo apt update
sudo apt install -y rpicam-apps python3-picamera2 python3-opencv
```

`python3-opencv` from apt includes pre-built ArUco support on Bookworm.

### 2) Check camera

```bash
# legacy command names on older images: libcamera-hello/libcamera-still
rpicam-hello -t 2000 --nopreview
rpicam-still -o ~/cam_test.jpg --nopreview
```

scp test image to laptop and verify

Expected: nopreview initializes with no camera errors.

Expected: image file exists and has non-zero size.

### 3) Python Picamera2 test

```bash
python3 - <<'PY'
from picamera2 import Picamera2
from time import sleep

picam2 = Picamera2()
picam2.start()
sleep(2)
picam2.capture_file('picam2_test.jpg')
picam2.stop()
print('Saved picam2_test.jpg')
PY

ls -lh ~/picam2_test.jpg
```

Expected: `picam2_test.jpg` created.

### If camera test fails

Run:

```bash
sudo raspi-config
```

- Interface Options -> Camera (enable if shown)
- Reboot and repeat tests

---

## UART Setup (Pi Zero ↔ 3pi+)

The Pi Zero 2W routes its primary UART to GPIO 14/15 (TXD/RXD, header pins 8/10).
Disable the serial console and enable the UART hardware:

```bash
sudo raspi-config
# Interface Options -> Serial Port
#   Would you like a login shell to be accessible over serial? -> No
#   Would you like the serial port hardware to be enabled? -> Yes
sudo reboot
```

Verify UART device exists after reboot:

```bash
ls -l /dev/serial0 /dev/ttyS0
```

Connect Pi Zero GPIO 14 (TX) → 3pi+ UART RX pin and GPIO 15 (RX) → 3pi+ UART TX pin.
Share ground. Use a level shifter if necessary (Pi Zero is 3.3 V; 3pi+ GPIO is also 3.3 V — direct connection is fine).

Baud rate: **115200** on both sides.

---

## Recommended Project Layout (Pi Zero)

```text
~/sensor_node/
	camera/
		aruco_robot_pose.py        <- main runtime: detect, pose, UART TX
		camera_cal_data.py         <- calibration image capture
		camera_calibrate_offline.py <- run on laptop to solve intrinsics
		camera_intrinsics.json     <- saved after calibration
		camera_extrinsics.json     <- camera mount params (edit before use)
		aruco_config.json          <- dictionary, marker size, valid IDs
		checkerboard_a4_9x6_25mm.svg <- print this for calibration
		captures/                  <- still captures (general)
		cal_images/                <- checkerboard images for calibration
	logs/
	services/
		camera.service            <- systemd unit for camera/pose runtime
```

---

## Scaffold Added in This Repo

The following files are available locally in this workspace under `pizero/sensor_node/camera/`:

| File | Purpose |
|---|---|
| `camera_capture.py` | General periodic still capture utility for manual/debug use |
| `camera_cal_data.py` | Capture checkerboard images for calibration |
| `camera_calibrate_offline.py` | Solve camera intrinsics from checkerboard images (run on laptop) |
| `camera_intrinsics.json` | Template; replaced by calibration output |
| `camera_extrinsics.json` | Camera-to-robot mount parameters (edit to match your setup) |
| `aruco_config.json` | ArUco dictionary, marker size, valid IDs |
| `aruco_robot_pose.py` | Main runtime: grab frames, detect ArUco, compute robot-frame pose, UART TX |
| `checkerboard_a4_9x6_25mm.svg` | Printable checkerboard for calibration |
| `CHECKERBOARD_PRINTING.md` | Print instructions |
| `POSE_PROTOCOL.md` | UART message format spec |

Also in the repo root (MicroPython, goes on 3pi+):

| File | Purpose |
|---|---|
| `camera_pose_serial.py` | UART RX helper that parses pose messages from Pi Zero |
| `camera_circle_touch.py` | Updated 3pi+ controller with ArUco pose correction |

---

## Deploy to Pi Zero

From your development machine:

```bash
scp -r pizero/sensor_node pi@pizero-sensor:/home/pi/
# also copy config files you edited:
scp pizero/sensor_node/camera/camera_extrinsics.json pi@pizero-sensor:/home/pi/sensor_node/camera/
scp pizero/sensor_node/camera/aruco_config.json pi@pizero-sensor:/home/pi/sensor_node/camera/
scp pizero/sensor_node/camera/camera_intrinsics.json pi@pizero-sensor:/home/pi/sensor_node/camera/
```

---

## Calibration Workflow (One-Time Setup)

### Step 1 — Print checkerboard

Print `checkerboard_a4_9x6_25mm.svg` at 100% scale (no fit-to-page).
See `CHECKERBOARD_PRINTING.md` for instructions and verification.

### Step 2 — Capture calibration images on Pi Zero

```bash
python3 /home/pi/sensor_node/camera/camera_cal_data.py \
    --output-dir /home/pi/sensor_node/camera/cal_images \
    --width 1280 --height 720
```

Press Enter to capture each frame; aim for 20–30 images at varied angles and distances.
Copy images back to laptop:

```bash
scp -r pi@pizero-sensor:/home/pi/sensor_node/camera/cal_images ./cal_images
```

### Step 3 — Solve calibration on laptop (recommended)

```bash
python3 pizero/sensor_node/camera/camera_calibrate_offline.py \
    --images ./cal_images \
    --rows 6 --cols 9 --square-mm 25.0 \
    --output pizero/sensor_node/camera/camera_intrinsics.json
```

Or run directly on the Pi Zero (slower but works):

```bash
python3 /home/pi/sensor_node/camera/camera_calibrate_offline.py \
    --images /home/pi/sensor_node/camera/cal_images \
    --rows 6 --cols 9 --square-mm 25.0 \
    --output /home/pi/sensor_node/camera/camera_intrinsics.json
```

### Step 4 — Edit camera_extrinsics.json

Measure how the camera is mounted on the robot and fill in:
- `camera_x_r_m`: camera forward offset in robot frame (meters)
- `camera_y_r_m`: camera left offset in robot frame (meters, positive = left)
- `camera_z_r_m`: camera height above ground (meters)
- `pitch_deg`: camera pitch-down from horizontal (degrees, positive = looking down)
- `yaw_deg`: small yaw trim if camera is not exactly forward-facing (degrees)
- `roll_deg`: small roll trim (degrees, normally 0)

### Step 5 — Run ArUco pose node

```bash
python3 /home/pi/sensor_node/camera/aruco_robot_pose.py
```

---

## Next Steps (Implementation Order)

1. Flash Pi Zero 2W with Raspberry Pi OS Lite (Bookworm, 32-bit), enable SSH.
2. Install software stack (`apt` packages above).
3. Configure UART: disable serial console, enable hardware UART.
4. Confirm camera smoke tests pass (`rpicam-hello`, `picam2_test.jpg`).
5. Print and verify checkerboard.
6. Capture calibration images; run calibration solver.
7. Mount camera on robot; measure and fill in `camera_extrinsics.json`.
8. Bench-test `aruco_robot_pose.py` with a known marker at known distance.
9. Wire UART between Pi Zero and 3pi+; verify pose messages arrive on 3pi+.
10. Deploy `camera_circle_touch.py` to 3pi+ and test integrated operation.

---

## Validation Checklist

- Pi Zero 2W boots headless and SSH is stable.
- Camera still capture works from CLI and Python.
- ArUco markers are detected and annotated frames look correct.
- Calibration RMS reprojection error < 1.0 pixel.
- Printed checkerboard squares measure correct physical size.
- Robot-frame x, y, yaw match tape-measure ground truth within ~5%.
- UART pose messages arrive on 3pi+ at expected rate.
- 3pi+ controller converges on tagged targets without black-circle sensing.

---

## Notes

- HDMI display is **not required** for any calibration or runtime step. It is optional for live camera preview or debugging.
- Keep `camera_intrinsics.json` in version control after calibration so it is not lost.
- If markers appear at the edge of frame, increase `--width`/`--height` or adjust camera mount angle.
- See `POSE_PROTOCOL.md` for the UART message format and `camera_pose_serial.py` for the 3pi+ receiver.

## pi zero services
```bash
sudo systemctl daemon-reload
sudo systemctl restart camera.service
sudo systemctl status camera.service
journalctl -u [camera.service](http://_vscodecontentref_/10) -f
```