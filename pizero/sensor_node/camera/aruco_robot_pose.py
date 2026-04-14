#!/usr/bin/env python3
"""
aruco_robot_pose.py — ArUco detection and robot-frame pose estimation for Pi Zero.

Reads camera intrinsics, camera mount extrinsics, and ArUco config from JSON files,
grabs frames with Picamera2, detects ArUco markers, computes each marker's 2D pose
(x_m, y_m, yaw_rad) in the robot frame, and transmits pose messages over UART to
the Pololu 3pi+ 2040 robot controller.

Frame conventions
─────────────────
  Camera frame (OpenCV/ArUco output):
    x_c = right in image, y_c = down in image, z_c = forward (out of lens)

  Robot frame (odom.py / CHANGES.md convention):
    x_r = forward, y_r = left, z_r = up

  Global/map frame:
    Same convention as robot frame at home, with x forward and y left at start.

Transform: camera frame → robot frame
  p_robot = R_rc @ p_camera + t_rc
  where t_rc is the camera position in robot frame (from camera_extrinsics.json).

  R_rc is built from:
    pitch_deg : camera tilt down from horizontal (primary extrinsic)
    yaw_deg   : small trim for camera not facing exactly forward
    roll_deg  : small trim for camera tilt sideways

Yaw of marker in robot frame:
  R_rm = R_rc @ R_cm   (R_cm from cv2.Rodrigues(rvec))
  yaw_r = atan2(R_rm[1, 0], R_rm[0, 0])
  This is the angle of the marker x-axis (ArUco convention) projected onto the
  robot xy plane, measured counter-clockwise from robot forward (+x_r).

Usage
─────
  python3 aruco_robot_pose.py
  python3 aruco_robot_pose.py --config aruco_config.json --intrinsics camera_intrinsics.json
  python3 aruco_robot_pose.py --no-uart   (bench test mode, no UART output)
  python3 aruco_robot_pose.py --verbose   (extra debug output)

UART message format (see POSE_PROTOCOL.md):
  One line per detection:
    POSE,<id>,<timestamp_ms>,<x_m:.4f>,<y_m:.4f>,<yaw_deg:.2f>,<dist_m:.4f>,<qual>\n
"""
import argparse
import json
import math
import signal
import sys
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np


# ─── Signal handling ──────────────────────────────────────────────────────────

RUNNING = True


def stop_handler(signum, frame):
    global RUNNING
    RUNNING = False


signal.signal(signal.SIGINT, stop_handler)
signal.signal(signal.SIGTERM, stop_handler)


# ─── Config loading ───────────────────────────────────────────────────────────

def load_json(path: str) -> dict:
    with open(path, "r") as f:
        return json.load(f)


def build_rotation_rc(pitch_deg: float, yaw_deg: float, roll_deg: float) -> np.ndarray:
    """
    Build rotation matrix R_rc that transforms vectors from camera frame to robot frame.

    Camera frame: x_c right, y_c down, z_c forward (optical axis).
    Robot frame:  x_r forward, y_r left, z_r up.

    pitch_deg: camera pitch down from horizontal (+ve = tilting optical axis toward ground).
    yaw_deg:   small yaw trim (+ve = camera rotated to the left from forward).
    roll_deg:  small roll trim (+ve = top of camera tilted to the right).

    Derivation for pitch_r only (zero yaw, zero roll):
      camera x (right)  in robot frame → -y_r = [0, -1, 0]
      camera z (optical) in robot frame → cos(pitch_r)*x_r - sin(pitch_r)*z_r
      camera y (down)   in robot frame → z_c x x_c cross product
                                       → [-sin(pitch_r), 0, -cos(pitch_r)]

    R_rc_pitch = [[0,     -sin(pitch_r),  cos(pitch_r)],
                  [-1,     0,             0            ],
                  [0,     -cos(pitch_r), -sin(pitch_r) ]]
    """
    pitch_r = math.radians(pitch_deg)
    yaw_trim_r = math.radians(yaw_deg)
    roll_r = math.radians(roll_deg)

    # Base: camera pitched pitch_r down, no yaw/roll
    R_base = np.array([
        [0.0,       -math.sin(pitch_r),  math.cos(pitch_r)],
        [-1.0,       0.0,                0.0               ],
        [0.0,       -math.cos(pitch_r), -math.sin(pitch_r) ],
    ], dtype=np.float64)

    # Small yaw trim about robot z_r (up)
    R_yaw = np.array([
        [ math.cos(yaw_trim_r), -math.sin(yaw_trim_r), 0.0],
        [ math.sin(yaw_trim_r),  math.cos(yaw_trim_r), 0.0],
        [ 0.0,                   0.0,                  1.0],
    ], dtype=np.float64)

    # Small roll trim about robot x_r (forward)
    R_roll = np.array([
        [1.0,  0.0,                0.0               ],
        [0.0,  math.cos(roll_r),  -math.sin(roll_r)  ],
        [0.0,  math.sin(roll_r),   math.cos(roll_r)  ],
    ], dtype=np.float64)

    return R_yaw @ R_roll @ R_base


def build_aruco_detector(dictionary_name: str, detector_cfg: Optional[dict] = None):
    """Build an ArUco detector, handling both modern and legacy OpenCV APIs."""
    aruco_dicts = {
        name: getattr(cv2.aruco, name)
        for name in dir(cv2.aruco)
        if name.startswith("DICT_")
    }
    if dictionary_name not in aruco_dicts:
        raise ValueError(f"Unknown ArUco dictionary: {dictionary_name}. "
                         f"Known: {sorted(aruco_dicts.keys())}")

    aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dicts[dictionary_name])

    if hasattr(cv2.aruco, "ArucoDetector"):
        params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, params)
        legacy = False
    else:
        params = (cv2.aruco.DetectorParameters_create()
                  if hasattr(cv2.aruco, "DetectorParameters_create")
                  else cv2.aruco.DetectorParameters())
        detector = None
        legacy = True

    detector_cfg = detector_cfg or {}

    # Numeric detector parameter overrides from config
    numeric_fields = {
        "adaptiveThreshWinSizeMin": int,
        "adaptiveThreshWinSizeMax": int,
        "adaptiveThreshWinSizeStep": int,
        "adaptiveThreshConstant": float,
        "minMarkerPerimeterRate": float,
        "maxMarkerPerimeterRate": float,
        "polygonalApproxAccuracyRate": float,
        "minCornerDistanceRate": float,
        "minDistanceToBorder": int,
        "minOtsuStdDev": float,
        "perspectiveRemoveIgnoredMarginPerCell": float,
        "maxErroneousBitsInBorderRate": float,
        "errorCorrectionRate": float,
    }

    for key, cast in numeric_fields.items():
        if key in detector_cfg and hasattr(params, key):
            try:
                setattr(params, key, cast(detector_cfg[key]))
            except Exception:
                pass

    # Corner refinement mode from config: NONE | SUBPIX | CONTOUR | APRILTAG
    refine = str(detector_cfg.get("cornerRefinementMethod", "SUBPIX")).upper()
    refine_map = {
        "NONE": getattr(cv2.aruco, "CORNER_REFINE_NONE", 0),
        "SUBPIX": getattr(cv2.aruco, "CORNER_REFINE_SUBPIX", 1),
        "CONTOUR": getattr(cv2.aruco, "CORNER_REFINE_CONTOUR", 2),
        "APRILTAG": getattr(cv2.aruco, "CORNER_REFINE_APRILTAG", 3),
    }
    if hasattr(params, "cornerRefinementMethod"):
        params.cornerRefinementMethod = refine_map.get(refine, refine_map["SUBPIX"])

    if hasattr(params, "cornerRefinementWinSize") and "cornerRefinementWinSize" in detector_cfg:
        try:
            params.cornerRefinementWinSize = int(detector_cfg["cornerRefinementWinSize"])
        except Exception:
            pass
    if hasattr(params, "cornerRefinementMaxIterations") and "cornerRefinementMaxIterations" in detector_cfg:
        try:
            params.cornerRefinementMaxIterations = int(detector_cfg["cornerRefinementMaxIterations"])
        except Exception:
            pass
    if hasattr(params, "cornerRefinementMinAccuracy") and "cornerRefinementMinAccuracy" in detector_cfg:
        try:
            params.cornerRefinementMinAccuracy = float(detector_cfg["cornerRefinementMinAccuracy"])
        except Exception:
            pass

    if hasattr(params, "detectInvertedMarker") and "detectInvertedMarker" in detector_cfg:
        params.detectInvertedMarker = bool(detector_cfg["detectInvertedMarker"])

    # Rebuild detector with updated params for modern API
    if not legacy:
        detector = cv2.aruco.ArucoDetector(aruco_dict, params)

    return aruco_dict, detector, params, legacy


def detect_markers(frame_gray: np.ndarray, aruco_dict, detector, params, legacy: bool):
    """Detect ArUco markers, returning corners and ids."""
    if not legacy:
        corners, ids, _ = detector.detectMarkers(frame_gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(
            frame_gray, aruco_dict, parameters=params
        )
    return corners, ids


def marker_robot_pose(tvec: np.ndarray, rvec: np.ndarray,
                      R_rc: np.ndarray, t_rc: np.ndarray):
    """
    Convert a single marker's camera-frame pose (tvec, rvec from ArUco) into
    the robot frame, returning (x_r, y_r, yaw_r_rad).

    tvec: (3,) translation from camera frame origin to marker center, in camera frame, meters.
    rvec: (3,) Rodrigues rotation vector (camera frame to marker frame).
    R_rc: (3,3) rotation matrix, camera frame → robot frame.
    t_rc: (3,) camera position in robot frame, meters.

    Returns: (x_r_m, y_r_m, yaw_r_rad)
      x_r_m, y_r_m: marker position in robot frame (robot forward and left), meters.
      yaw_r_rad: marker heading in robot frame — angle of the ArUco marker x-axis
                 projected onto the robot XY plane, CCW from robot forward (+x_r).

    ArUco marker axes (OpenCV convention):
      x-axis: points RIGHT along the printed face of the marker (parallel to the
              bottom edge of the printed square; lies IN the marker plane, not normal).
      y-axis: points UP along the printed face (parallel to the side edge; in the plane).
      z-axis: points OUT of the marker face toward the camera (NORMAL to marker plane).

    For a marker lying flat on the floor (face up toward camera):
      - marker x and y axes lie in the ground plane (same plane as robot XY)
      - marker z-axis points up
      - yaw_r_rad is the angle of the marker x-axis in the ground plane, which
        directly gives the orientation of the printed marker (and thus the target).
    """
    p_c = tvec.flatten()

    # Marker position in robot frame
    p_r = R_rc @ p_c + t_rc

    x_r = float(p_r[0])
    y_r = float(p_r[1])

    # OpenCV estimatePoseSingleMarkers gives rvec such that:
    #   p_camera = R @ p_marker + tvec
    # so cv2.Rodrigues(rvec) gives a matrix R whose COLUMNS are the marker frame
    # axes expressed in camera coordinates.  R_rm = R_rc @ R has columns that are
    # the marker frame axes in robot coordinates.
    R_marker_axes_in_cam, _ = cv2.Rodrigues(rvec)

    # Columns of R_rm are marker x, y, z axes expressed in robot frame.
    R_rm = R_rc @ R_marker_axes_in_cam

    # Yaw = direction of marker x-axis projected into robot XY plane, CCW from robot +x.
    # R_rm[:, 0] is the marker x-axis in robot frame (parallel to the marker face,
    # along the bottom edge of the printed square).
    yaw_r = math.atan2(float(R_rm[1, 0]), float(R_rm[0, 0]))

    # Camera-frame distance (z_c ≈ distance for forward-pointing camera)
    dist_c = float(np.linalg.norm(p_c))

    return x_r, y_r, yaw_r, dist_c


# ─── UART ────────────────────────────────────────────────────────────────────

def open_uart(device: str, baudrate: int):
    """Open UART serial port. Returns serial.Serial or None on failure."""
    try:
        import serial
        port = serial.Serial(
            device,
            baudrate,
            timeout=0.0,
            write_timeout=0.0,
            inter_byte_timeout=0.0,
            rtscts=False,
            dsrdtr=False,
            xonxoff=False,
        )
        return port
    except ImportError:
        print("pyserial not installed — UART disabled. Run: pip3 install pyserial", file=sys.stderr)
        return None
    except Exception as e:
        print(f"UART open failed ({device}): {e}", file=sys.stderr)
        return None


def send_pose(uart, marker_id: int, timestamp_ms: int,
              x_m: float, y_m: float, yaw_deg: float,
              dist_m: float, qual: int) -> None:
    """Send a POSE message over UART (see POSE_PROTOCOL.md)."""
    msg = (f"POSE,{marker_id},{timestamp_ms},"
           f"{x_m:.4f},{y_m:.4f},{yaw_deg:.2f},{dist_m:.4f},{qual}\n")
    uart.write(msg.encode("ascii"))


# ─── Main ─────────────────────────────────────────────────────────────────────

def main() -> int:
    parser = argparse.ArgumentParser(description="ArUco robot-frame pose estimator for Pi Zero")
    parser.add_argument("--config", default="aruco_config.json",
                        help="ArUco config JSON (default: aruco_config.json)")
    parser.add_argument("--intrinsics", default="camera_intrinsics.json",
                        help="Camera intrinsics JSON (default: camera_intrinsics.json)")
    parser.add_argument("--extrinsics", default="camera_extrinsics.json",
                        help="Camera extrinsics JSON (default: camera_extrinsics.json)")
    parser.add_argument("--no-uart", action="store_true",
                        help="Disable UART output (bench test / laptop mode)")
    parser.add_argument("--save-annotated", default="",
                        help="If set, save annotated JPEG frames to this directory")
    parser.add_argument("--verbose", action="store_true", help="Extra debug output")
    args = parser.parse_args()

    # ── Load configuration ─────────────────────────────────────────────────
    script_dir = Path(__file__).parent

    def resolve(path: str) -> str:
        p = Path(path)
        if not p.is_absolute():
            p = script_dir / p
        return str(p)

    try:
        cfg = load_json(resolve(args.config))
        intr = load_json(resolve(args.intrinsics))
        extr = load_json(resolve(args.extrinsics))
    except FileNotFoundError as e:
        print(f"Config file not found: {e}", file=sys.stderr)
        return 1

    # ── Camera intrinsics ──────────────────────────────────────────────────
    camera_matrix = np.array(intr["camera_matrix"], dtype=np.float64)
    dist_coeffs = np.array(intr["dist_coeffs"], dtype=np.float64).flatten()
    img_w = int(intr["image_size"][0])
    img_h = int(intr["image_size"][1])

    if args.verbose:
        print(f"Intrinsics: fx={camera_matrix[0,0]:.1f} fy={camera_matrix[1,1]:.1f} "
              f"cx={camera_matrix[0,2]:.1f} cy={camera_matrix[1,2]:.1f}")
        print(f"Dist: {dist_coeffs.tolist()}")

    # ── Camera extrinsics (camera pose in robot frame) ─────────────────────
    # t_rc: position of camera origin in robot frame
    t_rc = np.array([
        extr["camera_x_r_m"],    # forward
        extr["camera_y_r_m"],    # left
        extr["camera_z_r_m"],    # up
    ], dtype=np.float64)

    R_rc = build_rotation_rc(
        pitch_deg=float(extr["pitch_deg"]),
        yaw_deg=float(extr.get("yaw_deg", 0.0)),
        roll_deg=float(extr.get("roll_deg", 0.0)),
    )

    if args.verbose:
        print(f"Extrinsics: t_rc={t_rc.tolist()}")
        print(f"R_rc:\n{R_rc}")

    # ── ArUco setup ────────────────────────────────────────────────────────
    dictionary_name = str(cfg["dictionary"])
    marker_length_m = float(cfg["marker_length_m"])
    valid_ids_list = list(cfg.get("valid_ids", []))
    valid_ids = set(int(i) for i in valid_ids_list) if valid_ids_list else None
    selection_policy = str(cfg.get("selection_policy", "nearest"))
    fps_target = float(cfg.get("fps_target", 5))
    frame_interval_s = 1.0 / max(1.0, fps_target)

    cam_w = int(cfg.get("camera_width", img_w))
    cam_h = int(cfg.get("camera_height", img_h))
    warmup_s = float(cfg.get("warmup_s", 2.0))

    detector_cfg = dict(cfg.get("detector_params", {}))
    aruco_dict, detector, params, legacy = build_aruco_detector(dictionary_name, detector_cfg)

    print(f"ArUco: {dictionary_name}, marker={marker_length_m*1000:.0f}mm, "
          f"valid_ids={valid_ids or 'all'}, policy={selection_policy}")

    # ── Picamera2 ──────────────────────────────────────────────────────────
    try:
        from picamera2 import Picamera2
    except ImportError:
        print("Picamera2 not installed. Run: sudo apt install -y python3-picamera2",
              file=sys.stderr)
        return 2

    picam2 = Picamera2()
    config_cam = picam2.create_still_configuration(main={"size": (cam_w, cam_h)})
    picam2.configure(config_cam)

    print(f"Opening camera {cam_w}x{cam_h} ...")
    picam2.start()
    time.sleep(warmup_s)
    print("Camera ready.")

    # ── UART ───────────────────────────────────────────────────────────────
    uart = None
    if not args.no_uart:
        uart_dev = str(cfg.get("uart_device", "/dev/serial0"))
        uart_baud = int(cfg.get("uart_baudrate", 115200))
        uart = open_uart(uart_dev, uart_baud)
        if uart:
            print(f"UART: {uart_dev} @ {uart_baud}")
        else:
            print("UART unavailable — continuing without serial output.")

    # ── Annotated output ──────────────────────────────────────────────────
    ann_dir: Optional[Path] = None
    if args.save_annotated:
        ann_dir = Path(args.save_annotated)
        ann_dir.mkdir(parents=True, exist_ok=True)

    log_to_file = bool(cfg.get("log_to_file", False))
    log_file = None
    if log_to_file:
        log_path = script_dir / "pose_log.csv"
        log_file = open(log_path, "w")
        log_file.write("timestamp_ms,marker_id,x_m,y_m,yaw_deg,dist_m,qual\n")
        print(f"Logging to {log_path}")

    annotate_display = bool(cfg.get("annotate_display", False))

    # ── Detection loop ─────────────────────────────────────────────────────
    frame_count = 0
    detect_count = 0
    t_start = time.time()
    t_next = t_start

    print("Running (Ctrl+C to stop) ...\n")

    try:
        while RUNNING:
            now = time.time()
            if now < t_next:
                time.sleep(max(0.0, t_next - now))
            t_next = time.time() + frame_interval_s

            # Grab frame as numpy array (BGR)
            try:
                frame = picam2.capture_array()
            except Exception as e:
                print(f"camera capture error: {e}", file=sys.stderr)
                try:
                    picam2.stop()
                except Exception:
                    pass
                time.sleep(0.2)
                try:
                    picam2.start()
                    time.sleep(0.5)
                except Exception as e2:
                    print(f"camera restart failed: {e2}", file=sys.stderr)
                    time.sleep(1.0)
                continue
            timestamp_ms = int((time.time() - t_start) * 1000)
            frame_count += 1

            # Convert to BGR (Picamera2 returns RGB by default in some configs)
            if frame.shape[2] == 4:
                frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
            elif frame.shape[2] == 3:
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            if bool(cfg.get("preprocess_equalize", False)):
                gray = cv2.equalizeHist(gray)
            gray = np.ascontiguousarray(gray, dtype=np.uint8)

            corners, ids = detect_markers(gray, aruco_dict, detector, params, legacy)

            detections = []

            if ids is not None and len(ids) > 0:
                ids_flat = ids.flatten().tolist()

                if annotate_display or ann_dir:
                    cv2.aruco.drawDetectedMarkers(frame, corners, ids)

                # Estimate pose for all markers at once
                try:
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        corners, marker_length_m, camera_matrix, dist_coeffs
                    )
                except Exception as e:
                    if args.verbose:
                        print(f"  pose estimation error: {e}")
                    rvecs, tvecs = None, None

                for idx, marker_id in enumerate(ids_flat):
                    if valid_ids is not None and marker_id not in valid_ids:
                        continue
                    if rvecs is None:
                        continue

                    rvec = rvecs[idx]
                    tvec = tvecs[idx]

                    x_r, y_r, yaw_r, dist_c = marker_robot_pose(
                        tvec, rvec, R_rc, t_rc
                    )
                    yaw_deg = math.degrees(yaw_r)

                    # qual: simple quality based on marker area in pixels
                    pts = corners[idx][0]
                    area = float(cv2.contourArea(pts))
                    qual = min(9, max(0, int(area / 500)))

                    detections.append({
                        "id": marker_id,
                        "x_r": x_r,
                        "y_r": y_r,
                        "yaw_r": yaw_r,
                        "yaw_deg": yaw_deg,
                        "dist_c": dist_c,
                        "qual": qual,
                        "rvec": rvec,
                        "tvec": tvec,
                        "corners_idx": idx,
                    })

                    if annotate_display or ann_dir:
                        cv2.drawFrameAxes(
                            frame, camera_matrix, dist_coeffs,
                            rvec, tvec, marker_length_m * 0.5
                        )
                        label = (f"ID{marker_id} x={x_r:.2f}m y={y_r:.2f}m "
                                 f"yaw={yaw_deg:.0f}deg")
                        cx = int(np.mean(pts[:, 0]))
                        cy = int(np.mean(pts[:, 1])) - 10
                        cv2.putText(frame, label, (cx - 80, cy),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            # ── Selection policy ───────────────────────────────────────────
            selected = []
            if detections:
                if selection_policy == "nearest":
                    best = min(detections, key=lambda d: d["dist_c"])
                    selected = [best]
                elif selection_policy == "lowest_id":
                    best = min(detections, key=lambda d: d["id"])
                    selected = [best]
                else:  # "all"
                    selected = detections

            # ── Transmit and log ───────────────────────────────────────────
            for d in selected:
                detect_count += 1
                if args.verbose or True:
                    print(f"  ID={d['id']:2d} x={d['x_r']:+.3f}m y={d['y_r']:+.3f}m "
                          f"yaw={d['yaw_deg']:+6.1f}deg dist={d['dist_c']:.3f}m "
                          f"qual={d['qual']}")

                if uart is not None:
                    try:
                        send_pose(uart, d["id"], timestamp_ms,
                                  d["x_r"], d["y_r"], d["yaw_deg"],
                                  d["dist_c"], d["qual"])
                    except Exception as e:
                        if args.verbose:
                            print(f"  UART send error: {e}")

            if args.verbose and (ids is None or len(ids) == 0) and (frame_count % 20 == 0):
                blur_metric = float(cv2.Laplacian(gray, cv2.CV_64F).var())
                print(f"  no-tag blur_metric={blur_metric:.1f}")

                if log_file is not None:
                    log_file.write(
                        f"{timestamp_ms},{d['id']},"
                        f"{d['x_r']:.4f},{d['y_r']:.4f},{d['yaw_deg']:.2f},"
                        f"{d['dist_c']:.4f},{d['qual']}\n"
                    )
                    log_file.flush()

            # ── Save annotated frame ───────────────────────────────────────
            if ann_dir is not None:
                ann_path = ann_dir / f"frame_{frame_count:06d}.jpg"
                cv2.imwrite(str(ann_path), frame,
                            [cv2.IMWRITE_JPEG_QUALITY, int(cfg.get("jpeg_quality", 80))])

            # ── Periodic stats ─────────────────────────────────────────────
            elapsed = time.time() - t_start
            if frame_count % 50 == 0:
                fps = frame_count / max(0.001, elapsed)
                print(f"  [{elapsed:.0f}s] frames={frame_count} detections={detect_count} "
                      f"fps={fps:.1f}")

    finally:
        picam2.stop()
        if uart is not None:
            uart.close()
        if log_file is not None:
            log_file.close()

    elapsed = time.time() - t_start
    fps = frame_count / max(0.001, elapsed)
    print(f"\nStopped. {frame_count} frames in {elapsed:.1f}s = {fps:.1f} fps, "
          f"{detect_count} detections.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
