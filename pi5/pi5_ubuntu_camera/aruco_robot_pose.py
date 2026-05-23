#!/usr/bin/env python3
"""
aruco_robot_pose.py — ArUco detection and robot-frame pose estimation for Ubuntu on Pi 5.

This variant is adapted for the working Pi 5 Ubuntu camera path:
  rpicam-vid --codec mjpeg  -> decode JPEG frames in Python/OpenCV.

It keeps the same pose math and UART message format used by sensor_node/camera/aruco_robot_pose.py.

Usage examples:
  python3 aruco_robot_pose.py
  python3 aruco_robot_pose.py --no-uart --verbose
  python3 aruco_robot_pose.py --save-annotated ./ann --save-raw ./raw
"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np


RUNNING = True


def stop_handler(signum, frame):
    del signum, frame
    global RUNNING
    RUNNING = False


signal.signal(signal.SIGINT, stop_handler)
signal.signal(signal.SIGTERM, stop_handler)


def load_json(path: str) -> dict:
    with open(path, "r") as file_handle:
        return json.load(file_handle)


class RPiCamMJPEGStream:
    def __init__(self, width: int, height: int, hz: float, device: str) -> None:
        self.width = width
        self.height = height
        self.hz = hz
        self.device = device
        self.proc: Optional[subprocess.Popen] = None
        self.thread: Optional[threading.Thread] = None
        self.running = False
        self.buffer = bytearray()
        self.latest_frame: Optional[np.ndarray] = None
        self.frame_lock = threading.Lock()
        self.frame_event = threading.Event()

    def _build_env(self) -> dict[str, str]:
        env = os.environ.copy()
        local_lib = "/usr/local/lib/aarch64-linux-gnu"
        env["LD_LIBRARY_PATH"] = f"{local_lib}:{env.get('LD_LIBRARY_PATH', '')}".rstrip(":")
        env.setdefault("LIBCAMERA_IPA_MODULE_PATH", f"{local_lib}/libcamera/ipa")
        return env

    def _build_cmd(self) -> list[str]:
        cmd = [
            "/usr/local/bin/rpicam-vid",
            "--codec",
            "mjpeg",
            "--framerate",
            str(max(1, int(round(self.hz)))),
            "--width",
            str(self.width),
            "--height",
            str(self.height),
            "--nopreview",
            "-t",
            "0",
            "-o",
            "-",
        ]
        if str(self.device).isdigit():
            cmd.extend(["--camera", str(self.device)])
        return cmd

    def start(self) -> None:
        cmd = self._build_cmd()
        print(f"Starting stream: {' '.join(cmd)}")
        try:
            self.proc = subprocess.Popen(
                cmd,
                env=self._build_env(),
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=0,
            )
        except FileNotFoundError as exc:
            raise RuntimeError("/usr/local/bin/rpicam-vid not found") from exc

        time.sleep(0.5)
        if self.proc.poll() is not None:
            stderr = self.proc.stderr.read().decode("utf-8", errors="ignore") if self.proc.stderr else ""
            raise RuntimeError(f"rpicam-vid failed to start (code {self.proc.returncode})\n{stderr}")

        self.running = True
        self.thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.thread.start()

    def _reader_loop(self) -> None:
        assert self.proc is not None and self.proc.stdout is not None
        jpeg_start = b"\xff\xd8"
        jpeg_end = b"\xff\xd9"

        while self.running:
            chunk = self.proc.stdout.read(65536)
            if not chunk:
                self.running = False
                break
            self.buffer.extend(chunk)

            while True:
                start_idx = self.buffer.find(jpeg_start)
                if start_idx == -1:
                    if len(self.buffer) > 1_000_000:
                        self.buffer.clear()
                    break

                end_idx = self.buffer.find(jpeg_end, start_idx)
                if end_idx == -1:
                    if start_idx > 0:
                        del self.buffer[:start_idx]
                    break

                jpeg_data = bytes(self.buffer[start_idx : end_idx + 2])
                del self.buffer[: end_idx + 2]

                frame = cv2.imdecode(np.frombuffer(jpeg_data, np.uint8), cv2.IMREAD_COLOR)
                if frame is None:
                    continue

                with self.frame_lock:
                    self.latest_frame = frame
                self.frame_event.set()

    def get_latest_frame(self, timeout: float = 2.0) -> Optional[np.ndarray]:
        if not self.frame_event.wait(timeout=timeout):
            return None
        with self.frame_lock:
            if self.latest_frame is None:
                return None
            return self.latest_frame.copy()

    def stop(self) -> None:
        self.running = False
        if self.proc is not None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait()
        if self.thread is not None:
            self.thread.join(timeout=1)


def build_rotation_rc(pitch_deg: float, yaw_deg: float, roll_deg: float) -> np.ndarray:
    pitch_r = math.radians(pitch_deg)
    yaw_trim_r = math.radians(yaw_deg)
    roll_r = math.radians(roll_deg)

    R_base = np.array([
        [0.0, -math.sin(pitch_r), math.cos(pitch_r)],
        [-1.0, 0.0, 0.0],
        [0.0, -math.cos(pitch_r), -math.sin(pitch_r)],
    ], dtype=np.float64)

    R_yaw = np.array([
        [math.cos(yaw_trim_r), -math.sin(yaw_trim_r), 0.0],
        [math.sin(yaw_trim_r), math.cos(yaw_trim_r), 0.0],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)

    R_roll = np.array([
        [1.0, 0.0, 0.0],
        [0.0, math.cos(roll_r), -math.sin(roll_r)],
        [0.0, math.sin(roll_r), math.cos(roll_r)],
    ], dtype=np.float64)

    return R_yaw @ R_roll @ R_base


def build_aruco_detector(dictionary_name: str, detector_cfg: Optional[dict] = None):
    aruco_dicts = {
        name: getattr(cv2.aruco, name)
        for name in dir(cv2.aruco)
        if name.startswith("DICT_")
    }
    if dictionary_name not in aruco_dicts:
        raise ValueError(f"Unknown ArUco dictionary: {dictionary_name}. Known: {sorted(aruco_dicts.keys())}")

    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dicts[dictionary_name])
    else:
        aruco_dict = cv2.aruco.Dictionary_get(aruco_dicts[dictionary_name])

    if hasattr(cv2.aruco, "ArucoDetector"):
        params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, params)
        legacy = False
    else:
        params = (
            cv2.aruco.DetectorParameters_create()
            if hasattr(cv2.aruco, "DetectorParameters_create")
            else cv2.aruco.DetectorParameters()
        )
        detector = None
        legacy = True

    detector_cfg = detector_cfg or {}

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

    if not legacy:
        detector = cv2.aruco.ArucoDetector(aruco_dict, params)

    return aruco_dict, detector, params, legacy


def detect_markers(frame_gray: np.ndarray, aruco_dict, detector, params, legacy: bool):
    if not legacy:
        corners, ids, _ = detector.detectMarkers(frame_gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(frame_gray, aruco_dict, parameters=params)
    return corners, ids


def marker_robot_pose(tvec: np.ndarray, rvec: np.ndarray, R_rc: np.ndarray, t_rc: np.ndarray):
    p_c = tvec.flatten()
    p_r = R_rc @ p_c + t_rc

    x_r = float(p_r[0])
    y_r = float(p_r[1])

    R_marker_axes_in_cam, _ = cv2.Rodrigues(rvec)
    R_rm = R_rc @ R_marker_axes_in_cam
    yaw_r = math.atan2(float(R_rm[1, 0]), float(R_rm[0, 0]))
    dist_c = float(np.linalg.norm(p_c))

    return x_r, y_r, yaw_r, dist_c


def wall_tag_robot_pose(tvec: np.ndarray, rvec: np.ndarray, R_rc: np.ndarray, t_rc: np.ndarray):
    p_c = tvec.flatten()
    p_r = R_rc @ p_c + t_rc

    x_r = float(p_r[0])
    y_r = float(p_r[1])

    R_tag_axes_in_cam, _ = cv2.Rodrigues(rvec)
    R_rm = R_rc @ R_tag_axes_in_cam
    yaw_r = math.atan2(float(R_rm[1, 0]), float(R_rm[0, 0]))
    dist_c = float(np.linalg.norm(p_c))

    return x_r, y_r, yaw_r, dist_c


def open_uart(device: str, baudrate: int):
    try:
        import serial

        return serial.Serial(
            device,
            baudrate,
            timeout=0.0,
            write_timeout=0.0,
            inter_byte_timeout=0.0,
            rtscts=False,
            dsrdtr=False,
            xonxoff=False,
        )
    except ImportError:
        print("pyserial not installed — UART disabled. Run: pip3 install pyserial", file=sys.stderr)
        return None
    except Exception as exc:
        print(f"UART open failed ({device}): {exc}", file=sys.stderr)
        return None


def send_pose(uart, marker_id: int, timestamp_ms: int, x_m: float, y_m: float, yaw_deg: float, dist_m: float, qual: int) -> None:
    msg = f"POSE,{marker_id},{timestamp_ms},{x_m:.4f},{y_m:.4f},{yaw_deg:.2f},{dist_m:.4f},{qual}\n"
    uart.write(msg.encode("ascii"))


def main() -> int:
    parser = argparse.ArgumentParser(description="ArUco robot-frame pose estimator for Ubuntu Pi 5")
    parser.add_argument("--config", default="./aruco_config.json", help="ArUco config JSON")
    parser.add_argument("--intrinsics", default="./camera_intrinsics.json", help="Camera intrinsics JSON")
    parser.add_argument("--extrinsics", default="./camera_extrinsics.json", help="Camera extrinsics JSON")
    parser.add_argument("--device", default="0", help="Camera index for rpicam-vid (default: 0)")
    parser.add_argument("--no-uart", action="store_true", help="Disable UART output")
    parser.add_argument("--save-annotated", default="", help="If set, save annotated JPEG frames to this directory")
    parser.add_argument("--save-raw", default="", help="If set, save raw JPEG frames to this directory")
    parser.add_argument("--save-every", type=int, default=1, help="Save one frame every N frames")
    parser.add_argument("--verbose", action="store_true", help="Extra debug output")
    args = parser.parse_args()

    script_dir = Path(__file__).parent

    def resolve(path: str) -> str:
        candidate = Path(path)
        if not candidate.is_absolute():
            candidate = script_dir / candidate
        return str(candidate)

    try:
        cfg = load_json(resolve(args.config))
        intr = load_json(resolve(args.intrinsics))
        extr = load_json(resolve(args.extrinsics))
    except FileNotFoundError as exc:
        print(f"Config file not found: {exc}", file=sys.stderr)
        return 1

    camera_matrix = np.array(intr["camera_matrix"], dtype=np.float64)
    dist_coeffs = np.array(intr["dist_coeffs"], dtype=np.float64).flatten()
    img_w = int(intr["image_size"][0])
    img_h = int(intr["image_size"][1])

    t_rc = np.array([
        extr["camera_x_r_m"],
        extr["camera_y_r_m"],
        extr["camera_z_r_m"],
    ], dtype=np.float64)

    R_rc = build_rotation_rc(
        pitch_deg=float(extr["pitch_deg"]),
        yaw_deg=float(extr.get("yaw_deg", 0.0)),
        roll_deg=float(extr.get("roll_deg", 0.0)),
    )

    dictionary_name = str(cfg["dictionary"])
    marker_length_m = float(cfg["marker_length_m"])
    valid_ids_list = list(cfg.get("valid_ids", []))
    valid_ids = set(int(item) for item in valid_ids_list) if valid_ids_list else None
    selection_policy = str(cfg.get("selection_policy", "nearest"))
    fps_target = float(cfg.get("fps_target", 5))

    cam_w = int(cfg.get("camera_width", img_w))
    cam_h = int(cfg.get("camera_height", img_h))
    warmup_s = float(cfg.get("warmup_s", 2.0))
    jpeg_quality = int(cfg.get("jpeg_quality", 80))

    detector_cfg = dict(cfg.get("detector_params", {}))
    aruco_dict, detector, params, legacy = build_aruco_detector(dictionary_name, detector_cfg)

    wall_tag_ids = set(int(item) for item in cfg.get("wall_tag_ids", []))

    print(
        f"ArUco: {dictionary_name}, marker={marker_length_m*1000:.0f}mm, "
        f"valid_ids={valid_ids or 'all'}, policy={selection_policy}"
    )
    if wall_tag_ids:
        print(f"Vertical wall tag IDs: {sorted(wall_tag_ids)}")

    stream = RPiCamMJPEGStream(width=cam_w, height=cam_h, hz=fps_target, device=args.device)

    try:
        stream.start()
    except RuntimeError as exc:
        print(f"Camera stream failed: {exc}", file=sys.stderr)
        return 2

    print(f"Opening camera {cam_w}x{cam_h} ...")
    time.sleep(max(0.0, warmup_s))
    print("Camera ready.")

    uart = None
    if not args.no_uart:
        uart_dev = str(cfg.get("uart_device", "/dev/serial0"))
        uart_baud = int(cfg.get("uart_baudrate", 115200))
        uart = open_uart(uart_dev, uart_baud)
        if uart:
            print(f"UART: {uart_dev} @ {uart_baud}")
        else:
            print("UART unavailable — continuing without serial output.")

    ann_dir: Optional[Path] = None
    if args.save_annotated:
        ann_dir = Path(args.save_annotated)
        ann_dir.mkdir(parents=True, exist_ok=True)

    raw_dir: Optional[Path] = None
    if args.save_raw:
        raw_dir = Path(args.save_raw)
        raw_dir.mkdir(parents=True, exist_ok=True)

    save_every = max(1, int(args.save_every))

    log_to_file = bool(cfg.get("log_to_file", False))
    log_file = None
    if log_to_file:
        log_path = script_dir / "pose_log.csv"
        log_file = open(log_path, "w")
        log_file.write("timestamp_ms,marker_id,x_m,y_m,yaw_deg,dist_m,qual\n")
        print(f"Logging to {log_path}")

    annotate_display = bool(cfg.get("annotate_display", False))

    frame_count = 0
    detect_count = 0
    t_start = time.time()

    print("Running (Ctrl+C to stop) ...\n")

    try:
        while RUNNING:
            frame = stream.get_latest_frame(timeout=2.0)
            if frame is None:
                if args.verbose:
                    print("  frame timeout")
                continue

            timestamp_ms = int((time.time() - t_start) * 1000)
            frame_count += 1

            frame_raw = frame.copy()
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

                try:
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        corners, marker_length_m, camera_matrix, dist_coeffs
                    )
                except Exception as exc:
                    if args.verbose:
                        print(f"  pose estimation error: {exc}")
                    rvecs, tvecs = None, None

                for idx, marker_id in enumerate(ids_flat):
                    if valid_ids is not None and marker_id not in valid_ids:
                        continue
                    if rvecs is None:
                        continue

                    rvec = rvecs[idx]
                    tvec = tvecs[idx]
                    is_wall_tag = marker_id in wall_tag_ids

                    if is_wall_tag:
                        x_r, y_r, yaw_r, dist_c = wall_tag_robot_pose(tvec, rvec, R_rc, t_rc)
                    else:
                        x_r, y_r, yaw_r, dist_c = marker_robot_pose(tvec, rvec, R_rc, t_rc)

                    yaw_deg = math.degrees(yaw_r)
                    pts = corners[idx][0]
                    area = float(cv2.contourArea(pts))
                    qual = min(9, max(0, int(area / 500)))

                    detections.append(
                        {
                            "id": marker_id,
                            "x_r": x_r,
                            "y_r": y_r,
                            "yaw_deg": yaw_deg,
                            "dist_c": dist_c,
                            "qual": qual,
                            "rvec": rvec,
                            "tvec": tvec,
                            "corners_idx": idx,
                            "is_wall_tag": is_wall_tag,
                        }
                    )

                    if annotate_display or ann_dir:
                        cv2.drawFrameAxes(
                            frame,
                            camera_matrix,
                            dist_coeffs,
                            rvec,
                            tvec,
                            marker_length_m * 0.5,
                        )
                        label = f"ID{marker_id} x={x_r:.2f}m y={y_r:.2f}m yaw={yaw_deg:.0f}deg"
                        cx = int(np.mean(pts[:, 0]))
                        cy = int(np.mean(pts[:, 1])) - 10
                        cv2.putText(frame, label, (cx - 80, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            selected = []
            if detections:
                if selection_policy == "nearest":
                    selected = [min(detections, key=lambda item: item["dist_c"])]
                elif selection_policy == "lowest_id":
                    selected = [min(detections, key=lambda item: item["id"])]
                else:
                    selected = detections

            for detection in selected:
                detect_count += 1
                tag_type = "WALL" if detection.get("is_wall_tag", False) else "TAG"
                print(
                    f"  {tag_type} ID={detection['id']:2d} "
                    f"x={detection['x_r']:+.3f}m y={detection['y_r']:+.3f}m "
                    f"yaw={detection['yaw_deg']:+6.1f}deg dist={detection['dist_c']:.3f}m "
                    f"qual={detection['qual']}"
                )

                if uart is not None:
                    try:
                        send_pose(
                            uart,
                            detection["id"],
                            timestamp_ms,
                            detection["x_r"],
                            detection["y_r"],
                            detection["yaw_deg"],
                            detection["dist_c"],
                            detection["qual"],
                        )
                    except Exception as exc:
                        if args.verbose:
                            print(f"  UART send error: {exc}")

                if log_file is not None:
                    log_file.write(
                        f"{timestamp_ms},{detection['id']},{detection['x_r']:.4f},"
                        f"{detection['y_r']:.4f},{detection['yaw_deg']:.2f},"
                        f"{detection['dist_c']:.4f},{detection['qual']}\n"
                    )
                    log_file.flush()

            if ann_dir is not None and (frame_count % save_every == 0):
                ann_path = ann_dir / f"frame_{frame_count:06d}.jpg"
                cv2.imwrite(str(ann_path), frame, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])

            if raw_dir is not None and (frame_count % save_every == 0):
                raw_path = raw_dir / f"raw_{frame_count:06d}.jpg"
                cv2.imwrite(str(raw_path), frame_raw, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])

            elapsed = time.time() - t_start
            if frame_count % 50 == 0:
                fps = frame_count / max(0.001, elapsed)
                print(f"  [{elapsed:.0f}s] frames={frame_count} detections={detect_count} fps={fps:.1f}")

    finally:
        stream.stop()
        if uart is not None:
            uart.close()
        if log_file is not None:
            log_file.close()

    elapsed = time.time() - t_start
    fps = frame_count / max(0.001, elapsed)
    print(f"\nStopped. {frame_count} frames in {elapsed:.1f}s = {fps:.1f} fps, {detect_count} detections.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
