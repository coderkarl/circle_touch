"""Shared camera utilities for Ubuntu on Pi 5 camera scripts."""

from __future__ import annotations

import os
import subprocess
import threading
import time
from typing import Optional

import cv2
import numpy as np


def build_rpicam_env() -> dict[str, str]:
    env = os.environ.copy()
    local_lib = "/usr/local/lib/aarch64-linux-gnu"
    env["LD_LIBRARY_PATH"] = f"{local_lib}:{env.get('LD_LIBRARY_PATH', '')}".rstrip(":")
    env.setdefault("LIBCAMERA_IPA_MODULE_PATH", f"{local_lib}/libcamera/ipa")
    return env


class RPiCamMJPEGStream:
    """Read MJPEG frames from rpicam-vid stdout and expose the latest decoded frame."""

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
                env=build_rpicam_env(),
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

    def get_latest_frame(self, timeout: float = 2.0):
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
