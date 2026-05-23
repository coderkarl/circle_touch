#!/usr/bin/env python3
"""
camera_cal_data.py — Checkerboard calibration image capture for Ubuntu on Pi 5.

This version uses the working `rpicam-vid --codec mjpeg` capture path instead of
Picamera2/OpenCV V4L2 direct capture.

Usage:
  Interactive (press Enter to capture, 'q' to quit):
    python3 camera_cal_data.py --output-dir ./cal_images --width 1296 --height 972

  Timed (capture every N seconds for M images):
    python3 camera_cal_data.py --output-dir ./cal_images --interval 3.0 --count 25

IMPORTANT: calibrate at the same resolution you will use for ArUco detection.
"""

from __future__ import annotations

import argparse
import logging
import os
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

RUNNING = True


def configure_logging(verbose: bool) -> None:
    level = logging.DEBUG if verbose else logging.INFO
    logging.basicConfig(
        level=level,
        format="%(asctime)s %(levelname)s %(message)s",
        datefmt="%Y-%m-%dT%H:%M:%S",
    )


def stop_handler(signum, frame) -> None:
    del signum, frame
    global RUNNING
    RUNNING = False


def timestamp_filename(index: int) -> str:
    ts = datetime.utcnow().strftime("%Y%m%d_%H%M%S")
    return f"cal_{index:04d}_{ts}.jpg"


def detect_checkerboard_bbox(frame, board_cols: int, board_rows: int):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    board_size = (board_cols, board_rows)

    if hasattr(cv2, "findChessboardCornersSB"):
        found, corners = cv2.findChessboardCornersSB(gray, board_size)
    else:
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
        found, corners = cv2.findChessboardCorners(gray, board_size, flags)

    if not found or corners is None or len(corners) == 0:
        return False, None

    xs = corners[:, 0, 0]
    ys = corners[:, 0, 1]
    top_left = (int(xs.min()), int(ys.min()))
    bottom_right = (int(xs.max()), int(ys.max()))
    return True, (top_left, bottom_right)


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
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.frame_event = threading.Event()
        self.stderr_lines: list[str] = []

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
        logging.info("Starting stream: %s", " ".join(cmd))
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

        if self.proc and self.proc.stderr:
            try:
                stderr_text = self.proc.stderr.read().decode("utf-8", errors="ignore")
                if stderr_text:
                    self.stderr_lines.append(stderr_text)
            except Exception:
                pass

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


def save_frame(output_dir: Path, index: int, frame, quality: int) -> Path:
    filename = output_dir / timestamp_filename(index)
    cv2.imwrite(str(filename), frame, [cv2.IMWRITE_JPEG_QUALITY, int(max(1, min(100, quality)))])
    return filename


def run_interactive(
    output_dir: Path,
    width: int,
    height: int,
    warmup: float,
    quality: int,
    board_cols: int,
    board_rows: int,
    hz: float,
    device: str,
) -> int:
    output_dir.mkdir(parents=True, exist_ok=True)
    stream = RPiCamMJPEGStream(width=width, height=height, hz=hz, device=device)

    try:
        stream.start()
    except RuntimeError as exc:
        logging.error("%s", exc)
        return 2

    logging.info("Warming up camera for %.1f seconds", warmup)
    time.sleep(max(0.0, warmup))

    index = 0
    print(f"\nSave directory: {output_dir}")
    print(f"Resolution:     {width}x{height}")
    print(f"Checkerboard inner corners: {board_cols}x{board_rows}")
    print("Hold the checkerboard flat and steady.")
    print("Press Enter to detect/capture, 'q'+Enter to quit.\n")

    try:
        while RUNNING:
            try:
                line = input(f"[{index} captured] Press Enter to capture (or 'q' to quit): ")
            except EOFError:
                break
            if line.strip().lower() == "q":
                break

            frame = stream.get_latest_frame(timeout=2.0)
            if frame is None:
                print("  Skip: No frame available", flush=True)
                continue

            found, bbox = detect_checkerboard_bbox(frame, board_cols, board_rows)
            if found:
                filename = save_frame(output_dir, index, frame, quality)
                index += 1
                top_left, bottom_right = bbox
                print(f"  Saved: {filename.name} | bbox tl={top_left} br={bottom_right}", flush=True)
            else:
                print("  Skip: No checkerboard detected", flush=True)
    finally:
        stream.stop()

    print(f"\nDone. {index} images saved to {output_dir}")
    return 0


def run_timed(
    output_dir: Path,
    width: int,
    height: int,
    warmup: float,
    quality: int,
    interval: float,
    count: int,
    board_cols: int,
    board_rows: int,
    hz: float,
    device: str,
) -> int:
    output_dir.mkdir(parents=True, exist_ok=True)
    stream = RPiCamMJPEGStream(width=width, height=height, hz=hz, device=device)

    try:
        stream.start()
    except RuntimeError as exc:
        logging.error("%s", exc)
        return 2

    logging.info("Warming up camera for %.1f seconds", warmup)
    time.sleep(max(0.0, warmup))

    print(f"\nSave directory: {output_dir}")
    print(f"Capturing up to {count} images every {interval:.1f}s (save only when checkerboard is detected).")
    print(f"Checkerboard inner corners: {board_cols}x{board_rows}\n")

    index = 0
    attempts = 0
    try:
        while RUNNING and index < count:
            attempts += 1
            frame = stream.get_latest_frame(timeout=2.0)
            if frame is None:
                print(f"  [skip {attempts}] No frame available", flush=True)
            else:
                found, bbox = detect_checkerboard_bbox(frame, board_cols, board_rows)
                if found:
                    filename = save_frame(output_dir, index, frame, quality)
                    index += 1
                    top_left, bottom_right = bbox
                    print(
                        f"  [{index}/{count}] Saved: {filename.name} | "
                        f"bbox tl={top_left} br={bottom_right}",
                        flush=True,
                    )
                else:
                    print(f"  [skip {attempts}] No checkerboard detected", flush=True)

            if index < count and RUNNING:
                time.sleep(interval)
    finally:
        stream.stop()

    print(f"\nDone. {index} images saved to {output_dir}")
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Capture checkerboard images for camera calibration")
    parser.add_argument(
        "--output-dir",
        default="./cal_images",
        help="Directory to save calibration images (default: ./cal_images)",
    )
    parser.add_argument(
        "--device",
        default="0",
        help="Camera index for rpicam-vid (default: 0)",
    )
    parser.add_argument(
        "--width",
        type=int,
        default=1296,
        help="Capture width in pixels — should match ArUco runtime resolution",
    )
    parser.add_argument(
        "--height",
        type=int,
        default=972,
        help="Capture height in pixels — should match ArUco runtime resolution",
    )
    parser.add_argument("--hz", type=float, default=2.0, help="MJPEG stream frame rate (default: 10)")
    parser.add_argument("--warmup", type=float, default=2.0, help="Camera warmup seconds (default: 2.0)")
    parser.add_argument(
        "--quality",
        type=int,
        default=95,
        help="JPEG quality 1-100 (default: 95, higher is better for calibration)",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.0,
        help="If > 0, capture automatically every N seconds (timed mode)",
    )
    parser.add_argument(
        "--count",
        type=int,
        default=25,
        help="Number of images to capture in timed mode (default: 25)",
    )
    parser.add_argument(
        "--board-cols",
        type=int,
        default=9,
        help="Checkerboard inner corner columns (default: 9)",
    )
    parser.add_argument(
        "--board-rows",
        type=int,
        default=6,
        help="Checkerboard inner corner rows (default: 6)",
    )
    parser.add_argument("--verbose", action="store_true", help="Enable debug logging")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    configure_logging(args.verbose)
    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)

    output_dir = Path(args.output_dir).expanduser()

    if args.interval > 0:
        return run_timed(
            output_dir,
            args.width,
            args.height,
            args.warmup,
            args.quality,
            args.interval,
            args.count,
            args.board_cols,
            args.board_rows,
            args.hz,
            args.device,
        )

    return run_interactive(
        output_dir,
        args.width,
        args.height,
        args.warmup,
        args.quality,
        args.board_cols,
        args.board_rows,
        args.hz,
        args.device,
    )


if __name__ == "__main__":
    sys.exit(main())
