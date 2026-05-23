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
import signal
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

from camera_utils import RPiCamMJPEGStream as SharedRPiCamMJPEGStream

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
    stream = SharedRPiCamMJPEGStream(width=width, height=height, hz=hz, device=device)

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
    stream = SharedRPiCamMJPEGStream(width=width, height=height, hz=hz, device=device)

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
