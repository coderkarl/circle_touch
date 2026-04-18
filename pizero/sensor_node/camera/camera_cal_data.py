#!/usr/bin/env python3
"""
camera_cal_data.py — Checkerboard calibration image capture for Pi Zero.

Usage:
  Interactive (press Enter to capture, 'q' to quit):
    python3 camera_cal_data.py --output-dir ./cal_images --width 1296 --height 972

  Timed (capture every N seconds for M images):
    python3 camera_cal_data.py --output-dir ./cal_images --interval 3.0 --count 25

After capture, copy images to a laptop and run camera_calibrate_offline.py.

IMPORTANT: always calibrate at the same resolution you will use for ArUco detection.
The default 1296x972 matches aruco_robot_pose.py defaults.
"""
import argparse
import importlib
import logging
import os
import signal
import sys
import time
from datetime import datetime
from pathlib import Path

RUNNING = True


def configure_logging(verbose: bool) -> None:
    level = logging.DEBUG if verbose else logging.INFO
    logging.basicConfig(
        level=level,
        format="%(asctime)s %(levelname)s %(message)s",
        datefmt="%Y-%m-%dT%H:%M:%S",
    )


def stop_handler(signum, frame):
    del signum, frame
    global RUNNING
    RUNNING = False


def timestamp_filename(index: int) -> str:
    ts = datetime.utcnow().strftime("%Y%m%d_%H%M%S")
    return f"cal_{index:04d}_{ts}.jpg"


def capture_jpeg(picam2, filename: Path, quality: int) -> None:
    try:
        picam2.capture_file(str(filename), format="jpeg", quality=quality)
    except TypeError:
        picam2.capture_file(str(filename), format="jpeg")


def detect_checkerboard_bbox(frame, board_cols: int, board_rows: int):
    cv2 = importlib.import_module("cv2")
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
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


def run_interactive(output_dir: Path, width: int, height: int, warmup: float,
                    quality: int, board_cols: int, board_rows: int) -> int:
    try:
        picamera2_mod = importlib.import_module("picamera2")
        Picamera2 = getattr(picamera2_mod, "Picamera2")
    except (ModuleNotFoundError, AttributeError):
        logging.error("Picamera2 not installed. Run: sudo apt install -y python3-picamera2")
        return 2

    try:
        importlib.import_module("cv2")
    except ModuleNotFoundError:
        logging.error("OpenCV not installed. Run: sudo apt install -y python3-opencv")
        return 2

    output_dir.mkdir(parents=True, exist_ok=True)

    picam2 = Picamera2()
    config = picam2.create_still_configuration(main={"size": (width, height), "format": "RGB888"})
    picam2.configure(config)

    logging.info("Starting camera %dx%d", width, height)
    picam2.start()
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
            frame = picam2.capture_array("main")
            found, bbox = detect_checkerboard_bbox(frame, board_cols, board_rows)
            if found:
                filename = output_dir / timestamp_filename(index)
                cv2 = importlib.import_module("cv2")
                bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                cv2.imwrite(str(filename), bgr, [cv2.IMWRITE_JPEG_QUALITY, int(max(1, min(100, quality)))])
                index += 1
                top_left, bottom_right = bbox
                print(
                    f"  Saved: {filename.name} | bbox tl={top_left} br={bottom_right}",
                    flush=True,
                )
            else:
                print("  Skip: No checkerboard detected", flush=True)
    finally:
        picam2.stop()

    print(f"\nDone. {index} images saved to {output_dir}")
    return 0


def run_timed(output_dir: Path, width: int, height: int, warmup: float,
              quality: int, interval: float, count: int,
              board_cols: int, board_rows: int) -> int:
    try:
        picamera2_mod = importlib.import_module("picamera2")
        Picamera2 = getattr(picamera2_mod, "Picamera2")
    except (ModuleNotFoundError, AttributeError):
        logging.error("Picamera2 not installed. Run: sudo apt install -y python3-picamera2")
        return 2

    try:
        importlib.import_module("cv2")
    except ModuleNotFoundError:
        logging.error("OpenCV not installed. Run: sudo apt install -y python3-opencv")
        return 2

    output_dir.mkdir(parents=True, exist_ok=True)

    picam2 = Picamera2()
    config = picam2.create_still_configuration(main={"size": (width, height), "format": "RGB888"})
    picam2.configure(config)

    logging.info("Starting camera %dx%d", width, height)
    picam2.start()
    time.sleep(max(0.0, warmup))

    print(f"\nSave directory: {output_dir}")
    print(f"Capturing up to {count} images every {interval:.1f}s (save only when checkerboard is detected).")
    print(f"Checkerboard inner corners: {board_cols}x{board_rows}\n")

    index = 0
    attempts = 0
    try:
        while RUNNING and index < count:
            attempts += 1
            frame = picam2.capture_array("main")
            found, bbox = detect_checkerboard_bbox(frame, board_cols, board_rows)
            if found:
                filename = output_dir / timestamp_filename(index)
                cv2 = importlib.import_module("cv2")
                bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                cv2.imwrite(str(filename), bgr, [cv2.IMWRITE_JPEG_QUALITY, int(max(1, min(100, quality)))])
                index += 1
                top_left, bottom_right = bbox
                print(
                    f"  [{index}/{count}] Saved: {filename.name} | "
                    f"bbox tl={top_left} br={bottom_right}",
                    flush=True,
                )
            else:
                print(f"  [skip {attempts}] No checkerboard detected", flush=True)
            if index < count:
                time.sleep(interval)
    finally:
        picam2.stop()

    print(f"\nDone. {index} images saved to {output_dir}")
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture checkerboard images for camera calibration"
    )
    parser.add_argument("--output-dir", default="./cal_images",
                        help="Directory to save calibration images (default: ./cal_images)")
    parser.add_argument("--width", type=int, default=1296,
                        help="Capture width in pixels — must match ArUco runtime resolution")
    parser.add_argument("--height", type=int, default=972,
                        help="Capture height in pixels — must match ArUco runtime resolution")
    parser.add_argument("--warmup", type=float, default=2.0,
                        help="Camera warmup seconds (default: 2.0)")
    parser.add_argument("--quality", type=int, default=95,
                        help="JPEG quality 1-100 (default: 95, higher is better for calibration)")
    parser.add_argument("--interval", type=float, default=0.0,
                        help="If > 0, capture automatically every N seconds (timed mode)")
    parser.add_argument("--count", type=int, default=25,
                        help="Number of images to capture in timed mode (default: 25)")
    parser.add_argument("--board-cols", type=int, default=9,
                        help="Checkerboard inner corner columns (default: 9)")
    parser.add_argument("--board-rows", type=int, default=6,
                        help="Checkerboard inner corner rows (default: 6)")
    parser.add_argument("--verbose", action="store_true", help="Enable debug logging")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    configure_logging(args.verbose)
    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)

    output_dir = Path(args.output_dir).expanduser()

    if args.interval > 0:
        return run_timed(output_dir, args.width, args.height, args.warmup,
                         args.quality, args.interval, args.count,
                         args.board_cols, args.board_rows)
    else:
        return run_interactive(output_dir, args.width, args.height, args.warmup,
                               args.quality, args.board_cols, args.board_rows)


if __name__ == "__main__":
    sys.exit(main())
