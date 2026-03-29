#!/usr/bin/env python3
"""
camera_cal_data.py — Checkerboard calibration image capture for Pi Zero.

Usage:
  Interactive (press Enter to capture, 'q' to quit):
    python3 camera_cal_data.py --output-dir ./cal_images --width 1280 --height 720

  Timed (capture every N seconds for M images):
    python3 camera_cal_data.py --output-dir ./cal_images --interval 3.0 --count 25

After capture, copy images to a laptop and run camera_calibrate_offline.py.

IMPORTANT: always calibrate at the same resolution you will use for ArUco detection.
The default 1280x720 matches aruco_robot_pose.py defaults.
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


def run_interactive(output_dir: Path, width: int, height: int, warmup: float, quality: int) -> int:
    try:
        picamera2_mod = importlib.import_module("picamera2")
        Picamera2 = getattr(picamera2_mod, "Picamera2")
    except (ModuleNotFoundError, AttributeError):
        logging.error("Picamera2 not installed. Run: sudo apt install -y python3-picamera2")
        return 2

    output_dir.mkdir(parents=True, exist_ok=True)

    picam2 = Picamera2()
    config = picam2.create_still_configuration(main={"size": (width, height)})
    picam2.configure(config)

    logging.info("Starting camera %dx%d", width, height)
    picam2.start()
    time.sleep(max(0.0, warmup))

    index = 0
    print(f"\nSave directory: {output_dir}")
    print(f"Resolution:     {width}x{height}")
    print("Hold the checkerboard flat and steady.")
    print("Press Enter to capture, 'q'+Enter to quit.\n")

    try:
        while RUNNING:
            try:
                line = input(f"[{index} captured] Press Enter to capture (or 'q' to quit): ")
            except EOFError:
                break
            if line.strip().lower() == "q":
                break
            filename = output_dir / timestamp_filename(index)
            picam2.capture_file(str(filename), format="jpeg", quality=quality)
            index += 1
            print(f"  Saved: {filename.name}")
    finally:
        picam2.stop()

    print(f"\nDone. {index} images saved to {output_dir}")
    return 0


def run_timed(output_dir: Path, width: int, height: int, warmup: float,
              quality: int, interval: float, count: int) -> int:
    try:
        picamera2_mod = importlib.import_module("picamera2")
        Picamera2 = getattr(picamera2_mod, "Picamera2")
    except (ModuleNotFoundError, AttributeError):
        logging.error("Picamera2 not installed. Run: sudo apt install -y python3-picamera2")
        return 2

    output_dir.mkdir(parents=True, exist_ok=True)

    picam2 = Picamera2()
    config = picam2.create_still_configuration(main={"size": (width, height)})
    picam2.configure(config)

    logging.info("Starting camera %dx%d", width, height)
    picam2.start()
    time.sleep(max(0.0, warmup))

    print(f"\nSave directory: {output_dir}")
    print(f"Capturing {count} images every {interval:.1f}s — move the board between captures.\n")

    index = 0
    try:
        while RUNNING and index < count:
            filename = output_dir / timestamp_filename(index)
            picam2.capture_file(str(filename), format="jpeg", quality=quality)
            index += 1
            print(f"  [{index}/{count}] Saved: {filename.name}", flush=True)
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
    parser.add_argument("--width", type=int, default=1280,
                        help="Capture width in pixels — must match ArUco runtime resolution")
    parser.add_argument("--height", type=int, default=720,
                        help="Capture height in pixels — must match ArUco runtime resolution")
    parser.add_argument("--warmup", type=float, default=2.0,
                        help="Camera warmup seconds (default: 2.0)")
    parser.add_argument("--quality", type=int, default=95,
                        help="JPEG quality 1-100 (default: 95, higher is better for calibration)")
    parser.add_argument("--interval", type=float, default=0.0,
                        help="If > 0, capture automatically every N seconds (timed mode)")
    parser.add_argument("--count", type=int, default=25,
                        help="Number of images to capture in timed mode (default: 25)")
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
                         args.quality, args.interval, args.count)
    else:
        return run_interactive(output_dir, args.width, args.height, args.warmup, args.quality)


if __name__ == "__main__":
    sys.exit(main())
