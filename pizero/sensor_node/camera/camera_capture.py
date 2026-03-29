#!/usr/bin/env python3
import argparse
import importlib
import logging
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


def timestamp_filename(prefix: str, ext: str = "jpg") -> str:
    ts = datetime.utcnow().strftime("%Y%m%d_%H%M%S_%f")
    return f"{prefix}_{ts}.{ext}"


def run(output_dir: Path, interval: float, width: int, height: int, warmup: float, quality: int, prefix: str) -> int:
    try:
        picamera2_mod = importlib.import_module("picamera2")
        Picamera2 = getattr(picamera2_mod, "Picamera2")
    except (ModuleNotFoundError, AttributeError):
        logging.error("Picamera2 is not installed. Run: sudo apt install -y python3-picamera2")
        return 2

    output_dir.mkdir(parents=True, exist_ok=True)

    picam2 = Picamera2()
    config = picam2.create_still_configuration(main={"size": (width, height)})
    picam2.configure(config)

    logging.info("Starting camera with resolution %dx%d", width, height)
    picam2.start()
    time.sleep(max(0.0, warmup))

    capture_count = 0
    try:
        while RUNNING:
            filename = output_dir / timestamp_filename(prefix)
            picam2.capture_file(str(filename), format="jpeg", quality=quality)
            capture_count += 1
            print(f"captured={capture_count} file={filename}", flush=True)
            if interval <= 0:
                break
            time.sleep(interval)
    finally:
        logging.info("Stopping camera")
        picam2.stop()

    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Periodic Pi Camera still capture")
    parser.add_argument("--output-dir", default="./captures", help="Directory for images")
    parser.add_argument("--interval", type=float, default=1.0, help="Seconds between captures; <=0 captures once")
    parser.add_argument("--width", type=int, default=1280, help="Image width")
    parser.add_argument("--height", type=int, default=720, help="Image height")
    parser.add_argument("--warmup", type=float, default=1.0, help="Camera warmup seconds")
    parser.add_argument("--quality", type=int, default=90, help="JPEG quality 1-100")
    parser.add_argument("--prefix", default="cam", help="Filename prefix")
    parser.add_argument("--verbose", action="store_true", help="Enable debug logging")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    configure_logging(args.verbose)
    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)

    return run(
        output_dir=Path(args.output_dir).expanduser(),
        interval=args.interval,
        width=args.width,
        height=args.height,
        warmup=args.warmup,
        quality=args.quality,
        prefix=args.prefix,
    )


if __name__ == "__main__":
    sys.exit(main())
