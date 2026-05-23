import argparse
import os
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from camera_utils import (
    RPiCamMJPEGStream as SharedRPiCamMJPEGStream,
    build_aruco_detector as SharedBuildArucoDetector,
    detect_markers as SharedDetectMarkers,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture camera stream with rpicam-vid, detect ArUco markers, save detection frames."
    )
    parser.add_argument(
        "--device",
        default="/dev/video0",
        help="Camera device index or path (default: /dev/video0)",
    )
    parser.add_argument("--hz", type=float, default=10.0, help="Capture rate in Hz (default: 10)")
    parser.add_argument("--width", type=int, default=1280, help="Frame width (default: 1280)")
    parser.add_argument("--height", type=int, default=720, help="Frame height (default: 720)")
    parser.add_argument(
        "--capture-all",
        action="store_true",
        help="Save every frame instead of only frames with ArUco detections",
    )
    return parser.parse_args()


def parse_device(device_arg: str) -> int | str:
    return int(device_arg) if device_arg.isdigit() else device_arg


def annotate_aruco(frame, corners, ids):
    if ids is not None and len(ids) > 0:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids, borderColor=(0, 255, 0))
    return frame


def save_frame(output_dir: Path, prefix: str, frame) -> Path:
    capture_ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    file_path = output_dir / f"{prefix}_{capture_ts}.jpg"
    cv2.imwrite(str(file_path), frame)
    return file_path


def run_rpicam_vid_stream(args: argparse.Namespace, output_dir: Path) -> None:
    """Capture frames from the shared MJPEG stream, detect ArUco, and save detections."""
    stream = SharedRPiCamMJPEGStream(width=args.width, height=args.height, hz=args.hz, device=args.device)
    stream.start()

    aruco_dict, detector, params, legacy = SharedBuildArucoDetector("DICT_4X4_50", {})

    mode_text = "all frames" if args.capture_all else "frames with detections"
    print(f"Detecting ArUco markers. Saving {mode_text} to {output_dir}.")
    print("Press Ctrl+C to stop.")

    frame_count = 0
    detections_count = 0

    try:
        while True:
            frame = stream.get_latest_frame(timeout=2.0)
            if frame is None:
                print("ERROR: no frame available from stream")
                break

            frame_count += 1
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids = SharedDetectMarkers(gray, aruco_dict, detector, params, legacy)

            should_save = args.capture_all or (ids is not None and len(ids) > 0)
            if should_save:
                annotated = frame.copy()
                annotate_aruco(annotated, corners, ids)
                prefix = "frame" if args.capture_all else "frame_with_aruco"
                save_frame(output_dir, prefix, annotated)

            if ids is not None and len(ids) > 0:
                detections_count += 1
                print(f"Found {len(ids)} ArUco marker(s) in frame {frame_count}, saved.")

            if frame_count % 100 == 0:
                print(f"Processed {frame_count} frames, {detections_count} with ArUco detections.")

    except KeyboardInterrupt:
        print(f"\nStopped. Processed {frame_count} frames, found ArUco in {detections_count}.")
    finally:
        stream.stop()


def main() -> None:
    args = parse_args()
    if args.hz <= 0:
        raise SystemExit("ERROR: --hz must be > 0")
    if args.width <= 0 or args.height <= 0:
        raise SystemExit("ERROR: --width and --height must be > 0")

    output_dir = Path.home() / "cam_test"
    output_dir.mkdir(parents=True, exist_ok=True)

    device = parse_device(args.device)
    period_s = 1.0 / args.hz
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        print(f"WARN: Camera not opened with OpenCV ({args.device})")
        run_rpicam_vid_stream(args, output_dir)
        return

    print(
        f"Saving frames to {output_dir} at {args.hz:g} Hz "
        f"({args.width}x{args.height}) from {args.device}. Press Ctrl+C to stop."
    )
    if args.capture_all:
        print("Capture-all mode enabled: every frame will be saved.")
    aruco_dict, detector, params, legacy = SharedBuildArucoDetector("DICT_4X4_50", {})
    frame_count = 0
    next_capture_time = time.monotonic()

    try:
        while True:
            now = time.monotonic()
            if now < next_capture_time:
                time.sleep(next_capture_time - now)

            ok, frame = cap.read()

            if ok and frame is not None:
                frame_count += 1
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                corners, ids = SharedDetectMarkers(gray, aruco_dict, detector, params, legacy)

                should_save = args.capture_all or (ids is not None and len(ids) > 0)
                if should_save:
                    annotated = frame.copy()
                    annotate_aruco(annotated, corners, ids)
                    prefix = "frame" if args.capture_all else "frame_with_aruco"
                    save_frame(output_dir, prefix, annotated)

                if frame_count % 10 == 0:
                    print(f"Captured {frame_count} frames")
            else:
                print("WARN: Frame read failed, switching to rpicam-vid stream")
                cap.release()
                run_rpicam_vid_stream(args, output_dir)
                return

            next_capture_time += period_s
    except KeyboardInterrupt:
        print(f"Stopped. Captured {frame_count} frames.")
    finally:
        cap.release()


if __name__ == "__main__":
    main()