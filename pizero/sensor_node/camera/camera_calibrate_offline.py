#!/usr/bin/env python3
"""
camera_calibrate_offline.py — Solve camera intrinsics from checkerboard images.

Run on a laptop (or on the Pi Zero if needed).
Reads images captured by camera_cal_data.py and outputs camera_intrinsics.json.

Usage:
  python3 camera_calibrate_offline.py \\
      --images ./cal_images \\
      --rows 6 --cols 9 --square-mm 25.0 \\
      --output camera_intrinsics.json

  --rows, --cols: number of INTERNAL corners (not squares).
      A 10x7 square board has 9x6 internal corners.
  --square-mm: physical size of each square in millimeters.
      Must match what you printed. Verify with a ruler before calibrating.

The output JSON contains:
  camera_matrix  : 3x3 intrinsic matrix [[fx,0,cx],[0,fy,cy],[0,0,1]]
  dist_coeffs    : distortion coefficients [k1,k2,p1,p2,k3]
  image_size     : [width, height] in pixels
  rms_px         : RMS reprojection error in pixels (target < 1.0)
  pattern_rows   : internal corners in Y
  pattern_cols   : internal corners in X
  square_mm      : physical square size used
  num_images     : number of images successfully processed
  timestamp      : ISO 8601 calibration timestamp

After running, copy camera_intrinsics.json to the Pi Zero at:
  /home/pi/sensor_node/camera/camera_intrinsics.json
"""
import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np


def find_checkerboard_images(image_dir: Path, pattern_size: tuple) -> list:
    """Find all images where checkerboard corners are detected."""
    rows, cols = pattern_size
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # Prepare object points: (0,0,0), (1,0,0), ..., (cols-1, rows-1, 0) in square units
    objp = np.zeros((rows * cols, 3), dtype=np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)

    object_points = []  # 3D points in board space
    image_points = []   # 2D points in image space
    image_size = None
    good_images = []
    failed_images = []

    extensions = {".jpg", ".jpeg", ".png", ".bmp"}
    image_files = sorted(
        f for f in image_dir.iterdir()
        if f.is_file() and f.suffix.lower() in extensions
    )

    if not image_files:
        print(f"No images found in {image_dir}", file=sys.stderr)
        return [], [], [], None, [], []

    print(f"Processing {len(image_files)} images...")

    for img_path in image_files:
        img = cv2.imread(str(img_path))
        if img is None:
            print(f"  SKIP (could not read): {img_path.name}")
            failed_images.append(img_path)
            continue

        h, w = img.shape[:2]
        if image_size is None:
            image_size = (w, h)
        elif image_size != (w, h):
            print(f"  SKIP (size mismatch {w}x{h} vs {image_size[0]}x{image_size[1]}): {img_path.name}")
            failed_images.append(img_path)
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (cols, rows), None)

        if ret:
            # Refine corner locations to sub-pixel accuracy
            corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            object_points.append(objp.copy())
            image_points.append(corners_refined)
            good_images.append(img_path)
            print(f"  OK: {img_path.name}")
        else:
            print(f"  FAIL (no corners found): {img_path.name}")
            failed_images.append(img_path)

    return object_points, image_points, good_images, image_size, failed_images, image_files


def run_calibration(object_points: list, image_points: list,
                    image_size: tuple, square_mm: float) -> dict:
    """Run camera calibration and return result dict."""
    # Scale object points from square units to millimeters
    scaled_obj_pts = [p * square_mm for p in object_points]

    print(f"\nRunning calibration with {len(object_points)} images...")
    rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        scaled_obj_pts, image_points, image_size, None, None
    )

    return {
        "camera_matrix": camera_matrix.tolist(),
        "dist_coeffs": dist_coeffs.flatten().tolist(),
        "rms_px": float(rms),
        "rvecs": [r.flatten().tolist() for r in rvecs],
        "tvecs": [t.flatten().tolist() for t in tvecs],
    }


def compute_per_image_error(object_points, image_points, rvecs, tvecs,
                             camera_matrix, dist_coeffs, square_mm):
    """Compute and print reprojection error per image."""
    cm = np.array(camera_matrix, dtype=np.float64)
    dc = np.array(dist_coeffs, dtype=np.float64).flatten()
    errors = []
    for i, (op, ip) in enumerate(zip(object_points, image_points)):
        proj, _ = cv2.projectPoints(op * square_mm, rvecs[i], tvecs[i], cm, dc)
        err = float(np.sqrt(np.mean((ip - proj) ** 2)))
        errors.append(err)
    return errors


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Solve camera intrinsics from checkerboard calibration images"
    )
    parser.add_argument("--images", required=True,
                        help="Directory containing checkerboard images (from camera_cal_data.py)")
    parser.add_argument("--rows", type=int, default=6,
                        help="Internal corner rows (default: 6 for a 10x7 board)")
    parser.add_argument("--cols", type=int, default=9,
                        help="Internal corner cols (default: 9 for a 10x7 board)")
    parser.add_argument("--square-mm", type=float, default=25.0,
                        help="Physical size of each square in mm (default: 25.0)")
    parser.add_argument("--output", default="camera_intrinsics.json",
                        help="Output JSON file path (default: camera_intrinsics.json)")
    parser.add_argument("--annotate-dir", default="",
                        help="If set, save annotated images with detected corners to this dir")
    args = parser.parse_args()

    image_dir = Path(args.images).expanduser()
    if not image_dir.is_dir():
        print(f"Image directory not found: {image_dir}", file=sys.stderr)
        return 1

    pattern_size = (args.rows, args.cols)

    object_points, image_points, good_images, image_size, failed, all_files = \
        find_checkerboard_images(image_dir, pattern_size)

    print(f"\nResults: {len(good_images)} good / {len(failed)} failed / {len(all_files)} total")

    if len(good_images) < 6:
        print("ERROR: need at least 6 valid checkerboard images. Capture more.", file=sys.stderr)
        return 1

    result = run_calibration(object_points, image_points, image_size, args.square_mm)

    cm = np.array(result["camera_matrix"], dtype=np.float64)
    dc = np.array(result["dist_coeffs"], dtype=np.float64)
    rvecs_np = [np.array(r) for r in result["rvecs"]]
    tvecs_np = [np.array(t) for t in result["tvecs"]]

    per_image_errors = compute_per_image_error(
        object_points, image_points, rvecs_np, tvecs_np, cm, dc, args.square_mm
    )

    print(f"\nCalibration complete:")
    print(f"  RMS reprojection error: {result['rms_px']:.4f} px")
    print(f"  fx={cm[0,0]:.2f}  fy={cm[1,1]:.2f}")
    print(f"  cx={cm[0,2]:.2f}  cy={cm[1,2]:.2f}")
    print(f"  dist_coeffs: {[f'{v:.6f}' for v in dc.tolist()]}")
    print()

    if result["rms_px"] > 1.5:
        print("WARNING: RMS > 1.5 px. Re-capture with more varied board poses.")
    elif result["rms_px"] > 1.0:
        print("NOTE: RMS > 1.0 px. Acceptable but try to get below 1.0 for best ArUco pose accuracy.")
    else:
        print("RMS looks good (< 1.0 px).")

    print("\nPer-image error (px):")
    for path, err in zip(good_images, per_image_errors):
        flag = "  <-- high" if err > 2.0 * result["rms_px"] else ""
        print(f"  {path.name}: {err:.3f}{flag}")

    if args.annotate_dir:
        ann_dir = Path(args.annotate_dir)
        ann_dir.mkdir(parents=True, exist_ok=True)
        for img_path, ip in zip(good_images, image_points):
            img = cv2.imread(str(img_path))
            cv2.drawChessboardCorners(img, (args.cols, args.rows), ip, True)
            cv2.imwrite(str(ann_dir / img_path.name), img)
        print(f"\nAnnotated images saved to {ann_dir}")

    output = {
        "camera_matrix": result["camera_matrix"],
        "dist_coeffs": result["dist_coeffs"],
        "image_size": list(image_size),
        "rms_px": result["rms_px"],
        "pattern_rows": args.rows,
        "pattern_cols": args.cols,
        "square_mm": args.square_mm,
        "num_images": len(good_images),
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "_note": (
            "camera_matrix is [[fx,0,cx],[0,fy,cy],[0,0,1]]. "
            "dist_coeffs is [k1,k2,p1,p2,k3]. "
            "Units: pixels for fx/fy/cx/cy."
        ),
    }

    out_path = Path(args.output).expanduser()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w") as f:
        json.dump(output, f, indent=2)
    print(f"\nSaved: {out_path}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
