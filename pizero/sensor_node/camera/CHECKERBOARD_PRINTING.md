# Checkerboard Printing Instructions

## File

`checkerboard_a4_9x6_25mm.svg`

`checkerboard_a4_9x6_25mm.pdf` (preferred for printing on Ubuntu)

## Board Spec

| Property | Value |
|---|---|
| Paper size | A4 Landscape (297 × 210 mm) |
| Squares | 10 columns × 7 rows |
| Internal corners | 9 × 6 (pass these to `--cols 9 --rows 6` in calibration) |
| Square size | **25.0 mm** (pass `--square-mm 25.0` to calibration solver) |
| Board area | 250 × 175 mm (fits with ~23.5 mm margins on each side) |

## How to Print

1. Open the SVG in a browser, Inkscape, or any PDF viewer that respects physical dimensions.
2. **Print at 100% scale — do NOT enable "Fit to page" or "Scale to fit".** This is the single most important step.
3. Select **A4 Landscape** in the printer dialog.
4. Print on plain white paper (not glossy — glossy causes glare in camera images).

### Ubuntu (Image Viewer) Workaround

Ubuntu Image Viewer often auto-scales SVGs for the printable area, which can shrink the board.

Use one of these instead:

- **Inkscape**: Open SVG → `File` → `Print` → set scale to **100%** and disable fit/shrink options.
- **PDF path (recommended)**:

```bash
cd /home/karl/robots/circle_touch/pizero/sensor_node/camera
rsvg-convert -f pdf -o checkerboard_a4_9x6_25mm.pdf checkerboard_a4_9x6_25mm.svg
```

Then open the PDF in Document Viewer (Evince) and print with:

- Paper size: **A4**
- Orientation: **Landscape**
- Page scaling: **None / 100% / Actual size**

If your printer has an option like "Fit to printable area", keep it **off**.

## Verify After Printing

Measure several squares with a ruler or calipers:

- Each square should measure **25.0 mm ± 0.5 mm**.
- If squares are a different size, check your printer's scale setting and reprint.
- The entire board (10×7 squares) should measure **250 mm × 175 mm**.
- Quick inch check: 10 squares × 25 mm = **250 mm ≈ 9.84 in** (not exactly 10 in).

If the measured size differs significantly (e.g., printer scaled to Letter paper), either:
- Reprint at the correct scale on A4 paper, or
- Measure the actual printed square size and pass that value to `--square-mm` in the calibration solver.

## Mounting the Board

- Glue or tape the print to a **rigid flat surface** (cardboard, foam board, etc.).
- A flat board gives better corner-finding accuracy than a floppy paper sheet.
- The board does not need to be level — varied tilt angles are good for calibration.

## Calibration Reminder

When capturing:\
- Hold the board at varied distances (0.3 m to 0.8 m from camera).
- Tilt and rotate the board in different orientations between captures.
- Cover all corners of the camera frame across the image set.
- Aim for 20–30 images.

Pass to calibration solver:
```bash
python3 camera_calibrate_offline.py \
    --images ./cal_images \
    --rows 6 --cols 9 --square-mm 25.0 \
    --output camera_intrinsics.json
```
