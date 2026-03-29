# Enhanced Circle Touch Robot - Advanced Features

## New Capabilities

Your circle_touch.py has been enhanced with three major features:

### 1. **Extended X-Axis Line Detection**
- Detects the long arm of the cross that extends ~2 inches beyond the circle
- Uses `detect_extended_x_axis_line()` function
- Helps identify the global +X axis reference
- Returns: (line_detected, position_left_center_right, quality)

### 2. **Heading Calibration at Home**
- On startup, when robot is at home (0, 0), it detects the cross
- Uses the extended X-axis line to establish that home direction = 0° (facing +X)
- Sets `initial_heading_estimate` and `initial_heading_calibrated` flags
- Subsequent heading measurements are calibrated relative to this baseline

### 3. **Heading Alignment at Targets**
- New `state_align_heading` state
- When cross is detected within 15cm of target:
  1. Robot enters alignment mode
  2. Slowly turns until heading is within ±6° of correct alignment
  3. Uses `estimate_heading_error_from_cross()` to guide turning
  4. Then corrects position and moves to next waypoint
- Slow turning (200 units/sec) allows precise alignment

### 4. **Spiral Search Pattern**
- New `state_spiral` state
- When robot reaches waypoint area but no cross found:
  1. Initiates expanding spiral search
  2. Spirals outward at ~10cm/sec radius increase
  3. Continuously turns while moving forward
  4. Forward speed ramps up over 5 seconds
  5. Exits if cross found or after 15 seconds
- Helps find targets even with dead-reckoning errors

## How It Works

### Startup Sequence
```
1. Robot powers on at (0, 0) facing +X
2. Navigates area, line sensors start reading
3. Detects cross at home
4. Uses extended X-axis line to set heading = 0°
5. initial_heading_calibrated = True
```

### Navigation to Target
```
1. Calculate heading to next waypoint
2. Navigate using proportional control
3. Get close to waypoint (< 3cm)
```

### Target Arrival - Case A: Cross Found
```
1. Cross detected within 15cm
2. Enter state_align_heading
3. Slowly turn to align heading
4. Once aligned (< 6° error):
   - Correct position to exact target
   - Reset odometry drift
   - Move to next waypoint
```

### Target Arrival - Case B: Cross Not Found
```
1. Near waypoint (< 3cm) but no cross
2. Enter state_spiral
3. Spiral outward while turning
4. If cross found during spiral:
   - Exit spiral
   - Enter alignment mode
5. If spiral timeout (15 sec):
   - Exit spiral
   - Continue to next waypoint
```

## New Parameters

```python
# Cross detection - updated to 0.75 inch actual thickness
cross_line_thickness = 0.75 * 0.0254  # meters
extended_x_arm_length = 2 * 0.0254    # extends beyond circle

# Spiral search parameters
spiral_radius_rate = 0.1         # meters per second expansion
spiral_turn_speed = 300          # slow turn during spiral
spiral_forward_speed = 200       # slow forward during spiral

# Heading alignment parameters
heading_align_threshold_rad = 0.1  # ±0.1 rad (~6°) is aligned
heading_align_turn_speed = 200     # slow turn speed

# Initial calibration
initial_heading_calibrated = False
initial_heading_estimate = 0.0
```

## New States

```python
state_stop = 0         # Stop and transition
state_track = 1        # Navigate to waypoint
state_turn = 2         # Turn in place
state_spiral = 4       # Spiral search pattern
state_align_heading = 5  # Align heading with cross
```

## Function Enhancements

### `detect_extended_x_axis_line(line_values, threshold=400)`
- Detects the extended +X arm of the cross
- Returns position: -1 (left), 0 (center), +1 (right)
- Used for initial heading calibration

### `estimate_heading_error_from_cross(line_values, center_y_detected, threshold=400)`
- Analyzes cross pattern to estimate heading error
- Uses sensor offset × 0.3 radians per sensor offset
- Returns heading error in radians
- Used for alignment control

### `correct_heading_at_target(bot_odom, target_waypoint, heading_error_rad)`
- Applies measured heading correction gradually
- Applies 10% of measured error per call
- Smoother than step correction

## Behavior Changes

### Before
- Robot went straight to waypoint
- Detected cross only if perfectly aligned
- Could miss targets if dead-reckoning drift occurred
- Position corrected abruptly when cross found

### After
- Robot detects extended X-line at home to calibrate heading
- Slowly aligns heading with cross before position correction
- Initiates spiral search if cross not found at waypoint
- Smooth heading alignment over several sensor cycles
- More robust to dead-reckoning uncertainty

## Console Output

You'll see new debug information:

```
state 4, LSpd 250, RSpd 350  <- Spiral search state
state 5, LSpd 200, RSpd -200  <- Alignment state
```

## Testing Recommendations

1. **Calibration Test**: Robot should detect cross at home and set heading = 0°
2. **Alignment Test**: When approaching target, watch robot slowly turn to align
3. **Spiral Test**: Place robot ~30cm from target, trigger spiral search
4. **Accuracy**: Position should be within ±5mm of target after alignment

## Parameter Tuning

If robot behavior needs adjustment:

```python
# Spiral too slow/fast:
spiral_radius_rate = 0.1  # increase for faster expansion

# Alignment too aggressive/gentle:
heading_align_threshold_rad = 0.1  # smaller = more precise
heading_align_turn_speed = 200     # larger = faster turning

# Cross detection sensitivity:
line_threshold = 400  # lower = more sensitive to black
```

## Uncertainty Handling

The system now handles:
- **Waypoint uncertainty**: ±15cm radius allows for dead-reckoning drift
- **Heading uncertainty**: Slow alignment ensures proper orientation before position reset
- **Target line thickness**: Uses actual 0.75" measurement for better detection
- **Extended reference**: The 2" X-arm extension provides unambiguous heading reference

## Known Behaviors

1. Spiral search may take up to 15 seconds if target is far from waypoint
2. Heading alignment continuously monitors cross pattern (no timeout)
3. Initial calibration only happens at home (waypoint 0)
4. Each target can have ~15cm error before spiral search triggers
