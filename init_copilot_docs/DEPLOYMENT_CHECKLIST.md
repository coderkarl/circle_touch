# Deployment Checklist: Cross Detection Implementation

## Pre-Deployment Review

### Code Understanding
- [ ] Read README_CROSS_DETECTION.md
- [ ] Understand how `detect_cross_pattern()` works
- [ ] Understand how `correct_position_at_target()` works
- [ ] Know what parameters to tune (threshold, detection distance)

### Hardware Verification
- [ ] Robot has 5 line sensors (downward-looking)
- [ ] Encoders are properly calibrated (counts_per_meter)
- [ ] Gyroscope is functioning
- [ ] Display and motors work

## Testing Before First Run

### 1. Sensor Calibration (5-10 minutes)
```
[ ] Connect robot to power
[ ] Open ir_sensor_demo.py from library
[ ] Place robot on white paper (your background)
[ ] Press button A to calibrate bump sensors
[ ] Continue calibration on white surface for 2-3 seconds
[ ] Press button A again to calibrate line sensors
[ ] Move robot side-to-side over white surface while calibrating
```

### 2. Sensor Value Verification (5 minutes)
```
[ ] Still in ir_sensor_demo.py
[ ] Place sensor over WHITE surface → values should be 700-900
[ ] Place sensor over BLACK line → values should be 0-200
[ ] If not: check sensor mounting, cleanliness, lighting
[ ] Note the actual values you see (for tuning threshold later)
```

### 3. Cross Pattern Testing (5-10 minutes)
```
[ ] Print a 6-inch circle with 1-inch cross on white paper
[ ] Place robot approaching the cross pattern
[ ] Run circle_touch.py
[ ] Watch console output: "line [values]"
[ ] When crossing black lines:
    - [ ] Center sensors (indices 1,2,3) should show <400
    - [ ] Side sensors should show values closer to white
    - [ ] Console should show 2-3 sensors below threshold
[ ] Display should show "X" when cross is detected
```

### 4. Position Correction Testing (5-10 minutes)
```
[ ] Place robot on course with test targets
[ ] Run circle_touch.py
[ ] Robot should:
    - [ ] Start at (0,0)
    - [ ] Navigate toward first waypoint
    - [ ] Stop near target (~3cm)
    - [ ] Detect cross ("X" on display)
    - [ ] Position should correct to target
    - [ ] Move to next target
[ ] After 2-3 targets:
    - [ ] Check that no drift has accumulated
    - [ ] Position should still be accurate
```

## First Deployment

### Initial Run Checklist
```
[ ] Course is set up with target circles and crosses
[ ] Black lines are clearly darker than white background
[ ] Adequate lighting (no heavy shadows)
[ ] Robot battery is fully charged
[ ] Display shows startup values
```

### Monitoring During Run
```
[ ] Watch position updates on display
[ ] Monitor console output for:
    - "line [values...]" - should show <400 over black
    - "state %d, LSpd %d, RSpd %d" - should show valid speeds
    - "X" indicator when crossing targets
[ ] Observe motor movements:
    - Proportional speed adjustment toward target
    - Turns when needed
    - Stops at targets
```

### Expected Milestones
```
Minute 0:    Robot starts, displays (0, 0, 0°)
Minute 1:    Robot is heading toward first target
Minute 2:    Robot is close to first target (< 5cm)
Minute 3:    Robot detects cross ("X" on display)
Minute 4:    Robot moves to second target
Minute 5+:   Robot continues to remaining targets
```

## Troubleshooting During Deployment

### Cross Not Detected
**Checklist:**
```
[ ] Is robot near the target? (should be within 10cm)
[ ] Are sensor values correct? (check console "line" output)
[ ] Are black values < 400? (if not, raise threshold)
[ ] Are 2+ sensors seeing black? (if not, check line thickness)
[ ] Are line sensors mounted correctly? (check height/angle)
```

**Actions:**
```
1. Increase line_threshold from 400 to 450
2. Run ir_sensor_demo.py to check actual values
3. Clean line sensors
4. Check black line darkness (may need darker marker)
```

### False Cross Detections
**Checklist:**
```
[ ] Is robot detecting crosses away from targets?
[ ] Are there shadows on the course?
[ ] Are sensors dirty?
[ ] Is correction_distance too large?
```

**Actions:**
```
1. Increase cross_detect_threshold from 2 to 3
2. Clean sensors and course
3. Improve lighting to eliminate shadows
4. Reduce correction_distance from 0.1 to 0.05
```

### Position Not Correcting
**Checklist:**
```
[ ] Is cross being detected? (check for "X" on display)
[ ] Is robot within 10cm of target? (check position values)
[ ] Is correct_position_at_target() being called? (add print statement)
```

**Actions:**
```
1. Add debug print: print(f"Correcting to {wp.x}, {wp.y}")
2. Increase correction_distance from 0.1 to 0.15
3. Verify waypoints are correct (check values in code)
```

### Drift Accumulation
**Checklist:**
```
[ ] Are encoders accurate? (test with known distance)
[ ] Is gyro drift compensated? (check gyro_bias_dps)
[ ] Are crosses being detected at targets? (check for "X")
```

**Actions:**
```
1. Re-calibrate encoders: drive known distance, compare
2. Adjust gyro_bias_dps in odom.py (currently -0.3)
3. Ensure line sensor calibration is fresh
4. Check that position corrections are being applied
```

## Post-Run Analysis

### Data to Collect
```
[ ] Final position accuracy at each target
[ ] Number of cross detections vs expected
[ ] Any positions where correction didn't work
[ ] Console output showing sensor values
```

### Metrics to Track
```
- Position accuracy at each waypoint (mm)
- Cross detection rate (% of targets where cross detected)
- Total drift over entire course (before/after)
- Number of false detections
```

### Improvement Notes
```
[ ] Which parameters worked best for your course?
[ ] What threshold value gave best results?
[ ] Did lighting affect detection?
[ ] Did line darkness matter?
[ ] Any waypoints that had problems?
```

## Tuning Parameters (if needed)

### Conservative Tuning (Most reliable)
```python
line_threshold = 350              # Stricter black detection
cross_detect_threshold = 3        # Need 3 sensors
correction_distance = 0.05        # Only correct very close
```

### Aggressive Tuning (Fastest drift correction)
```python
line_threshold = 450              # More forgiving black
cross_detect_threshold = 2        # Standard 2 sensors
correction_distance = 0.15        # Correct further away
```

### Your Optimized Settings (after testing)
```python
line_threshold = ___              # Your value
cross_detect_threshold = __       # Your value
correction_distance = 0.__        # Your value
```

## Success Criteria

✓ **Minimal drift** - Position accurate within ±5cm after 3+ targets
✓ **Reliable detection** - Crosses detected at 90%+ of targets
✓ **Fast correction** - Position resets within 1 second of cross detection
✓ **Smooth navigation** - Robot transitions smoothly between waypoints
✓ **No false positives** - No corrections away from targets

## Final Checklist Before Production

```
[ ] All tests passed
[ ] Parameters tuned for your specific course
[ ] Documentation updated with your settings
[ ] Robot battery tested and working
[ ] Course fully marked with circles and crosses
[ ] Adequate lighting verified
[ ] Emergency stop procedure tested
[ ] Backup of working code created
```

---

**Questions?** Refer to:
- Quick problems → QUICK_REFERENCE.md
- How it works → SENSOR_GEOMETRY.md
- Getting started → README_CROSS_DETECTION.md
