# Quick Reference: 3π+ Cross Detection

## Line Sensor Values (Raw 0-1023)
| Surface | Expected Value | Detection |
|---------|---|---|
| Black line | 0-200 | **Detected** (<400) |
| Dark gray | 200-400 | Boundary |
| Light gray | 400-700 | Not detected |
| White paper | 700-900 | Clear |
| Very white | 900-1023 | Very clear |

## Cross Detection Logic
```
Is sensor[i] < 400?
    No  → White surface (sensor = 0 in black_sensors array)
    Yes → Black surface (sensor = 1 in black_sensors array)

Sum black_sensors count:
    < 2 → Cross NOT detected
    ≥ 2 → Cross detected ✓
```

## Sensor Spacing
```
0   1   2   3   4
|---|---|---|---|  ← 14.4mm between each
└───────────────┘
    ~57.6mm total
```

## Position Correction Trigger
```
Cross detected? YES
    ↓
Within 10cm of target waypoint? YES
    ↓
Reset position to target coordinates
Reset accumulated drift
    ↓
Continue to next waypoint
```

## Current Parameters in circle_touch.py
```python
line_threshold = 400          # Sensitivity: <400 = black
cross_detect_threshold = 2    # Strictness: min 2 sensors
sensor_spacing_m = 0.0144     # 14.4mm per sensor
correction_distance = 0.1     # 10cm from target
```

## Tuning Guide

**Problem:** Cross not detected (no "X" on display)
- **Check 1:** Are line sensor values < 400? → Increase brightness or use `read_calibrated()`
- **Check 2:** Are 2+ sensors seeing black? → Check line thickness/darkness
- **Check 3:** Is robot aligned? → Check if offset > 2 sensors away

**Problem:** False cross detections
- **Solution 1:** Increase `line_threshold` to 450
- **Solution 2:** Increase `cross_detect_threshold` to 3
- **Solution 3:** Check for dirt/shadows on sensor

**Problem:** Corrections not working
- **Check:** Is robot within 10cm of target? → Check `dist_to_target` logic
- **Check:** Are corrections resetting position? → Verify `correct_position_at_target()` is called

## Console Output Interpretation
```
state 1, LSpd 750, RSpd 750, line [850, 750, 150, 200, 800]
                                     ↑    ↑    ↑   ↑   ↑
                                  white  white black black white
                                           ↑_______↑_______↑
                                           Cross detected at center-left
```

## Testing Checklist
- [ ] Robot drives toward waypoint
- [ ] Display shows position updating
- [ ] Robot stops ~3cm from target
- [ ] Console shows line[2] and line[3] < 400 when over cross
- [ ] Display shows "X" when cross detected
- [ ] Position corrects to exact target coordinates
- [ ] Robot moves to next waypoint
- [ ] No drift accumulation after multiple targets
