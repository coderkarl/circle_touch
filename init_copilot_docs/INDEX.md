# Circle Touch Robot - Complete Documentation Index

## Program Files

### circle_touch.py (UPDATED)
The main robot control program with enhanced cross detection:
- Navigates to waypoints using encoders and gyro
- Detects black target crosses with line sensors
- Corrects odometry drift when crosses are detected
- Displays position and detects cross indicator ("X") on screen

### odom.py
Odometry calculation class:
- Integrates encoder counts into distance traveled
- Integrates gyro data into heading
- Maintains current x, y position and heading (bot_rad)
- Counts per meter calibration for your specific robot

## Documentation Files

### README_CROSS_DETECTION.md ← START HERE
Complete overview of the system:
- What was implemented and why
- How the cross detection works
- Testing recommendations
- Performance expectations
- Next steps and troubleshooting

### IMPLEMENTATION_SUMMARY.md
Technical details of the implementation:
- Key improvements from library analysis
- Hardware specifications verified
- How to use the updated code
- Tuning recommendations

### SENSOR_GEOMETRY.md
Detailed sensor specifications and geometry:
- Sensor array layout (5 sensors, 14.4mm spacing)
- Sensor characteristics (0-1023 range)
- Output values and detection thresholds
- Practical values to expect
- Debugging guide with example code

### QUICK_REFERENCE.md
Quick lookup for common questions:
- Sensor value table
- Cross detection logic diagram
- Parameter reference
- Tuning guide
- Testing checklist
- Console output interpretation

### CHANGES.md
What was changed and why:
- Cross detection function improvements
- Position correction algorithm
- Integration with main loop
- Hardware reference from actual 3π+ library

## Reference Material

### pololu-3pi-2040-robot/ (Cloned Library)
Official Pololu library code for 3π+ 2040 Robot:

**Key files for understanding sensors:**
- `micropython_demo/pololu_3pi_2040_robot/ir_sensors.py` - Line sensor implementation (QTR-based)
- `micropython_demo/pololu_3pi_2040_robot/encoders.py` - Encoder quadrature counter
- `micropython_demo/pololu_3pi_2040_robot/imu.py` - Gyroscope and accelerometer

**Example programs:**
- `micropython_demo/ir_sensor_demo.py` - Interactive sensor testing
- `micropython_demo/line_follower.py` - PID-based line following
- `micropython_demo/encoder_test.py` - Encoder testing
- `micropython_demo/gyro_turn.py` - Gyro-based turning

## How to Get Started

### 1. Understand the System (5 min)
Read: **README_CROSS_DETECTION.md**

### 2. Know Your Hardware (5 min)
Read: **SENSOR_GEOMETRY.md** "Sensor Array Layout" section

### 3. Test the Implementation (10-15 min)
1. Run `ir_sensor_demo.py` from the library
2. Calibrate line sensors with button A
3. Observe raw sensor values over black and white surfaces
4. Check that black (<400) and white (>700) discrimination works

### 4. Deploy and Test (20-30 min)
1. Place robot on course with target crosses
2. Run `circle_touch.py`
3. Monitor console output for "line [values]"
4. Check display for "X" when crossing targets
5. Verify position corrections with odometry

### 5. Tune if Needed (5-10 min)
Refer: **QUICK_REFERENCE.md** "Tuning Guide" section

## Key Parameters in circle_touch.py

```python
# Line Sensor Detection (line 165-168)
line_threshold = 400          # Black/white discrimination (0-1023)
cross_detect_threshold = 2    # Min sensors seeing black
sensor_spacing_m = 0.0144     # 14.4mm per sensor
# Position Correction (line 191)
if dist_to_target < 0.1:      # Activate when within 10cm
```

## Expected Behavior

**At startup:**
- Robot initializes at (0, 0) with heading = 0
- Display shows position, heading, gyro rate

**When navigating:**
- Robot calculates heading to target
- Proportional control adjusts motor speeds
- Position updates continuously from odometry

**When approaching target:**
- Robot slows as it gets close
- Line sensors start detecting black cross
- Display shows "X" when cross is detected
- Position corrects to exact target (eliminates drift)

**After correction:**
- All accumulated errors reset
- Next waypoint is selected
- Process repeats

## Troubleshooting Quick Links

| Problem | Solution |
|---------|----------|
| Cross not detected | See QUICK_REFERENCE.md "Problem: Cross not detected" |
| False detections | See QUICK_REFERENCE.md "Problem: False cross detections" |
| Corrections not working | See QUICK_REFERENCE.md "Problem: Corrections not working" |
| Need sensor values | Run `ir_sensor_demo.py` from library |
| Need example code | Check `micropython_demo/*.py` files in library |
| Need specifications | Read SENSOR_GEOMETRY.md or `ir_sensors.py` source |

## File Organization Summary

```
circle_touch/
├── Program Code
│   ├── circle_touch.py          (Main program - UPDATED)
│   └── odom.py                  (Odometry - unchanged)
│
├── Documentation (Read in this order)
│   ├── README_CROSS_DETECTION.md   ← START HERE (Overview)
│   ├── IMPLEMENTATION_SUMMARY.md   (Technical details)
│   ├── SENSOR_GEOMETRY.md          (Hardware specs)
│   ├── QUICK_REFERENCE.md          (Troubleshooting)
│   ├── CHANGES.md                  (What changed)
│   └── (this file)
│
└── Reference Library
    └── pololu-3pi-2040-robot/   (Cloned Pololu library)
```

## Contact/Reference

For detailed implementation questions, see the library source code in:
`pololu-3pi-2040-robot/micropython_demo/pololu_3pi_2040_robot/`

For general Pololu 3π+ information:
https://www.pololu.com/product/4964

## Version Information

- **3π+ Robot:** 2040 Model
- **Library:** Cloned from pololu-3pi-2040-robot
- **Circle_Touch Implementation:** Cross detection version
- **Date:** January 2026
