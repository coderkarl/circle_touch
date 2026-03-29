# Circle Touch Robot - Position Correction Update

## Summary
Enhanced the circle_touch.py program to use the black target crosses (marked on the course) to correct dead-reckoned position and heading. This adds absolute position feedback to the odometry system.

## Key Changes

### 1. Cross Detection (`detect_cross_pattern` function)
- Analyzes the 5 line sensor readings to detect when the robot crosses over a black cross
- Returns:
  - `cross_detected`: Boolean indicating if a cross pattern was found
  - `lateral_error_m`: Measured lateral offset from cross center in meters
  - `cross_quality`: Quality metric (number of sensors detecting black, 0-5)
- **How it works:**
  - Line sensors return values 0-1023 (0=dark/black, 1023=light/white)
  - Values below 400 are considered "black"
  - When 2+ sensors detect black, a cross is recognized
  - The position of detected black sensors indicates robot's lateral offset
  - Offset is calculated based on actual 3π+ sensor spacing (14.4mm per sensor)

### 2. Position Correction (`correct_position_at_target` function)
- When a cross is detected near the target waypoint (within 10cm), the odometry is corrected
- Sets the robot's position to exactly the target waypoint coordinates
- This eliminates accumulated dead-reckoning drift before moving to the next waypoint

### 3. Integration with Main Loop
- Cross detection happens every odom update cycle (50ms)
- Position correction only applies when:
  - A cross pattern is detected, AND
  - The robot is within 10cm of the current target waypoint
- This prevents false corrections from crosses at other locations

### 4. Visual Feedback
- Display now shows "X" when a cross is detected
- Console prints full line sensor array for debugging

## Technical Details

### Target Geometry
- **Circle diameter:** 6 inches (0.1524 m)
- **Cross arms:** Extend to circle perimeter
- **Cross line thickness:** 1 inch (0.0254 m)
- **X-axis extension:** ~2 inches past the circle (0.0508 m)

### Sensor Parameters (from actual 3π+ hardware)
- **Line threshold:** 400 (raw sensor values 0-1023: <400 = black, >400 = white)
- **Cross detect threshold:** 2 (minimum 2 sensors must see black)
- **Sensor spacing:** 14.4mm between sensors on 3π+ (0.0144m)
- **Total coverage:** ~57.6mm across robot
- **Correction distance:** Applies when within 10cm of waypoint

### Hardware Details from Library Analysis
- **Sensors:** 5 IR reflectance sensors in downward-looking array
- **Raw range:** 0-1023 (0=dark, 1023=light)
- **Calibrated range:** Optional 0-1000 (requires calibration)
- **Update rate:** Up to 8MHz PIO-based counting
- **Coordinates:** Sensor 0=left, Sensor 2=center, Sensor 4=right

### Coordinate Frame
- Global origin at home position (0, 0)
- +X axis = robot front direction when starting
- +Y axis = robot left direction when starting
- Heading (bot_rad) in radians: 0 = facing +X

## How It Works in Practice

1. **Dead Reckoning:** Encoder and gyro continuously update position
2. **Cross Detection:** As robot approaches target, line sensors detect the black cross pattern
3. **Position Correction:** When cross is detected near target, odometry is corrected to exact target location
4. **Error Reset:** This resets accumulated navigation errors before moving to next waypoint

## Expected Improvements
- Reduced drift over multiple waypoints
- Better heading alignment as robot approaches targets
- More accurate final positioning at each waypoint
- Better robustness for longer courses

## Future Enhancements
- Smooth correction instead of step function (extended Kalman filter)
- Better heading estimation from cross orientation
- Cross detection at arbitrary locations (not just near current target)
- Multi-cross pattern recognition
