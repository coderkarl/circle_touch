# 3π+ Line Sensor Geometry Reference

## Sensor Array Layout (From ir_sensors.py)

```
5 Line Sensors (IR Reflectance)
├─ Sensor 0 (Left)     │ ~14.4mm spacing │ ~0mm from left edge
├─ Sensor 1           │ ~14.4mm spacing │ ~14.4mm from left edge
├─ Sensor 2 (Center)  │ ~14.4mm spacing │ ~28.8mm from left edge
├─ Sensor 3           │ ~14.4mm spacing │ ~43.2mm from left edge
└─ Sensor 4 (Right)   │ ~14.4mm spacing │ ~57.6mm from left edge

2 Bump Sensors (also IR)
├─ Bump Left
└─ Bump Right
```

## Sensor Characteristics (from QTRSensors class)

### Output Values
- **Range:** 0-1023 (TIMEOUT = 1024)
- **0:** Black/dark (object close to sensor)
- **1023:** White/light (reflective surface far away)
- **Typical white surface:** 800-1023
- **Typical black line:** 0-200
- **Gray boundary:** 200-800

### Detection Method
The library uses a precise timing method:
1. Charge capacitors (all sensors = high)
2. Set pins to input (discharge through measured surface)
3. Count microseconds until pin goes low
4. Higher count = darker surface (longer discharge time)

### Calibration (From LineSensors class)

**Automatic Calibration:**
```python
line_sensors.calibrate()  # Take 10 measurements
# Updates: cal_min[] and cal_max[]
```

**Calibrated Output:**
```python
line = line_sensors.read_calibrated()  # Returns 0-1000
# 0 = darkest seen during calibration
# 1000 = brightest seen during calibration
```

## Your Current Implementation

### Threshold Settings
```python
line_threshold = 400  # Raw values < 400 = black
cross_detect_threshold = 2  # Min 2 sensors must see black
sensor_spacing_m = 0.0144  # 14.4mm between sensors
```

### Sensor Position Calculation
When black is detected:
```
center_of_black = sum(i * black_sensors[i]) / black_count
# i ranges 0-4 (sensor indices)
# Ideal center = 2.0 (middle sensor)

lateral_error = (center_of_black - 2.0) * 0.0144 meters
# Example: If sensors 1,2,3 see black → center_of_black = 2.0 → error = 0
# Example: If sensors 0,1,2 see black → center_of_black = 1.0 → error = -14.4mm
# Example: If sensors 2,3,4 see black → center_of_black = 3.0 → error = +14.4mm
```

## Practical Values to Expect

### Raw Sensor Readings (uncalibrated)
- **White paper background:** 700-900
- **Black cross lines:** 50-150
- **Threshold (400):** Good middle ground for black/white discrimination

### Detection Scenarios
- **Perfectly aligned with cross:** Sensors 1, 2, 3 see black (~150 each)
- **Offset left by ~14mm:** Sensors 0, 1, 2 see black
- **Offset right by ~14mm:** Sensors 2, 3, 4 see black
- **Far off alignment:** Only 0-1 sensor sees black (no detection)

## Tips for Optimal Detection

1. **Ensure Good Line Contrast:**
   - Black lines should read <200
   - White background should read >700
   - If readings are in middle range (300-700), calibrate sensors

2. **Check Lighting:**
   - IR sensors are affected by ambient light
   - Shadows can cause false positives
   - Test in controlled lighting conditions

3. **Line Width Matching:**
   - 1-inch black lines are ~25.4mm wide
   - At ideal alignment, will trigger 2-3 sensors
   - Thinner lines may only trigger 1-2 sensors

4. **Sensor Mounting:**
   - Should be ~5-10mm above floor
   - Should be centered on the robot
   - Any tilt will affect readings

## Debugging Cross Detection

Print sensor values to diagnose issues:
```python
print("Raw:", line)  # Raw 0-1023 values
black_count = sum(1 for v in line if v < 400)
print(f"Black sensors: {black_count}")

# If no detection:
# - Black line not dark enough (> 400)
# - Too few sensors seeing black (< 2)
# - Robot not aligned with cross

# If false detections:
# - Threshold too high (400)
# - Shadows causing dark readings
# - Dirty sensors
```

## Calibration Recommendation

For best results:
1. Place robot on white background
2. Run `ir_sensor_demo.py` and calibrate with button A
3. This establishes min/max values for your specific lighting/surfaces
4. Then use `read_calibrated()` for more reliable 0-1000 range

Currently your code uses `read()` for raw values, which is fine if your black lines are consistently <400 on white background >700.
