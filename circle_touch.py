# This example makes the 3pi+ 2040 drive forward until it hits a wall, detect
# the collision with its bumpers, then reverse, turn, and keep driving.

from pololu_3pi_2040_robot import robot
from pololu_3pi_2040_robot.extras import editions
import time
import odom
import math

line_sensors = robot.LineSensors()
encoders = robot.Encoders()
motors = robot.Motors()
bot_odom = odom.Odom()
bump_sensors = robot.BumpSensors()
buzzer = robot.Buzzer()
display = robot.Display()
yellow_led = robot.YellowLED()

imu = robot.IMU()
imu.reset()
imu.enable_default()

edition = "Hyper"
if edition == "Hyper":
    max_speed = 1125
    left_nom_speed = 750
    right_nom_speed = 750
    turn_speed = 700
    turn_time = 150
    motors.flip_left(True)
    motors.flip_right(True)

display.fill(0)
display.show()

bump_sensors.calibrate()

line_sensors.start_read()

time.sleep_ms(1000)

now = time.ticks_ms()
odom_time = now
disp_time = now

odom_period_msec = 50
disp_period_msec = 500
yaw_rate_deg = 0.0

prev_dist = 0.0

state_stop = 0
state_track = 1
state_rev = -1
state_turn = 2
state_seek = 3
state_spiral = 4
state_align_heading = 5

state = state_track
next_state = state_track

class Point():
    def __init__(self, ax, ay):
        self.x = ax
        self.y = ay
        
    def __add__(self, other):
        return Point(self.x + other.x, self.y + other.y)
    
    def __sub__(self, other):
        return Point(self.x - other.x, self.y - other.y)
    
    def angle_deg(self):
        return math.atan2(self.y, self.x) * 180.0 / math.pi
    
    def distance(self):
        return (self.x * self.x + self.y * self.y)**0.5


def detect_cross_pattern(line_values, threshold=400):
    """
    Detect if robot is crossing a black cross pattern using line sensors.
    
    line_values: array of 5 sensor readings from left to right
    Returns: (cross_detected, lateral_error_m, cross_quality)
    
    The cross consists of vertical (Y-axis) and horizontal (X-axis) black lines
    at the target location. When the robot crosses the center, multiple sensors see black.
    
    Raw sensor values: 0-1023 (0=dark, 1023=light)
    threshold: values below this are considered black
    
    Reference from 3π+ specs:
    - 5 line sensors spaced across robot width
    - Typical sensor spacing: ~13-16mm apart on 3π+
    - Total width coverage: ~52-64mm (about 2.1-2.5 inches)
    """
    # Count how many sensors see black (low value = dark)
    black_sensors = [1 if v < threshold else 0 for v in line_values]
    black_count = sum(black_sensors)
    
    # Cross is detected if multiple sensors see black (indicates intersection)
    if black_count < cross_detect_threshold:
        return False, 0.0, 0
    
    # Calculate center of mass of black sensors to determine position error
    # Sensors indexed 0-4 from left to right, with sensor 2 being center
    if black_count == 0:
        return False, 0.0, 0
    
    center_of_black = sum(i * black_sensors[i] for i in range(5)) / black_count
    center_position = 2.0  # ideal center is at index 2
    
    # Sensor spacing on 3π+: approximately 14.4mm between sensors
    # Total span: ~57.6mm for 5 sensors
    # 3π+ body width is ~32mm, sensors overhang
    sensor_spacing_m = 0.0144  # meters between sensors
    
    # Lateral offset from center in meters
    lateral_error_m = (center_of_black - center_position) * sensor_spacing_m
    
    # Quality metric: how clear the cross detection is
    # Based on how many sensors see black and how dark they are
    cross_quality = black_count
    
    return True, lateral_error_m, cross_quality


def detect_extended_x_axis_line(line_values, threshold=400):
    """
    Detect the extended +X axis line of the cross that extends beyond the circle.
    This line is perpendicular to the robot's forward direction.
    
    Returns: (line_detected, line_position, line_quality)
    line_position: -1 = left of center, 0 = centered, +1 = right of center
    """
    black_sensors = [1 if v < threshold else 0 for v in line_values]
    black_count = sum(black_sensors)
    
    if black_count < 1:
        return False, 0, 0
    
    # For the extended X-axis line, we care about the width and position
    # A single continuous black line across multiple sensors indicates the X-axis
    center_of_black = sum(i * black_sensors[i] for i in range(5)) / black_count
    
    # Determine if line is off to left (-1) or right (+1)
    if center_of_black < 2.0:
        line_position = -1  # line is left of center
    elif center_of_black > 2.0:
        line_position = 1   # line is right of center
    else:
        line_position = 0   # line is centered
    
    return True, line_position, black_count


def estimate_heading_error_from_cross(line_values, center_y_detected, threshold=400):
    """
    Estimate heading error by analyzing the cross pattern.
    
    If the vertical arm (Y-axis) of the cross is detected off-center,
    the robot's heading needs adjustment.
    
    Returns: heading_error_rad (estimated in radians)
    """
    black_sensors = [1 if v < threshold else 0 for v in line_values]
    black_count = sum(black_sensors)
    
    if black_count < 2:
        return 0.0
    
    center_of_black = sum(i * black_sensors[i] for i in range(5)) / black_count
    center_position = 2.0
    
    # If the detected pattern is offset, estimate heading error
    # Each sensor offset (~14.4mm) at ~50mm distance = ~16 degrees rotation needed
    offset_sensors = center_of_black - center_position
    
    # Heading error estimate: offset_sensors * ~0.3 radians per sensor
    # (14.4mm offset / ~75mm effective distance ≈ 0.19 rad per sensor, use 0.3 as conservative)
    heading_error_rad = offset_sensors * 0.3
    
    return heading_error_rad


def correct_position_at_target(bot_odom, target_waypoint, lateral_error_m):
    """
    When a cross is detected, correct the odometry to match the target.
    This acts as an absolute position correction.
    
    bot_odom: odometry object to update
    target_waypoint: target Point where the cross was detected
    lateral_error_m: lateral offset from center line in meters
    """
    # Update position to target waypoint
    bot_odom.botx = target_waypoint.x
    bot_odom.boty = target_waypoint.y
    
    # The cross is aligned with global axes, so heading should be close to 0, 90, 180, or 270 degrees
    # The lateral error from the sensors tells us if we're aligned or slightly off
    # For now, we keep the measured heading as-is since the cross constrains position
    # In future, could refine heading based on which axis the cross was detected on
    pass


def correct_heading_at_target(bot_odom, target_waypoint, heading_error_rad):
    """
    Correct the robot's heading estimate when a target cross is detected.
    Uses the cross pattern alignment to refine heading.
    
    This is called before position correction to ensure we have accurate heading.
    """
    # Apply the measured heading error
    bot_odom.bot_rad = bot_odom.bot_rad + heading_error_rad * 0.1  # Apply 10% correction gradually




waypoints = []
waypoints.append(Point(0.0, 0.0))
waypoints.append(Point(24.0*0.0254, 0.0*0.0254))
waypoints.append(Point(24.0*0.0254, -24.0*0.0254))
waypoints.append(Point(40.0*0.0254, -24.0*0.0254))
waypoints.append(Point(48.0*0.0254, 8.0*0.0254))

waypoint_types = []
waypoint_types.append("CIRCLE")
waypoint_types.append("INTERMEDIATE")
waypoint_types.append("CIRCLE")
waypoint_types.append("INTERMEDIATE")
waypoint_types.append("CIRCLE")

wp_ind = 1
num_wp = len(waypoints)

line = [0, 0, 0, 0, 0]

# Target circle and cross dimensions (in meters)
target_circle_radius = 6 * 0.0254 / 2.0  # 6 inch diameter
cross_arm_length = target_circle_radius  # extends to circle perimeter
cross_line_thickness = 0.75 * 0.0254  # 0.75 inch (actual thickness)
extended_x_arm_length = 2 * 0.0254  # ~2 inches beyond circle

# Line sensor thresholds for detecting black
# Raw sensor values: 0 (dark) to 1023 (light)
# Black cross lines should read significantly lower than white background
line_threshold = 400  # values below this are considered black
cross_detect_threshold = 2  # minimum 2 sensors must see black to detect cross

# Cross detection state
cross_detected = False
cross_detect_time = 0
cross_detect_timeout_ms = 500  # timeout for cross detection state

# Spiral search parameters
spiral_search_active = False
spiral_start_time = 0
spiral_radius_m = 0.0  # starts at 0, increases
spiral_radius_rate = 0.15  # meters per second spiral expansion
spiral_turn_speed = 400  # turn speed for searching
spiral_forward_speed = 500  # forward speed during spiral (must overcome friction)

# Heading alignment parameters
heading_aligned = False
heading_align_threshold_rad = 0.1  # ±0.1 rad (~6 degrees) considered aligned
heading_align_turn_speed = 350  # turn speed for alignment (must overcome friction)

# Initial heading calibration
initial_heading_calibrated = True  # Start as True - robot assumes it faces +X axis at home
initial_heading_estimate = 0.0  # Will be refined if cross detected at home

# Waypoint navigation direction: forward = 1, backward = -1
wp_direction = 1  # Start going forward through waypoints
mission_complete = False

while True:
    #motors.set_speeds(max_speed, max_speed)
    bump_sensors.read()
    now = time.ticks_ms()
    
    if imu.gyro.data_ready() and (now - odom_time) > odom_period_msec:
        imu.gyro.read()
        yaw_rate_deg = imu.gyro.last_reading_dps[2]  # degrees per second
        #print("%.2f, %.2f" % (now, yaw_rate_deg))
        enc = encoders.get_counts()
        odom_time = now
        bot_odom.update_odom(enc[0], enc[1], yaw_rate_deg)
        
        line = line_sensors.read()
        line_sensors.start_read()
        
        # Initialize cross detection variables
        cross_found = False
        lateral_error_m = 0.0
        heading_error = 0.0
        extended_line_found = False
        
        # IMPORTANT: Don't process crosses until we're close to a waypoint
        # This prevents interference with dead reckoning navigation
        # We'll check for crosses later when near_goal is true
    
    # Desired heading toward waypoint
    wp = waypoints[wp_ind]
    bxy = Point(bot_odom.botx, bot_odom.boty)
    wp_diff = wp - bxy
    dist_to_goal = wp_diff.distance()
    
    near_goal = False
    if dist_to_goal < 0.05:  # 5cm threshold to allow cross detection before stopping
        near_goal = True
    
    des_heading = wp_diff.angle_deg()
    yaw_error_deg = bot_odom.bot_rad*180.0 / math.pi - des_heading
    yaw_error_sign = 0.0
    if abs(yaw_error_deg) > 1.0:
        yaw_error_sign = yaw_error_deg / abs(yaw_error_deg)
    
    # Debug output every 2 seconds
    if (now - disp_time) > 2000:
        disp_time = now
        state_names = ["STOP", "TRACK", "REV", "TURN", "SPIRAL", "ALIGN"]
        print("State: %s, wp: %d, dist: %.3f, near: %d, yaw_err: %.1f, L:%d R:%d" % 
              (state_names[state], wp_ind, dist_to_goal, near_goal, yaw_error_deg, left_speed, right_speed))
    left_speed = 0
    right_speed = 0
    
    if state == state_stop:
        motors.set_speeds(0,0)
        time.sleep_ms(200)  # Reduced from 500ms - gives odometry time to update without long pause
        state = next_state
    elif state == state_track:    
        if abs(yaw_error_deg) < 10.0:
            # Heading is good - move forward
            motors.set_speeds(left_nom_speed, right_nom_speed)
            left_speed = left_nom_speed
            right_speed = right_nom_speed
        elif abs(yaw_error_deg) < 30.0:
            # Heading is off but not too much - move forward with correction
            offset_cmd = 100.0 * yaw_error_deg / 30.0
            left_speed = int(left_nom_speed + offset_cmd)
            right_speed = int(right_nom_speed - offset_cmd)
            motors.set_speeds(left_speed, right_speed)
        else:
            # Heading is way off - stop and turn
            motors.set_speeds(0, 0)
            state = state_stop
            next_state = state_turn
    elif state == state_turn:
        # Turn in place until heading is better
        left_speed = int(left_nom_speed * yaw_error_sign)
        right_speed = int(-right_nom_speed * yaw_error_sign)
        motors.set_speeds(left_speed, right_speed)
        if abs(yaw_error_deg) < 15.0:  # Use 15° threshold to exit turn with hysteresis
            motors.set_speeds(0, 0)
            state = state_stop
            next_state = state_track
    elif state == state_spiral:
        # Spiral search: expand outward while turning to find the target circle
        if not spiral_search_active:
            spiral_search_active = True
            spiral_start_time = now
            spiral_radius_m = 0.0
        
        # Calculate time in spiral
        spiral_time_s = (now - spiral_start_time) / 1000.0
        
        # Expand radius and turn
        spiral_radius_m = spiral_time_s * spiral_radius_rate
        
        # Spiral motion: forward + turn
        # Forward speed starts at 50% to overcome friction, ramps to 100% over 3 seconds
        forward_speed = int(spiral_forward_speed * max(0.5, min(1.0, 0.5 + spiral_time_s / 6.0)))
        turn_speed = spiral_turn_speed
        
        left_speed = forward_speed + turn_speed
        right_speed = forward_speed - turn_speed
        motors.set_speeds(left_speed, right_speed)
        
        # Exit spiral if cross is found
        if cross_found:
            motors.set_speeds(0, 0)
            spiral_search_active = False
            state = state_stop
            next_state = state_align_heading
        
        # Exit spiral after 15 seconds (3 meter radius)
        if spiral_time_s > 15.0:
            motors.set_speeds(0, 0)
            spiral_search_active = False
            state = state_stop
            next_state = state_track
    
    elif state == state_align_heading:
        # Slowly turn to align with the cross
        # Use heading error from cross detection to guide rotation
        if abs(heading_error) < heading_align_threshold_rad:
            # Heading is aligned
            motors.set_speeds(0, 0)
            state = state_stop
            next_state = state_track
            correct_heading_at_target(bot_odom, waypoints[wp_ind], heading_error)
            correct_position_at_target(bot_odom, waypoints[wp_ind], lateral_error_m)
            heading_aligned = True
        else:
            # Turn slowly to reduce heading error
            turn_direction = 1.0 if heading_error > 0 else -1.0
            left_speed = int(heading_align_turn_speed * turn_direction)
            right_speed = int(-heading_align_turn_speed * turn_direction)
            motors.set_speeds(left_speed, right_speed)
    
    # Handle arrival at waypoint
    if near_goal and not mission_complete:
        # Determine waypoint type and behavior
        current_wp_type = waypoint_types[wp_ind]
        
        if current_wp_type == "INTERMEDIATE":
            # For intermediate waypoints: trust dead reckoning, don't search for cross
            # Just mark as reached and move to next waypoint
            motors.set_speeds(0, 0)
            for k in range(2):
                buzzer.play("a32")
                time.sleep_ms(100)
            time.sleep_ms(300)
            wp_ind += wp_direction
            
            # Check if we've completed the forward pass or backward pass
            if wp_direction == 1 and wp_ind >= num_wp:
                # Reached end of waypoints, start going backward
                wp_ind = num_wp - 2
                wp_direction = -1
            elif wp_direction == -1 and wp_ind <= 0:
                # Reached home going backward - mission complete
                wp_ind = 0
                mission_complete = True
                motors.set_speeds(0, 0)
        
        elif current_wp_type == "CIRCLE":
            # For circle waypoints: check if line sensor sees black within 20cm range
            # If so, consider waypoint achieved (don't update position, just advance)
            line_sensor_sees_black = any(s < line_threshold for s in line)
            
            if line_sensor_sees_black:
                # Found the black circle - move to next waypoint
                motors.set_speeds(0, 0)
                for k in range(4):
                    buzzer.play("a32")
                    time.sleep_ms(100)
                time.sleep_ms(500)
                wp_ind += wp_direction
                
                # Check if we've completed the forward pass or backward pass
                if wp_direction == 1 and wp_ind >= num_wp:
                    # Reached end of waypoints, start going backward
                    wp_ind = num_wp - 2
                    wp_direction = -1
                elif wp_direction == -1 and wp_ind <= 0:
                    # Reached home going backward - mission complete
                    wp_ind = 0
                    mission_complete = True
                    motors.set_speeds(0, 0)
            else:
                # Circle waypoint but no black detected yet
                # Move forward slowly to find it (stay in movement mode)
                search_forward_speed = 250
                search_turn_speed = 60
                left_speed = search_forward_speed + search_turn_speed
                right_speed = search_forward_speed - search_turn_speed
                motors.set_speeds(left_speed, right_speed)
    

    if False and (bump_sensors.left_is_pressed() or bump_sensors.right_is_pressed()):
        yellow_led.on()
        motors.set_speeds(0, 0)
        buzzer.play("a32")
        display.fill(0)
        display.text("Left", 0, 0)
        display.show()
        time.sleep_ms(200)
        buzzer.play("b32")
        yellow_led.off()

        display.fill(0)
        display.show()
        
        time.sleep_ms(1000)
        state = state_stop
        next_state = state_stop
        rev_start_time = now
        prev_dist = bot_odom.dist
        
    if (now - disp_time) > disp_period_msec:
        disp_time = now
        xcm = int(bot_odom.botx * 100.0)
        ycm = int(bot_odom.boty * 100.0)
        yaw_deg = int(bot_odom.bot_rad*180.0/math.pi)
        display.fill_rect(0, 0, 256, 60, 0)
        display.text("X: "+str(xcm), 0, 0)
        display.text("Y: "+str(ycm), 0, 10)
        display.text("Yaw: "+str(yaw_deg), 0, 30)
        cross_str = "X" if cross_detected else " "
        display.text("w: "+str(int(yaw_rate_deg))+" "+cross_str, 0, 40)
        display.show()
        #print("line", line)
        print("state %d, LSpd %d, RSpd %d, line %s" % (state, left_speed, right_speed, str(line)))
        
