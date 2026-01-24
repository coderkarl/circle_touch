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




waypoints = []
waypoints.append(Point(0.0, 0.0))
waypoints.append(Point(21.5*0.0254, -21.5*0.0254))
waypoints.append(Point(39.5*0.0254, -1.0*0.0254))

wp_ind = 1
num_wp = len(waypoints)

line = [0, 0, 0, 0, 0]

# Target circle and cross dimensions (in meters)
target_circle_radius = 6 * 0.0254 / 2.0  # 6 inch diameter
cross_arm_length = target_circle_radius  # extends to circle perimeter
cross_line_thickness = 1 * 0.0254  # 1 inch

# Line sensor thresholds for detecting black
# Raw sensor values: 0 (dark) to 1023 (light)
# Black cross lines should read significantly lower than white background
line_threshold = 400  # values below this are considered black
cross_detect_threshold = 2  # minimum 2 sensors must see black to detect cross

# Cross detection state
cross_detected = False
cross_detect_time = 0
cross_detect_timeout_ms = 500  # timeout for cross detection state

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
        
        # Detect cross pattern and correct position if found
        cross_found, lateral_error_m, quality = detect_cross_pattern(line, threshold=line_threshold)
        if cross_found:
            # Only correct position near the target waypoint
            wp = waypoints[wp_ind]
            bxy = Point(bot_odom.botx, bot_odom.boty)
            wp_diff = wp - bxy
            dist_to_target = wp_diff.distance()
            
            # If we're reasonably close to a target, use the cross to correct position
            if dist_to_target < 0.1:  # within 10cm of target
                # Cross detected at target - correct the odometry
                correct_position_at_target(bot_odom, wp, lateral_error_m)
                cross_detected = True
                cross_detect_time = now
    
    # Desired heading toward waypoint
    wp = waypoints[wp_ind]
    bxy = Point(bot_odom.botx, bot_odom.boty)
    wp_diff = wp - bxy
    dist_to_goal = wp_diff.distance()
    
    near_goal = False
    if dist_to_goal < 0.03:
        near_goal = True
    
    des_heading = wp_diff.angle_deg()
    yaw_error_deg = bot_odom.bot_rad*180.0 / math.pi - des_heading
    yaw_error_sign = 0.0
    if abs(yaw_error_deg) > 1.0:
        yaw_error_sign = yaw_error_deg / abs(yaw_error_deg)
    
    left_speed = 0
    right_speed = 0
    
    if state == state_stop:
        motors.set_speeds(0,0)
        time.sleep_ms(500)
        state = next_state
    elif state == state_track:    
        if abs(yaw_error_deg) < 10.0:
            motors.set_speeds(left_nom_speed, right_nom_speed)
            left_speed = left_nom_speed
            right_speed = right_nom_speed
        elif abs(yaw_error_deg) < 30.0:
            offset_cmd = 100.0 * yaw_error_deg / 30.0
            left_speed = left_nom_speed + offset_cmd
            right_speed = right_nom_speed - offset_cmd
            motors.set_speeds(left_speed, right_speed)
        else:
            motors.set_speeds(0,0)
            state = state_stop
            next_state = state_turn
    elif state == state_turn:
        left_speed = left_nom_speed * yaw_error_sign
        right_speed = -right_nom_speed * yaw_error_sign
        motors.set_speeds(left_speed, right_speed)
        if abs(yaw_error_deg) < 10.0:
            state = state_stop
            next_state = state_track
    
    if near_goal and line[1] > 600 and line[2] > 600 and line[3] > 600:
        motors.set_speeds(0, 0)
        for k in range(4):
            buzzer.play("a32")
            time.sleep_ms(100)
        time.sleep_ms(500)
        wp_ind += 1
        if wp_ind >= num_wp:
            wp_ind = 0
        state = state_stop
        next_state = state_track
        cross_detected = False
    
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
        
