# This example makes the 3pi+ 2040 drive forward until it hits a wall, detect
# the collision with its bumpers, then reverse, turn, and keep driving.

# camera_circle_touch.py - Pololu 3pi+ 2040 navigation with ArUco camera pose correction.
# Pi Zero 2W runs aruco_robot_pose.py and sends POSE messages over UART.
# This file receives those messages via camera_pose_serial.CameraPoseSerial.

from pololu_3pi_2040_robot import robot
from pololu_3pi_2040_robot.extras import editions
import time
import odom
import math
import camera_pose_serial
import machine
import sys

# Test option: disable all camera/UART input handling on the 3pi+ side.
# True  = use camera_pose_serial over UART
# False = run navigation without camera input
CAMERA_INPUT_ENABLED = True

line_sensors = robot.LineSensors()
encoders = robot.Encoders()
motors = robot.Motors()
bot_odom = odom.SpeedControlledOdom(motors)
bot_odom.bot_rad = 0.0 #math.pi/2  # Start with heading aligned to +X axis of the map
bump_sensors = robot.BumpSensors()
buzzer = robot.Buzzer()
display = robot.Display()
yellow_led = robot.YellowLED()

imu = robot.IMU()
imu.reset()
imu.enable_default()

edition = "Hyper"
if edition == "Hyper":
    max_speed = 1200
    cruise_speed_mps = 0.20
    turn_speed_mps = 0.14
    heading_correction_mps = 0.06
    search_forward_mps = 0.08
    search_turn_mps = 0.03
    max_wheel_speed_mps = 0.25
    max_accel_mps2 = 0.35
    speed_measure_window_s = 0.10
    turn_time = 125
    motors.flip_left(True)
    motors.flip_right(True)

bot_odom.configure_speed_controller(
    kp=1700.0,
    ki=450.0,
    max_motor_cmd=max_speed,
    max_wheel_speed_mps=max_wheel_speed_mps,
    max_int_term_cmd=350.0,
    max_accel_mps2=max_accel_mps2,
    speed_measure_window_s=speed_measure_window_s,
)

display.fill(0)
display.show()

bump_sensors.calibrate()

# Camera pose serial receiver: Pi Zero sends POSE messages over UART at 115200 baud.
# uart_id=0 uses the default UART0 pins on the 3pi+. Adjust if wired differently.
if CAMERA_INPUT_ENABLED:
    cam = camera_pose_serial.CameraPoseSerial(uart_id=0, baudrate=115200, stale_ms=500,
                                              tx_pin=machine.Pin(28), rx_pin=machine.Pin(29))
else:
    cam = None

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


def wrap_angle_deg(angle_deg):
    while angle_deg <= -180.0:
        angle_deg += 360.0
    while angle_deg > 180.0:
        angle_deg -= 360.0
    return angle_deg


def apply_camera_pose_correction(bot_odom, target_waypoint, cam_x_m, cam_y_m, cam_yaw_deg,
                                 marker_map_yaw_deg=0.0):
    """
    Correct robot odometry using camera-measured ArUco marker pose in the robot frame.

    The camera reports (cam_x_m, cam_y_m) = position of the marker relative to the robot,
    in the robot frame (x forward, y left).  We know the marker's true position in the map
    is target_waypoint.  So the robot's true map position is:

      robot_map_x = target_waypoint.x - (cam_x_m * cos(bot_rad) - cam_y_m * sin(bot_rad))
      robot_map_y = target_waypoint.y - (cam_x_m * sin(bot_rad) + cam_y_m * cos(bot_rad))

    cam_yaw_deg: marker yaw in robot frame (CCW from robot +x, degrees).
    Because the ArUco tag x-axis is aligned with the map x-axis the tag's
    map-frame yaw is always 0, so:

      marker_map_yaw = bot_rad + cam_yaw_rad = 0  =>  bot_rad = -cam_yaw_rad

    This works without snapping because any measured yaw is valid - the robot
    is started with its heading aligned to the map +X axis, so the initial
    bot_rad = 0.0 is consistent with this assumption.
    """
    max_pose_update_dist_m = 1.25
    max_pose_update_heading_deg = 120.0

    measured_bot_heading_deg = wrap_angle_deg(marker_map_yaw_deg - cam_yaw_deg)
    measured_bot_heading_rad = math.radians(measured_bot_heading_deg)
    cos_h = math.cos(measured_bot_heading_rad)
    sin_h = math.sin(measured_bot_heading_rad)

    measured_bot_x = target_waypoint.x - (cam_x_m * cos_h - cam_y_m * sin_h)
    measured_bot_y = target_waypoint.y - (cam_x_m * sin_h + cam_y_m * cos_h)

    pose_err_x = measured_bot_x - bot_odom.botx
    pose_err_y = measured_bot_y - bot_odom.boty
    pose_err_dist_m = (pose_err_x * pose_err_x + pose_err_y * pose_err_y) ** 0.5

    current_bot_heading_deg = math.degrees(bot_odom.bot_rad)
    heading_err_deg = abs(wrap_angle_deg(current_bot_heading_deg - measured_bot_heading_deg))

    if pose_err_dist_m > max_pose_update_dist_m or heading_err_deg > max_pose_update_heading_deg:
        return {
            "applied": False,
            "measured_x": measured_bot_x,
            "measured_y": measured_bot_y,
            "measured_heading_deg": measured_bot_heading_deg,
            "marker_map_yaw_deg": marker_map_yaw_deg,
            "pose_err_dist_m": pose_err_dist_m,
            "heading_err_deg": heading_err_deg,
        }

    bot_odom.botx = measured_bot_x
    bot_odom.boty = measured_bot_y
    bot_odom.bot_rad = measured_bot_heading_rad
    return {
        "applied": True,
        "measured_x": measured_bot_x,
        "measured_y": measured_bot_y,
        "measured_heading_deg": measured_bot_heading_deg,
        "marker_map_yaw_deg": marker_map_yaw_deg,
        "pose_err_dist_m": pose_err_dist_m,
        "heading_err_deg": heading_err_deg,
    }




waypoints = []
waypoint_types = []
waypoint_ids = []

square_test = False  # If True, use simple square waypoints without camera targets for testing

if square_test:
    square_corners = [
        Point(0.0, 0.0),
        Point(0.0, 0.40),
        Point(0.40, 0.40),
        Point(0.40, 0.0)
    ]
    for corner in square_corners:
        waypoints.append(corner)
        waypoint_types.append("INTERMEDIATE")
        waypoint_ids.append(-1)
else:
    waypoints.append(Point(0.0, 0.0)) # home         0
    waypoints.append(Point(0.36, 0.06)) #intm         1
    waypoints.append(Point(0.90, 0.0)) #circle 1     2
    waypoints.append(Point(0.36, 0.0)) #intm         3
    waypoints.append(Point(0.36, 0.54)) #intm        4
    waypoints.append(Point(0.0, 0.54)) # circle 2    5
    waypoints.append(Point(0.9, 0.54)) #intm         6
    waypoints.append(Point(0.9, 1.26)) #intm         7
    waypoints.append(Point(0.54, 1.26)) #intm        8
    waypoints.append(Point(0.54, 0.9)) #intm         9
    waypoints.append(Point(0.0, 0.9)) #circle        10
    waypoints.append(Point(0.18, 7*0.18)) #intm         11
    waypoints.append(Point(0.0, 9*0.18)) #intm         12
    waypoints.append(Point(0.0, 12*0.18)) #intm         13
    waypoints.append(Point(3*0.18, 11*0.18)) #intm         14
    waypoints.append(Point(5*0.18, 11*0.18)) #intm         15
    waypoints.append(Point(5*0.18, 9*0.18)) #circle         16

    waypoint_types = ["INTERMEDIATE"] * len(waypoints)
    waypoint_types[0] = "CIRCLE"
    waypoint_types[2] = "CIRCLE"
    waypoint_types[5] = "CIRCLE"
    waypoint_types[10] = "CIRCLE"
    waypoint_types[16] = "CIRCLE"

    waypoint_ids = [-1] * len(waypoints)
    waypoint_ids[0] = 0
    waypoint_ids[2] = 1
    waypoint_ids[5] = 2
    waypoint_ids[10] = 3
    waypoint_ids[16] = 4

wp_ind = 1
num_wp = len(waypoints)

circle_dict = {}
for i, wp_type in enumerate(waypoint_types):
    if wp_type == "CIRCLE":
        circle_dict[waypoint_ids[i]] = waypoints[i]
        print("Circle waypoint: id=%d at (%.3f, %.3f)" % (waypoint_ids[i], waypoints[i].x, waypoints[i].y))

# Marker map-frame yaw (deg). Use 0 deg when local tag +x is aligned with map +x.
default_marker_map_yaw_deg = 0.0
marker_map_yaw_deg = {}
for marker_id in circle_dict:
    marker_map_yaw_deg[marker_id] = default_marker_map_yaw_deg

# Vertical wall aruco tags with their map coordinates and headings
wall_tags = [
    {"id": 10, "x": 2*0.18, "y": 4*0.18, "yaw_deg": 0},
    {"id": 11, "x": 5*0.18, "y": 8*0.18, "yaw_deg": 0},
    {"id": 12, "x": 1*0.18, "y": 10.5*0.18, "yaw_deg": 90},
    {"id": 13, "x": 5*0.18, "y": 1*0.18, "yaw_deg": 180},
]

wall_dict = {}
wall_tag_ids = []
for wall_tag in wall_tags:
    tag_id = wall_tag["id"]
    wall_dict[tag_id] = Point(wall_tag["x"], wall_tag["y"])
    wall_tag_ids.append(tag_id)
    marker_map_yaw_deg[tag_id] = wall_tag["yaw_deg"]
    print("Wall tag: id=%d at (%.3f, %.3f) yaw=%.1f deg" % (tag_id, wall_tag["x"], wall_tag["y"], wall_tag["yaw_deg"]))

line = [0, 0, 0, 0, 0]

# Line sensor threshold for black fallback at target
# Raw sensor values: 0 (dark) to 1023 (light)
line_threshold = 400  # values below this are considered black

# Camera-assisted speed reduction zone (to reduce motion blur while acquiring tags)
cam_slow_range_min_m = 0.10
cam_slow_range_max_m = 0.30
cam_slow_half_fov_deg = 20.0
cam_slow_speed_scale = 0.75

# Waypoint navigation direction: forward = 1, backward = -1
wp_direction = 1  # Start going forward through waypoints
last_processed_wp = -1  # Track which waypoint was just processed to avoid double-processing

# Distance and cumulative absolute angle since last camera pose correction.
# Initialized large so the first tag detection is immediately eligible.
distance_since_tag = 999.0
angle_since_tag_deg = 999.0
_prev_odom_dist_tag = bot_odom.dist
_prev_bot_rad_tag = bot_odom.bot_rad

while True:
    #motors.set_speeds(max_speed, max_speed)
    bump_sensors.read()
    now = time.ticks_ms()

    # Poll camera UART receiver every loop iteration (non-blocking)
    # Guard against rare UART/parse exceptions caused by noisy serial bytes.
    cam_pose = None
    if CAMERA_INPUT_ENABLED and cam is not None:
        try:
            cam.update()
        except Exception as exc:
            print("cam.update exception:", type(exc).__name__)
            sys.print_exception(exc)
            try:
                cam._buf = bytearray()  # reset parser buffer and continue loop
            except Exception:
                pass
        cam_pose = cam.get_nearest(fresh_only=True)

    cam_x_m    = cam_pose["x_m"]    if cam_pose else None
    cam_y_m    = cam_pose["y_m"]    if cam_pose else None
    cam_yaw_deg = cam_pose["yaw_deg"] if cam_pose else None
    cam_id     = cam_pose["id"]     if cam_pose else None
    cam_qual   = cam_pose["qual"]   if cam_pose else None

    if cam_x_m is not None:
        print("Camera sees marker %s at x=%.3f m, y=%.3f m, yaw=%.1f deg, qual=%.1f" %
              (str(cam_id), cam_x_m, cam_y_m, cam_yaw_deg, cam_qual))

        # Stage 1: potential detection threshold for circle or wall markers.
        if (cam_qual >= 4) and ((cam_id in circle_dict) or (cam_id in wall_dict)):
            tag_gate_ok = distance_since_tag > 0.3 or angle_since_tag_deg > 90.0
            print("Tag gate: dist_since=%.3f angle_since=%.1f ok=%s" %
                  (distance_since_tag, angle_since_tag_deg, str(tag_gate_ok)))
            if tag_gate_ok:
                # Stop first, then wait briefly for a high-quality sample.
                bot_odom.stop(immediate=True)

                corr_pose = None
                corr_marker_id = None
                wait_start_ms = time.ticks_ms()
                time.sleep_ms(100)  # brief initial pause to allow for new samples after stop
                while time.ticks_diff(time.ticks_ms(), wait_start_ms) < 1000:
                    if CAMERA_INPUT_ENABLED and cam is not None:
                        try:
                            cam.update()
                        except Exception as exc:
                            print("cam.update exception (wait):", type(exc).__name__)
                            sys.print_exception(exc)
                            try:
                                cam._buf = bytearray()
                            except Exception:
                                pass
                        wait_pose = cam.get_nearest(fresh_only=True)
                        if (wait_pose is not None
                                and ((wait_pose["id"] in circle_dict) or (wait_pose["id"] in wall_dict))
                                and wait_pose["qual"] >= 4):
                            corr_pose = wait_pose
                            corr_marker_id = wait_pose["id"]
                            break
                    time.sleep_ms(20)

                if corr_pose is not None:
                    cur_heading_deg = wrap_angle_deg(math.degrees(bot_odom.bot_rad))
                    print("Cam corr current: x=%.3f y=%.3f hdg=%.1f" %
                          (bot_odom.botx, bot_odom.boty, cur_heading_deg))

                    marker_yaw_deg = marker_map_yaw_deg.get(corr_marker_id, 0.0)
                    
                    # Use circle_dict if available, otherwise wall_dict
                    target_waypoint = circle_dict.get(corr_marker_id) or wall_dict.get(corr_marker_id)
                    
                    corr = apply_camera_pose_correction(
                        bot_odom, target_waypoint,
                        corr_pose["x_m"], corr_pose["y_m"], corr_pose["yaw_deg"],
                        marker_yaw_deg
                    )

                    print("Cam corr measured: x=%.3f y=%.3f hdg=%.1f (tag_yaw=%.1f), err_d=%.3f err_h=%.1f, applied=%s" %
                          (corr["measured_x"], corr["measured_y"], corr["measured_heading_deg"],
                           corr["marker_map_yaw_deg"],
                           corr["pose_err_dist_m"], corr["heading_err_deg"], str(corr["applied"])))

                    # Reset gating counters after correction attempt.
                    distance_since_tag = 0.0
                    angle_since_tag_deg = 0.0
                    _prev_odom_dist_tag = bot_odom.dist
                    _prev_bot_rad_tag = bot_odom.bot_rad
                else:
                    print("Tag wait timeout: no qual>=4 sample for any known marker within 1.0 s")

    if (now - odom_time) > odom_period_msec:
        if imu.gyro.data_ready():
            imu.gyro.read()
            yaw_rate_deg = imu.gyro.last_reading_dps[2]  # degrees per second
        dt_s = time.ticks_diff(now, odom_time) / 1000.0
        enc = encoders.get_counts()
        odom_time = now
        bot_odom.update_odom_and_control(enc[0], enc[1], dt_s)

        # Update distance and cumulative absolute angle since last tag correction.
        delta_dist = bot_odom.dist - _prev_odom_dist_tag
        _prev_odom_dist_tag = bot_odom.dist
        distance_since_tag += delta_dist
        delta_angle_deg = abs(wrap_angle_deg(math.degrees(bot_odom.bot_rad) - math.degrees(_prev_bot_rad_tag)))
        _prev_bot_rad_tag = bot_odom.bot_rad
        angle_since_tag_deg += delta_angle_deg

        line = line_sensors.read()
        line_sensors.start_read()
    
    # Desired heading toward waypoint
    wp = waypoints[wp_ind]
    bxy = Point(bot_odom.botx, bot_odom.boty)
    wp_diff = wp - bxy
    dist_to_goal = wp_diff.distance()
    
    near_goal = dist_to_goal < 0.05
    
    des_heading = wp_diff.angle_deg()
    bot_heading_deg = bot_odom.bot_rad * 180.0 / math.pi
    yaw_error_deg = wrap_angle_deg(bot_heading_deg - des_heading)
    yaw_error_sign = 0.0
    if abs(yaw_error_deg) > 1.0:
        yaw_error_sign = yaw_error_deg / abs(yaw_error_deg)

    # Predict camera-relative target pose from odometry so slowdown can happen
    # BEFORE the camera has a stable detection.
    pred_cam_x_m = None
    pred_cam_y_m = None
    pred_cam_range_m = None
    pred_cam_bearing_deg = None
    pred_cam_in_slow_zone = False

    current_wp_type = waypoint_types[wp_ind]
    if current_wp_type == "CIRCLE":
        target_wp = waypoints[wp_ind]
        dx_map = target_wp.x - bot_odom.botx
        dy_map = target_wp.y - bot_odom.boty
        cos_h = math.cos(bot_odom.bot_rad)
        sin_h = math.sin(bot_odom.bot_rad)

        # Map-frame delta -> robot-frame predicted camera coordinates.
        pred_cam_x_m = dx_map * cos_h + dy_map * sin_h
        pred_cam_y_m = -dx_map * sin_h + dy_map * cos_h
        pred_cam_range_m = (pred_cam_x_m * pred_cam_x_m + pred_cam_y_m * pred_cam_y_m) ** 0.5
        pred_cam_bearing_deg = math.degrees(math.atan2(pred_cam_y_m, pred_cam_x_m))

        pred_cam_in_slow_zone = (
            pred_cam_x_m > 0.0
            and cam_slow_range_min_m <= pred_cam_range_m <= cam_slow_range_max_m
            and abs(pred_cam_bearing_deg) <= cam_slow_half_fov_deg
        )

    track_speed_scale = cam_slow_speed_scale if pred_cam_in_slow_zone else 1.0
    left_track_mps = cruise_speed_mps * track_speed_scale
    right_track_mps = cruise_speed_mps * track_speed_scale

    left_target_mps = 0.0
    right_target_mps = 0.0
    
    # Debug output every 2 seconds
    if (now - disp_time) > 2000:
        disp_time = now
        state_names = {
            state_stop: "STOP",
            state_track: "TRACK",
            state_rev: "REV",
            state_turn: "TURN",
            state_seek: "SEEK",
        }
        print("State: %s, wp: %d, dist: %.3f, near: %d, yaw_err: %.1f, tgtL:%.2f tgtR:%.2f, cmdL:%d cmdR:%d" %
              (state_names.get(state, "?"), wp_ind, dist_to_goal, near_goal, yaw_error_deg,
               left_target_mps, right_target_mps, bot_odom.last_left_cmd, bot_odom.last_right_cmd))
    
    if state == state_stop:
        bot_odom.stop(immediate=True)
        state = next_state
    elif state == state_track:    
        if abs(yaw_error_deg) < 5.0:
            # Heading is good - move forward
            left_target_mps = left_track_mps
            right_target_mps = right_track_mps
        elif abs(yaw_error_deg) < 20.0:
            # Heading is off but not too much - move forward with correction
            offset_mps = heading_correction_mps * yaw_error_deg / 20.0
            left_target_mps = left_track_mps + offset_mps
            right_target_mps = right_track_mps - offset_mps
        else:
            # Heading is way off - stop and turn
            bot_odom.set_target_speeds_mps(0.0, 0.0)
            state = state_stop
            next_state = state_turn
    elif state == state_turn:
        # Turn in place until heading is better
        left_target_mps = turn_speed_mps * yaw_error_sign
        right_target_mps = -turn_speed_mps * yaw_error_sign
        if abs(yaw_error_deg) < 5.0:  # Use 5 deg threshold to exit turn with hysteresis
            bot_odom.set_target_speeds_mps(0.0, 0.0)
            state = state_stop
            next_state = state_track
    
    # Handle arrival at waypoint
    if near_goal and wp_ind != last_processed_wp:
        # Determine waypoint type and behavior
        current_wp_type = waypoint_types[wp_ind]
        
        if current_wp_type == "INTERMEDIATE":
            # For intermediate waypoints: trust dead reckoning, don't search for cross
            # Just mark as reached and move to next waypoint
            bot_odom.stop(immediate=True)
            left_target_mps = 0.0
            right_target_mps = 0.0
            for k in range(2):
                buzzer.play("a32")
                time.sleep_ms(100)
            time.sleep_ms(300)
            last_processed_wp = wp_ind  # Mark this waypoint as processed
            wp_ind += wp_direction
            
            # Check if we've completed the forward pass or backward pass
            if wp_direction == 1 and wp_ind >= num_wp:
                # Reached end of waypoints, start going backward
                wp_ind = num_wp - 2
                wp_direction = -1
            elif wp_direction == -1 and wp_ind < 0:  # Changed from <= to < to allow wp_ind=0 to be processed
                # Reached home going backward - now start forward again
                wp_ind = 1
                wp_direction = 1

            state = state_stop
            next_state = state_track
        elif current_wp_type == "CIRCLE":
            # For circle waypoints: use camera ArUco detection to determine arrival.
            # The camera reports the marker (x_r, y_r) in the robot frame.
            # Criteria for "at target":
            #   - Camera has a fresh detection  AND
            #   - Marker is within cam_arrive_x_m forward of robot  AND
            #   - Marker lateral offset |y_r| < cam_arrive_y_m
            # When arrived, apply camera pose correction to the odometry before advancing.
            cam_arrive_x_m = 0.20   # marker must be within 20 cm forward
            cam_arrive_y_m = 0.12   # marker must be within 12 cm lateral

            cam_at_target = (cam_x_m is not None
                             and 0.0 <= cam_x_m < cam_arrive_x_m
                             and abs(cam_y_m) < cam_arrive_y_m)

            # Fallback: also accept line sensor if camera is unavailable
            line_sensor_fallback = (cam_x_m is None
                                    and any(s < line_threshold for s in line))

            if cam_at_target or line_sensor_fallback:
                bot_odom.stop(immediate=True)
                left_target_mps = 0.0
                right_target_mps = 0.0
                for k in range(4):
                    buzzer.play("a32")
                    time.sleep_ms(100)
                time.sleep_ms(500)
                last_processed_wp = wp_ind
                wp_ind += wp_direction

                if wp_direction == 1 and wp_ind >= num_wp:
                    wp_ind = num_wp - 2
                    wp_direction = -1
                elif wp_direction == -1 and wp_ind < 0:
                    wp_ind = 1
                    wp_direction = 1

                state = state_stop
                next_state = state_track
            else:
                # No camera detection near goal - slow forward search
                left_target_mps = search_forward_mps + search_turn_mps
                right_target_mps = search_forward_mps - search_turn_mps

    bot_odom.set_target_speeds_mps(left_target_mps, right_target_mps)
    

    if False and (bump_sensors.left_is_pressed() or bump_sensors.right_is_pressed()):
        yellow_led.on()
        bot_odom.stop()
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
        cam_str = "C" if (cam_x_m is not None) else " "
        pi_alive_now = (cam.pi_alive() if (CAMERA_INPUT_ENABLED and cam is not None) else False)
        pi_str = "P" if pi_alive_now else " "
        display.text("w: "+str(int(yaw_rate_deg))+" "+cam_str+pi_str, 0, 40)
        display.show()
        print("X: %.2f m, Y: %.2f m, Yaw: %.1f deg, w: %.1f deg/s, wp: %d/%d" %
              (bot_odom.botx, bot_odom.boty, bot_odom.bot_rad*180.0/math.pi,
               yaw_rate_deg, wp_ind, num_wp))
        # if cam_x_m is not None:
        #     print("state %d, LSpd %d, RSpd %d, cam id=%s x=%.3f y=%.3f yaw=%.1f" %
        #           (state, left_speed, right_speed, str(cam_id),
        #            cam_x_m, cam_y_m, cam_yaw_deg))
        # else:
        #     print("state %d, LSpd %d, RSpd %d, no cam, pi=%s" %
        #           (state, left_speed, right_speed, str(pi_alive_now)))
        
