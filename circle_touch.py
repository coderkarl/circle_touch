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

waypoints = []
waypoints.append(Point(0.0, 0.0))
waypoints.append(Point(0.3, 0.0))
waypoints.append(Point(0.3, 0.6))

wp_ind = 1
num_wp = len(waypoints)

line = [0, 0, 0, 0, 0]

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
        display.text("w: " + str(int(yaw_rate_deg)), 0, 40)
        display.show()
        #print("line", line)
        print("state %d, LSpd %d, RSpd %d" % (state, left_speed, right_speed))
        
