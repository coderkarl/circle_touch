#!/usr/bin/env python3

import math

import time

class Odom():
    def __init__(self):
        self.prev_msec = time.ticks_ms()

        self.bot_rad = 0
        self.botx = 0.0
        self.boty = 0.0
        self.dist = 0.0
        
        self.counts_per_meter = -1706.0
        self.track_width = 3.4 * 0.0254  # 3.4 inches wheel-to-wheel distance, converted to meters
        self.prev_enc_left = 0
        self.prev_enc_right = 0

        self.gyro_factor = 0.95
        self.yaw_rate_bias_dps = 0.0
        self.stopped_yaw_rate_sum_deg = 0.0
        self.stopped_sample_count = 0
        self.stopped_time_s = 0.0
        self.min_bias_update_stop_s = 2.0
        self.yaw_rate_bias_weight_s = 2.0

    def _compute_dt_s(self):
        now_msec = time.ticks_ms()
        dt_s = time.ticks_diff(now_msec, self.prev_msec) / 1000.0
        self.prev_msec = now_msec
        if dt_s <= 0.0:
            dt_s = 1e-3
        return dt_s

    def _reset_stopped_yaw_accumulator(self):
        self.stopped_yaw_rate_sum_deg = 0.0
        self.stopped_sample_count = 0
        self.stopped_time_s = 0.0

    def _accumulate_stopped_yaw_rate(self, yaw_rate_deg, dt_s):
        self.stopped_yaw_rate_sum_deg += yaw_rate_deg
        self.stopped_sample_count += 1
        self.stopped_time_s += dt_s

    def _update_yaw_rate_bias_from_stop(self):
        if self.stopped_sample_count <= 0:
            self._reset_stopped_yaw_accumulator()
            return

        if self.stopped_time_s > self.min_bias_update_stop_s:
            stopped_avg_yaw_rate_dps = self.stopped_yaw_rate_sum_deg / self.stopped_sample_count
            old_weight = self.yaw_rate_bias_weight_s
            new_weight = self.stopped_time_s
            total_weight = old_weight + new_weight
            if total_weight <= 0.0:
                self.yaw_rate_bias_dps = stopped_avg_yaw_rate_dps
            else:
                self.yaw_rate_bias_dps = (
                    (self.yaw_rate_bias_dps * old_weight) + (stopped_avg_yaw_rate_dps * new_weight)
                ) / total_weight

        self._reset_stopped_yaw_accumulator()

    def _update_pose_from_counts_and_yaw(self, dleft, dright, yaw_rate_deg, dt_s):
        dmeters = (dleft + dright) / 2.0 / self.counts_per_meter
        self.dist += dmeters

        dleft_m = dleft / self.counts_per_meter
        dright_m = dright / self.counts_per_meter
        dtheta_rad_enc = (dright_m - dleft_m) / self.track_width

        is_stopped = (dleft == 0 and dright == 0)
        if is_stopped:
            self._accumulate_stopped_yaw_rate(yaw_rate_deg, dt_s)
            dtheta_rad = 0.0
        else:
            self._update_yaw_rate_bias_from_stop()
            dtheta_rad_gyro = math.radians(yaw_rate_deg*self.gyro_factor - self.yaw_rate_bias_dps) * dt_s
            dtheta_rad = dtheta_rad_gyro*0.5 + dtheta_rad_enc*0.5

            # dtheta_ref = abs(dtheta_rad_enc)
            # if dtheta_ref < 1e-6:
            #     dtheta_ref = 1e-6
            # if abs(dtheta_rad - dtheta_rad_enc) > (0.2 * dtheta_ref):
            #     dtheta_rad = dtheta_rad_enc

        self.bot_rad = self.bot_rad + dtheta_rad

        dx = dmeters * math.cos(self.bot_rad)
        dy = dmeters * math.sin(self.bot_rad)
        self.botx = self.botx + dx
        self.boty = self.boty + dy
    
    def update_odom(self, enc_left, enc_right, yaw_rate_deg):
        """
        Update odometry using encoder distance and gyro yaw rate.
        
        For differential drive:
        - Distance: dmeters = (dLeft + dRight) / 2 / counts_per_meter
        - Heading: dTheta = (yaw_rate_deg - yaw_rate_bias_dps) * dt
        - Position: integrate distance at current heading
        """
        dt_s = self._compute_dt_s()
        
        dleft = enc_left - self.prev_enc_left
        dright = enc_right - self.prev_enc_right
        self.prev_enc_left = enc_left
        self.prev_enc_right = enc_right

        self._update_pose_from_counts_and_yaw(dleft, dright, yaw_rate_deg, dt_s)


class SpeedControlledOdom(Odom):
    def __init__(self, motors=None):
        super().__init__()
        self.motors = motors

        self.target_left_mps = 0.0
        self.target_right_mps = 0.0

        self.last_left_mps = 0.0
        self.last_right_mps = 0.0
        self.last_left_raw_mps = 0.0
        self.last_right_raw_mps = 0.0
        self.cmd_left_mps = 0.0
        self.cmd_right_mps = 0.0
        self.last_left_cmd = 0
        self.last_right_cmd = 0

        self.max_motor_cmd = 1200
        self.max_wheel_speed_mps = 0.20

        self.kp = 1700.0
        self.ki = 450.0
        self.max_accel_mps2 = 0.35
        self.speed_measure_window_s = 0.10

        self.int_left = 0.0
        self.int_right = 0.0
        self.max_int_term_cmd = 350.0

        self._speed_dt_hist = []
        self._speed_left_hist = []
        self._speed_right_hist = []
        self._speed_dt_sum = 0.0
        self._speed_left_sum = 0
        self._speed_right_sum = 0

    def set_motor_interface(self, motors):
        self.motors = motors

    def configure_speed_controller(self, kp=None, ki=None,
                                   max_motor_cmd=None, max_wheel_speed_mps=None,
                                   max_int_term_cmd=None,
                                   max_accel_mps2=None,
                                   speed_measure_window_s=None):
        if kp is not None:
            self.kp = float(kp)
        if ki is not None:
            self.ki = float(ki)
        if max_motor_cmd is not None:
            self.max_motor_cmd = int(max_motor_cmd)
        if max_wheel_speed_mps is not None:
            self.max_wheel_speed_mps = float(max_wheel_speed_mps)
        if max_int_term_cmd is not None:
            self.max_int_term_cmd = float(max_int_term_cmd)
        if max_accel_mps2 is not None:
            self.max_accel_mps2 = float(max_accel_mps2)
        if speed_measure_window_s is not None:
            self.speed_measure_window_s = float(speed_measure_window_s)
            self._reset_speed_measurement_window()

    def set_target_speeds_mps(self, left_mps, right_mps):
        self.target_left_mps = self._clamp(float(left_mps), -self.max_wheel_speed_mps, self.max_wheel_speed_mps)
        self.target_right_mps = self._clamp(float(right_mps), -self.max_wheel_speed_mps, self.max_wheel_speed_mps)

    def stop(self, immediate=False):
        self.target_left_mps = 0.0
        self.target_right_mps = 0.0
        self.int_left = 0.0
        self.int_right = 0.0
        if immediate:
            self.cmd_left_mps = 0.0
            self.cmd_right_mps = 0.0
            self.last_left_mps = 0.0
            self.last_right_mps = 0.0
            self.last_left_raw_mps = 0.0
            self.last_right_raw_mps = 0.0
            self.last_left_cmd = 0
            self.last_right_cmd = 0
            self._reset_speed_measurement_window()
        if immediate and self.motors is not None:
            self.motors.set_speeds(0, 0)

    def _clamp(self, value, low, high):
        if value < low:
            return low
        if value > high:
            return high
        return value

    def _feedforward_cmd(self, target_mps):
        if self.max_wheel_speed_mps <= 0.0:
            return 0.0
        return (target_mps / self.max_wheel_speed_mps) * self.max_motor_cmd

    def _reset_speed_measurement_window(self):
        self._speed_dt_hist = []
        self._speed_left_hist = []
        self._speed_right_hist = []
        self._speed_dt_sum = 0.0
        self._speed_left_sum = 0
        self._speed_right_sum = 0

    def _update_speed_measurement(self, dleft, dright, dt_s):
        self.last_left_raw_mps = (dleft / self.counts_per_meter) / dt_s
        self.last_right_raw_mps = (dright / self.counts_per_meter) / dt_s

        if self.speed_measure_window_s <= dt_s:
            self.last_left_mps = self.last_left_raw_mps
            self.last_right_mps = self.last_right_raw_mps
            return

        self._speed_dt_hist.append(dt_s)
        self._speed_left_hist.append(dleft)
        self._speed_right_hist.append(dright)
        self._speed_dt_sum += dt_s
        self._speed_left_sum += dleft
        self._speed_right_sum += dright

        while len(self._speed_dt_hist) > 1 and (self._speed_dt_sum - self._speed_dt_hist[0]) >= self.speed_measure_window_s:
            self._speed_dt_sum -= self._speed_dt_hist.pop(0)
            self._speed_left_sum -= self._speed_left_hist.pop(0)
            self._speed_right_sum -= self._speed_right_hist.pop(0)

        if self._speed_dt_sum <= 0.0:
            self.last_left_mps = self.last_left_raw_mps
            self.last_right_mps = self.last_right_raw_mps
            return

        self.last_left_mps = (self._speed_left_sum / self.counts_per_meter) / self._speed_dt_sum
        self.last_right_mps = (self._speed_right_sum / self.counts_per_meter) / self._speed_dt_sum

    def _slew_to_target(self, current_value, target_value, max_rate, dt_s):
        if max_rate <= 0.0:
            return target_value
        max_step = max_rate * dt_s
        delta = target_value - current_value
        if delta > max_step:
            return current_value + max_step
        if delta < -max_step:
            return current_value - max_step
        return target_value

    def _wheel_cmd_from_speed(self, target_mps, meas_mps, int_state, dt_s):
        err = target_mps - meas_mps

        if self.ki > 0.0:
            int_state += err * dt_s
            max_int_state = self.max_int_term_cmd / self.ki
            int_state = self._clamp(int_state, -max_int_state, max_int_state)

        p_term = self.kp * err
        i_term = self.ki * int_state
        ff_term = self._feedforward_cmd(target_mps)
        raw_cmd = ff_term + p_term + i_term

        cmd = int(self._clamp(raw_cmd, -self.max_motor_cmd, self.max_motor_cmd))
        return cmd, int_state

    def update(self, enc_left, enc_right, dt_s):
        if dt_s <= 0.0:
            dt_s = 1e-3

        dleft = enc_left - self.prev_enc_left
        dright = enc_right - self.prev_enc_right
        self.prev_enc_left = enc_left
        self.prev_enc_right = enc_right

        dleft_m = dleft / self.counts_per_meter
        dright_m = dright / self.counts_per_meter

        dmeters = (dleft_m + dright_m) * 0.5
        self.dist += dmeters

        dtheta_rad = (dright_m - dleft_m) / self.track_width
        self.bot_rad = self.bot_rad + dtheta_rad

        dx = dmeters * math.cos(self.bot_rad)
        dy = dmeters * math.sin(self.bot_rad)
        self.botx = self.botx + dx
        self.boty = self.boty + dy

        self._update_speed_measurement(dleft, dright, dt_s)

        self.cmd_left_mps = self._slew_to_target(
            self.cmd_left_mps, self.target_left_mps, self.max_accel_mps2, dt_s
        )
        self.cmd_right_mps = self._slew_to_target(
            self.cmd_right_mps, self.target_right_mps, self.max_accel_mps2, dt_s
        )

        left_cmd, self.int_left = self._wheel_cmd_from_speed(
            self.cmd_left_mps, self.last_left_mps, self.int_left, dt_s
        )
        right_cmd, self.int_right = self._wheel_cmd_from_speed(
            self.cmd_right_mps, self.last_right_mps, self.int_right, dt_s
        )

        self.last_left_cmd = left_cmd
        self.last_right_cmd = right_cmd

        if self.motors is not None:
            self.motors.set_speeds(left_cmd, right_cmd)

    def update_odom(self, enc_left, enc_right, yaw_rate_deg):
        dt_s = self._compute_dt_s()
        dleft = enc_left - self.prev_enc_left
        dright = enc_right - self.prev_enc_right
        self.prev_enc_left = enc_left
        self.prev_enc_right = enc_right

        self._update_pose_from_counts_and_yaw(dleft, dright, yaw_rate_deg, dt_s)

    def update_odom_and_control(self, enc_left, enc_right, dt_s):
        self.update(enc_left, enc_right, dt_s)
