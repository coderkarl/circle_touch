#!/usr/bin/env python3

"""
Interactive speed-control test for Pololu 3pi+ 2040.

Usage (via REPL / serial console):
- Run this script on the robot.
- Enter commands when prompted.

Command formats:
  <left_mps> <right_mps> [duration_s]
    step <left_mps> <right_mps> [segment_s]
    stop

Examples:
  0.10 0.10 3
  0.15 -0.15 2.5
  0 0
    step 0.12 0.12 2
    step 0.15 -0.15 1.5
    stop

Type `q` to quit.
"""

from pololu_3pi_2040_robot import robot
import time
import odom


def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def run_sequence_test(controller, encoders, sequence,
                      control_period_ms=20, print_period_ms=100):
    start_ms = time.ticks_ms()
    next_control_ms = start_ms
    next_print_ms = start_ms
    prev_update_ms = start_ms

    total_duration_ms = 0
    prepared_sequence = []
    for segment_name, left_target_mps, right_target_mps, duration_s in sequence:
        left_target_mps = clamp(left_target_mps, -controller.max_wheel_speed_mps, controller.max_wheel_speed_mps)
        right_target_mps = clamp(right_target_mps, -controller.max_wheel_speed_mps, controller.max_wheel_speed_mps)
        duration_ms = int(duration_s * 1000)
        prepared_sequence.append((segment_name, left_target_mps, right_target_mps, duration_ms))
        total_duration_ms += duration_ms

    segment_index = 0
    segment_start_ms = start_ms
    segment_name, left_target_mps, right_target_mps, segment_duration_ms = prepared_sequence[0]
    controller.set_target_speeds_mps(left_target_mps, right_target_mps)

    print("--- Running sequence for %.2f s ---" % (total_duration_ms / 1000.0))
    print("t_s,segment,target_l,target_r,ramped_l,ramped_r,cmd_l,cmd_r,raw_l,raw_r,avg_l,avg_r")

    while time.ticks_diff(time.ticks_ms(), start_ms) < total_duration_ms:
        now_ms = time.ticks_ms()

        if time.ticks_diff(now_ms, segment_start_ms) >= segment_duration_ms:
            segment_index += 1
            if segment_index >= len(prepared_sequence):
                break
            segment_start_ms = now_ms
            segment_name, left_target_mps, right_target_mps, segment_duration_ms = prepared_sequence[segment_index]
            controller.set_target_speeds_mps(left_target_mps, right_target_mps)

        if time.ticks_diff(now_ms, next_control_ms) >= 0:
            dt_s = time.ticks_diff(now_ms, prev_update_ms) / 1000.0
            if dt_s <= 0.0:
                dt_s = control_period_ms / 1000.0

            prev_update_ms = now_ms
            enc = encoders.get_counts()
            controller.update_odom_and_control(enc[0], enc[1], dt_s)
            next_control_ms = time.ticks_add(next_control_ms, control_period_ms)

        if time.ticks_diff(now_ms, next_print_ms) >= 0:
            elapsed_s = time.ticks_diff(now_ms, start_ms) / 1000.0
            print("%.3f,%s,%.3f,%.3f,%.3f,%.3f,%d,%d,%.3f,%.3f,%.3f,%.3f" % (
                elapsed_s,
                segment_name,
                controller.target_left_mps,
                controller.target_right_mps,
                controller.cmd_left_mps,
                controller.cmd_right_mps,
                controller.last_left_cmd,
                controller.last_right_cmd,
                controller.last_left_raw_mps,
                controller.last_right_raw_mps,
                controller.last_left_mps,
                controller.last_right_mps,
            ))
            next_print_ms = time.ticks_add(next_print_ms, print_period_ms)

        time.sleep_ms(1)


def run_response_test(controller, encoders, left_target_mps, right_target_mps,
                      duration_s=3.0, control_period_ms=20, print_period_ms=100):
    return_s = 1.0
    print("--- Running manual response test for %.2f s + %.2f s return ---" % (duration_s, return_s))
    print("target L=%.3f m/s, R=%.3f m/s" % (left_target_mps, right_target_mps))
    run_sequence_test(
        controller,
        encoders,
        [
            ("manual", left_target_mps, right_target_mps, duration_s),
            ("return", 0.0, 0.0, return_s),
        ],
        control_period_ms=control_period_ms,
        print_period_ms=print_period_ms,
    )


def run_immediate_stop(controller):
    controller.stop(immediate=True)
    print("Immediate stop applied.")


def run_step_test(controller, encoders, left_target_mps, right_target_mps,
                  segment_s=2.0, baseline_s=1.0,
                  control_period_ms=20, print_period_ms=100):
    print("--- Running step test ---")
    print("baseline %.2f s -> step %.2f s -> return %.2f s" % (baseline_s, segment_s, segment_s))
    print("step target L=%.3f m/s, R=%.3f m/s" % (left_target_mps, right_target_mps))
    run_sequence_test(
        controller,
        encoders,
        [
            ("baseline", 0.0, 0.0, baseline_s),
            ("step", left_target_mps, right_target_mps, segment_s),
            ("return", 0.0, 0.0, segment_s),
        ],
        control_period_ms=control_period_ms,
        print_period_ms=print_period_ms,
    )


def parse_command(cmd):
    parts = cmd.strip().split()
    if not parts:
        return None

    if parts[0].lower() == "stop":
        if len(parts) != 1:
            return None
        return ("stop", 0.0, 0.0, 0.0)

    if parts[0].lower() == "step":
        if len(parts) < 3 or len(parts) > 4:
            return None
        left_mps = float(parts[1])
        right_mps = float(parts[2])
        segment_s = 2.0
        if len(parts) == 4:
            segment_s = float(parts[3])
        return ("step", left_mps, right_mps, segment_s)

    if len(parts) < 2 or len(parts) > 3:
        return None

    left_mps = float(parts[0])
    right_mps = float(parts[1])
    duration_s = 3.0
    if len(parts) == 3:
        duration_s = float(parts[2])

    return ("manual", left_mps, right_mps, duration_s)


def main():
    motors = robot.Motors()
    encoders = robot.Encoders()

    motors.flip_left(True)
    motors.flip_right(True)

    controller = odom.SpeedControlledOdom(motors)
    controller.configure_speed_controller(
        kp=1700.0,
        ki=450.0,
        max_motor_cmd=1200,
        max_wheel_speed_mps=0.25,
        max_int_term_cmd=350.0,
        max_accel_mps2=0.35,
        speed_measure_window_s=0.10,
    )

    print("Speed-control interactive test ready.")
    print("Enter: <left_mps> <right_mps> [duration_s]")
    print("   or: step <left_mps> <right_mps> [segment_s]")
    print("   or: stop   (q to quit)")

    while True:
        try:
            cmd = input("> ").strip()
        except (KeyboardInterrupt, EOFError):
            print("\nExiting.")
            break

        if not cmd:
            continue

        if cmd.lower() in ("q", "quit", "exit"):
            break

        try:
            parsed = parse_command(cmd)
            if parsed is None:
                print("Invalid input. Examples: 0.10 0.10 3   or   step 0.10 0.10 2   or   stop")
                continue

            mode, left_mps, right_mps, duration_s = parsed
            if mode != "stop" and duration_s <= 0.0:
                print("Duration must be > 0")
                continue

            if mode == "stop":
                run_immediate_stop(controller)
            elif mode == "step":
                run_step_test(controller, encoders, left_mps, right_mps, duration_s)
            else:
                run_response_test(controller, encoders, left_mps, right_mps, duration_s)
        except ValueError:
            print("Invalid numbers. Example: 0.10 -0.10 2.5   or   step 0.10 0.10 2")
        except Exception as exc:
            print("Test error:", type(exc).__name__, exc)

    controller.stop(immediate=True)
    print("Motors stopped.")


if __name__ == "__main__":
    main()
