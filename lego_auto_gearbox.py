# Automated gearbox using LEGO, Arduino and Robo-Tx
# (C) Kashif Baig
#
# Deploy Robo-Tx firmware to Arduino (see link below). Make sure
# SELECTED_PROFILE is set to PROFILE_ROBOT_MOTOR_SHIELD in file Settings.h
#
# https://github.com/kashif-baig/RoboTx_Firmware
# https://www.cohesivecomputing.co.uk/automated-gearbox-control-with-lego-technic-and-python/
#
# Robo-Tx API online help: https://help.cohesivecomputing.co.uk/Robo-Tx
#
# All examples are provided as is and at user's own risk.

import time
import datetime as dt
import threading
from pythonnet import load

load("coreclr")
import clr

# Use correct path for your OS platform.
clr.AddReference("./RoboTx/win-x64/RoboTx.Api")
from RoboTx.Api import RobotIO, Input
from RoboTx import *

# Define motor states
state_neutral = 0
state_constant = 1
state_decelerating = 2
state_accelerating = 3

# Define IR commands
cmd_slow_down = 21
cmd_speed_up = 9

# If connecting directly to an Arduino using USB, you can find the serial port using the Arduino IDE.
dtr_enable = True
serial_port = "COM8"
serial_baud_rate = 115200

autoGearBox = RobotIO(serial_port, serial_baud_rate, dtr_enable)
try:
    print("Connecting ...")
    autoGearBox.Connect()
    print("OK.")
    print("Press Enter to stop the program.")

    # Thread to detect Enter key
    detectEnter = threading.Thread(target=input)
    detectEnter.start()

    gear_lever = autoGearBox.Servo1
    # Configure servo range as 360 degrees.
    autoGearBox.ServoConfig.SetType(360, 500, 2500, gear_lever)

    # Set gear change latency in milliseconds.
    gear_change_latency = 0.030

    # Assume servo has speed of 0.18 sec/60 degrees.
    servo_ms_per_degree = 180 / 60000

    # Define servo positions for gears as angles
    neutral_position = 178
    first_gear_position = 145
    second_gear_position = 210

    # Enable pulse measuring on pin A2.
    timeout_ms = 1000  # clear measurement if no pulse detected within this period.
    trigger = 1  # pulse start detected on high signal.
    autoGearBox.PulseCounter.Enable(timeout_ms, trigger)

    # Shift in to neutral. Wiggle the gear lever back and forth
    # to ensure it properly rests in the neutral position.
    gear_lever.SetPosition(neutral_position - 15)
    time.sleep(0.2)
    gear_lever.SetPosition(neutral_position + 15)
    time.sleep(0.2)
    gear_lever.SetPosition(neutral_position)

    # Simulating motor under load. Lower values result in faster acceleration.
    motor_acceleration = 3

    # Calculate max time servo takes to swing from one gear position to another.
    servo_wait_time = servo_ms_per_degree * 35
    time.sleep(servo_wait_time + gear_change_latency)

    rpm = 0
    prev_rpm = -1

    is_in_first_gear = False
    state = state_neutral

    # Values obtained by trial and error when motor not under load.
    up_shift_rpm_threshold = 400
    up_shift_start_speed = 18
    down_shift_rpm_threshold = 370
    down_shift_start_speed = 40

    last_gear_shift_time = dt.datetime.now()

    # last valid IR button press
    last_ir_cmd_code = -1

    motor = autoGearBox.Motor1

    while detectEnter.is_alive():
        # Get an infra-red command, if received.
        ir_cmd = autoGearBox.GetIRCommand()

        if ir_cmd.Received:
            ir_state = "pressed" if ir_cmd.ButtonPressed else "released"
            print(f"IR Cmd: {ir_cmd.Code} {ir_state}")

            if ir_cmd.ButtonPressed:
                last_ir_cmd_code = ir_cmd.Code
            if ir_cmd.Code == cmd_slow_down:
                if ir_cmd.ButtonPressed:
                    # Slow down button was pressed
                    # Start slowing down motor
                    # motor.Drive(0)
                    if not is_in_first_gear and rpm > 0:
                        motor.Drive(up_shift_start_speed - 2)
                    elif rpm > 0:
                        motor.Drive(0)
                    print("Slowing to stop")
                    state = state_decelerating
                elif ir_cmd.ButtonReleased and rpm > 0:
                    # Slow down button was released
                    # Hold current speed
                    motor.StopAccelerating()
                    state = state_constant
            elif ir_cmd.Code == cmd_speed_up:
                if ir_cmd.ButtonPressed:
                    # Speed up button was pressed.
                    if rpm == 0:
                        # Shift in to 1st gear if starting from stationary
                        motor.SetAcceleration(motor_acceleration)
                        gear_lever.SetPosition(first_gear_position)
                        # time.sleep(servo_wait_time + gear_change_latency)
                        is_in_first_gear = True
                        last_gear_shift_time = dt.datetime.now()
                    # Start speeding up the motor
                    motor.Drive(95)
                    print("Speeding up")
                    state = state_accelerating
                elif ir_cmd.ButtonReleased:
                    # Speed up button was released
                    # Hold current speed
                    motor.StopAccelerating()
                    state = state_constant

        # Calculate motor RPM
        rpm = (
            int((1000 * 30) / autoGearBox.PulseCounter.Period)
            if autoGearBox.PulseCounter.Period > 0
            else 0
        )
        # print(f"RPM {rpm}")
        if rpm != prev_rpm:
            prev_rpm = rpm
            print(f"RPM {rpm}")
        if rpm == 0:
            # Reported RPM is 0.
            if state != state_neutral and last_ir_cmd_code == cmd_slow_down:
                # Slow down button was the last button pressed
                # Shift in to neutral and stop the motor
                state = state_neutral
                gear_lever.SetPosition(neutral_position + 10)
                time.sleep(0.2)
                gear_lever.SetPosition(neutral_position)
                motor.Drive(0)

        if (
            rpm > up_shift_rpm_threshold
            and is_in_first_gear
            and (dt.datetime.now() - last_gear_shift_time).total_seconds() > 2
        ):  # and last_ir_cmd_code == cmd_speed_up:
            # Upward gear change threshold exceeded
            motor.StopAccelerating()
            print("Gear change up")
            motor.SetAcceleration(motor_acceleration)

            # Shift in to neutral and slow down the motor
            gear_lever.SetPosition(neutral_position)
            time.sleep(servo_wait_time)
            motor.DriveNoAccel(up_shift_start_speed)

            # Shift in to 2nd gear
            gear_lever.SetPosition(second_gear_position)
            time.sleep(servo_wait_time + gear_change_latency)

            if state == state_accelerating:
                # Start accelerating
                motor.Drive(95)

            is_in_first_gear = False
            last_gear_shift_time = dt.datetime.now()
        elif (
            rpm != 0
            and rpm < down_shift_rpm_threshold
            and not is_in_first_gear
            and (dt.datetime.now() - last_gear_shift_time).total_seconds() > 2
        ):  # and last_ir_cmd_code == cmd_slow_down:
            # Downward gear change threshold exceeded
            motor.StopAccelerating()
            print("Gear change down")
            motor.SetAcceleration(motor_acceleration)

            # Shift in to neutral and speed up the motor.
            gear_lever.SetPosition(neutral_position)
            time.sleep(servo_wait_time)
            motor.DriveNoAccel(down_shift_start_speed)

            # Shift in to 1st gear
            gear_lever.SetPosition(first_gear_position)
            time.sleep(servo_wait_time + gear_change_latency)

            if state == state_decelerating:
                # Start decelerating
                motor.Drive(0)
            is_in_first_gear = True
            last_gear_shift_time = dt.datetime.now()
        time.sleep(0.01)
finally:
    autoGearBox.Close()
    print("Disconnecting.")
