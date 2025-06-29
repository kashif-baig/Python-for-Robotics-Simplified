# Arduino car line follower example using Robo-Tx framework
# (C) Kashif Baig
# 
# Deploy Robo-Tx firmware to Arduino (see link below). 
# It is recommened to use a Daokai L298P motor driver board (see images folder)
# and set the SELECTED_PROFILE to PROFILE_ROBOT_MOTOR_SHIELD in file Settings.h
#
# https://github.com/kashif-baig/RoboTx_Firmware
# https://www.cohesivecomputing.co.uk/line-following-vehicle-driven-using-python/
#
# Otherwise, set SELECTED_PROFILE to an appropriate profile in file Settings.h
# that matches your own configuration.
#
# This example uses a 5 channel digital line sensor connected to Arduino
# pins A0 to A4. A Bluetooth module (e.g. HC-05) will also need connecting to
# the Arduino, and must be configured for 57600 baud.
#
# Robo-Tx API online help: https://help.cohesivecomputing.co.uk/Robo-Tx
#
# All examples are provided as is and at user's own risk.

import threading
import time
from pythonnet import load
load("coreclr")
import clr

# Use correct path for your OS platform.
clr.AddReference("./RoboTx/win-x64/RoboTx.Api")
from RoboTx.Api import RobotIO, Digital
from RoboTx import *

# Returns position of line detected by sensor(s). Negative values indicate left side, positive indicate right side. 
def find_line_position(sensor :Digital):
    line_position = 0
    line_detected = False

    far_right_sensor = sensor.IN4.Value
    right_sensor = sensor.IN3.Value
    mid_sensor = sensor.IN2.Value
    left_sensor = sensor.IN1.Value
    far_left_sensor = sensor.IN0.Value

    line_sensors_triggered = sensor.InputCount

    if line_sensors_triggered == 5:
        # All sensors have been triggered. Let's keep things simple and treat as
        # no line found.
        line_detected = False
    elif line_sensors_triggered > 0:
        # At least one line sensor was triggered.
        # Where two adjacent sensors have been triggered, the resulting line position is the mid point between them.

        line_detected = True

        if left_sensor and mid_sensor and right_sensor:
            line_position = 0
        elif far_left_sensor:
            line_position = -20
            if left_sensor:
                line_position += 5
        elif left_sensor:
            line_position = -10
            if mid_sensor:
                line_position += 5
        elif far_right_sensor:
            line_position = 20
            if right_sensor:
                line_position -= 5
        elif right_sensor:
            line_position = 10
            if mid_sensor:
                line_position -= 5
        elif mid_sensor:
            line_position = 0
            
    return line_detected, line_position

# If connecting directly to an Arduino using USB, you can find the serial port using the Arduino IDE.
dtr_enable = False
serial_port = "COM16"
serial_baud_rate = 115200

car = RobotIO(serial_port, serial_baud_rate, dtr_enable)
try:
    print("Connecting ...")
    car.Connect()
    print("OK.")
    print("Press Enter to stop program.")

    # Thread to detect Enter key
    detectEnter = threading.Thread(target = input)
    detectEnter.start()

    # Enable triggering of input events whose sources are Arduino pins A0, A1, A2, A3 and A4.
    car.Digital.EnableInputs(0, 1, 2, 3, 4)

    # Need a short delay for enabled inputs to start being received.
    time.sleep(0.25)
    
    # Set max speed as percentage.
    max_linear_speed = 35

    motorL = car.Motor1
    motorR = car.Motor2

    # Set speed multiplier for a motor to match speed of other motor.
    # Otherwise, comment line out.
    car.MotorConfig.SetSpeedMultiplier(38.0/35.0, motorL)

    line_position = 0

    while detectEnter.is_alive():

        prev_line_position = line_position
        line_detected, line_position = find_line_position(car.Digital)

        if line_detected:

            steering_adjust = abs(line_position) * 0.5
            speed_adjust = 10.0 / abs(line_position) if line_position != 0 else 1

            # The amount by which steering is performed is proportional to
            # how far the line position is from the mid point.

            if line_position == 0:
                # Line is at mid position, so drive straight forward.
                motorR.Drive(max_linear_speed)
                motorL.Drive(max_linear_speed)
            elif line_position > 0:
                if line_position == 20:
                    # Line is far right, so rotate right.
                    motorL.Drive(max_linear_speed * speed_adjust)
                    motorR.Drive(-max_linear_speed * speed_adjust)
                elif line_position == 15:
                    # Line is further right, so steer more right.
                    motorL.Drive((max_linear_speed + steering_adjust) * speed_adjust)
                    motorR.Drive((max_linear_speed - steering_adjust) * speed_adjust)
                else:
                    # Line is at right, so steer right.
                    motorL.Drive(max_linear_speed)
                    motorR.Drive(max_linear_speed - steering_adjust)
            else:
                if line_position == -20:
                    # Line is far left, so rotate left.
                    motorL.Drive(-max_linear_speed * speed_adjust)
                    motorR.Drive(max_linear_speed * speed_adjust)
                elif line_position == -15:
                    # Line is further left, so steer more left.
                    motorL.Drive((max_linear_speed - steering_adjust) * speed_adjust)
                    motorR.Drive((max_linear_speed + steering_adjust) * speed_adjust)
                else:
                    # Line is at left, so steer left.
                    motorL.Drive(max_linear_speed - steering_adjust)
                    motorR.Drive(max_linear_speed)
        else:
            # No line detected

            # If the car has over-steered and missed the line, attempt to
            # correct course for a short duration
            if prev_line_position > 0:
                motorR.DriveForDuration(-20, .75)
                motorL.DriveForDuration(20, .75)
            elif prev_line_position < 0:
                motorR.DriveForDuration(20, .75)
                motorL.DriveForDuration(-20, .75)
            elif motorL.DurationLapsed and motorR.DurationLapsed:
                # Otherwise, just stop.
                motorR.Drive(0)
                motorL.Drive(0)

        # Short pause is necessary before next loop iteration
        time.sleep(0.01)
finally:
    car.Close()
    print("Disconnecting.")
