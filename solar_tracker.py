# Arduino solar tracker example using Robo-Tx framework
# (C) Kashif Baig
# 
# Deploy Robo-Tx firmware to Arduino (see link below). Make sure
# SELECTED_PROFILE is set to PROFILE_UNO_MOTOR_PLUS in file Settings.h
# It is recommended to use the Sensor Shield v5.0 with a separate power
# supply for the servos (see images folder).
#
# https://github.com/kashif-baig/RoboTx_Firmware
# https://www.cohesivecomputing.co.uk/solar-tracking-robot-control-using-python/
#
# Connect the servo motors to the Arduino pins as per the selected firmware profile settings.
#
# Robo-Tx API online help: https://help.cohesivecomputing.co.uk/Robo-Tx
#
# All examples are provided as is and at user's own risk.

import time
import threading
from pythonnet import load
load("coreclr")
import clr

# Use correct path for your OS platform.
clr.AddReference("./RoboTx/win-x64/RoboTx.Api")
from RoboTx.Api import RobotIO,Input
from RoboTx import *

# If connecting directly to an Arduino using USB, you can find the serial port using the Arduino IDE.
dtr_enable = False
serial_port = "COM4"
serial_baud_rate = 115200

# The tolerance varies according to the intensity of ambient light.
# The indoor light source changes greatly, whereas the change is small out in the sun.
# The value range is 10~100.
tolerance = 50

# Delay parameter, the smaller the value, the faster the response speed.
delay_time = 0.02

# Default Angle
servo_h_angle = 90
servo_h_limit_high = 175
servo_h_limit_low = 5

servo_v_limit_high = 170
servo_v_limit_low = 85
# Default angle
servo_v_angle = (servo_v_limit_low + servo_v_limit_high) / 2

solarTracker = RobotIO(serial_port, serial_baud_rate, dtr_enable)
try:
    print("Connecting ...")
    solarTracker.Connect()
    print("OK.")
    print("Press Enter to stop program.")

    # Enable analog inputs for light sensors.
    solarTracker.Analog.EnableInputsA(0, 1, 2, 3)

    # Thread to detect Enter key.
    detectEnter = threading.Thread(target = input)
    detectEnter.start()

    servo_h = solarTracker.Servo1
    servo_v = solarTracker.Servo2

    while detectEnter.is_alive():
        # Read light sensor values.
        # The sensor locations are identified from the view.
        # of looking at the solar panel. 
        left_top = solarTracker.Analog.A0.Value
        right_top = solarTracker.Analog.A1.Value
        left_down = solarTracker.Analog.A2.Value
        right_down = solarTracker.Analog.A3.Value

        # Calculate averages of adjacent readings.
        average_top = (left_top + right_top) / 2
        average_down = (left_down + right_down) / 2
        average_left = (left_top + left_down) / 2
        average_right = (right_top + right_down) / 2

        # Calculate differences.
        difference_vertical = average_top - average_down
        difference_horizontal = average_left - average_right

        # Check if the difference is within tolerance, otherwise change the vertical angle.
        if -1 * tolerance > difference_vertical or difference_vertical > tolerance:
            if average_top > average_down:
                servo_v_angle += 1
                if servo_v_angle > servo_v_limit_high:
                    servo_v_angle = servo_v_limit_high
            elif average_top < average_down:
                servo_v_angle -= 1
                if servo_v_angle < servo_v_limit_low:
                    servo_v_angle = servo_v_limit_low

            # If the servo rotation angle is opposite to the light,
            # use (180 - servo_v_angle) or (servo_v_angle) to switch the direction.
            servo_v.SetPosition(servo_v_angle)


        # Check if the difference is within tolerance, otherwise change the horizontal angle.
        if -1 * tolerance > difference_horizontal or difference_horizontal > tolerance:
            if average_left > average_right:
                servo_h_angle -= 1
                if servo_h_angle < servo_h_limit_low:
                    servo_h_angle = servo_h_limit_low
            elif average_left < average_right:
                servo_h_angle += 1
                if servo_h_angle > servo_h_limit_high:
                    servo_h_angle = servo_h_limit_high

            # If the servo rotation angle is opposite to the light, just
            # use (180 - servo_h_angle) or (servo_h_angle) to switch the direction.
            servo_h.SetPosition(180 - servo_h_angle)

        # Short pause is necessary before next loop iteration.
        time.sleep(delay_time)
finally:
    # Close connection to Arduino.
    solarTracker.Close()
    print("Disconnecting.")
