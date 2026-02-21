# Arduino car sonar obstacle avoidance example using Robo-Tx framework
# (C) Kashif Baig
# 
# Deploy Robo-Tx firmware to Arduino (see link below). 
# It is recommened to use a Daokai L298P motor driver board (see images folder)
# and set the SELECTED_PROFILE to PROFILE_ROBOT_MOTOR_SHIELD in file Settings.h
#
# https://github.com/kashif-baig/RoboTx_Firmware
# https://www.cohesivecomputing.co.uk/obstacle-avoiding-vehicle-driven-using-python/
#
# Otherwise, set SELECTED_PROFILE to an appropriate profile in file Settings.h
# that matches your own configuration.
#
# This example requires an SR04 sonar module and a 9g servo mounted on a holder that permits panning.
# Connect the devices to the Arduino pins as per the selected firmware profile settings.
# Be sure to sufficiently power the DC and servo motors, otherwise the Arduino will keep resetting.
# A Bluetooth module (e.g. HC-05) will also need connecting to the Arduino, and must be configured
# for 57600 baud.
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
from RoboTx.Api import RobotIO
from RoboTx import *


# If connecting directly to an Arduino using USB, you can find the serial port using the Arduino IDE.
dtr_enable = False
serial_port = "COM16"
serial_baud_rate = 115200

car = RobotIO(serial_port, serial_baud_rate, dtr_enable)
try:
    car.Connect()
    print("Press Enter to stop program.")

    # Thread to detect Enter key
    detectEnter = threading.Thread(target = input)
    detectEnter.start()

    turn_left = True

    # Serial communications latency in milliseconds
    comms_latency = 0.020

    # Max time for sonar echo (i.e. time for ping to travel 5m and back)
    echo_wait_time = 0.030 + comms_latency

    # Set servo to face directly forward (90 degrees).
    car.Servo1.SetPosition(90)

    # Normally 9g servo has speed of 0.12sec/60 degrees
    servo_sec_per_degree = 0.120 / 60

    # Calculate max time servo takes to swing to 90 degrees
    servo_wait_time = servo_sec_per_degree * 90
    time.sleep(servo_wait_time + comms_latency)

    avoidance_duration = 0.500
    minimum_clearance = 50
    normal_motor_speed = 35

    motorL = car.Motor1
    motorR = car.Motor2

    # Set speed multiplier if vehicle doesn't travel in a straight line.
    # Otherwise, comment it out.
    car.MotorConfig.SetSpeedMultiplier(38.0/35.0, motorL)

    while detectEnter.is_alive():

        car.Sonar.Ping()

        if car.Sonar.DistanceAcquired:
            # Get distance reported in centimeters.
            distance_in_centimeters = car.Sonar.GetDistance()

            if 0 <= distance_in_centimeters < 25:
                # Obstacle ahead, so stop.
                motorR.Drive(0)
                motorL.Drive(0)

                # Swing servo left 60 degrees (from 90 degrees).
                car.Servo1.SetPosition(150)
                servo_wait_time = servo_sec_per_degree * 60
                time.sleep(servo_wait_time + comms_latency)

                car.Sonar.Ping()
                time.sleep(echo_wait_time)

                distance_at_left = car.Sonar.GetDistance()

                # Swing servo right 120 degrees (from 135 degrees).
                car.Servo1.SetPosition(30)
                servo_wait_time = servo_sec_per_degree * 120
                time.sleep(servo_wait_time + comms_latency)

                car.Sonar.Ping()
                time.sleep(echo_wait_time)

                distance_at_right = car.Sonar.GetDistance()
                
                car.Servo1.SetPosition(90)

                if distance_at_left > minimum_clearance and distance_at_right > minimum_clearance:
                    # There is clearance on both left and right of the vehicle.
                    # Rotate either left or right to avoid obstacle.
                    if turn_left:
                        motorR.Drive(normal_motor_speed)
                        motorL.Drive(-normal_motor_speed)
                    else:
                        motorR.Drive(-normal_motor_speed)
                        motorL.Drive(normal_motor_speed)
                    time.sleep(avoidance_duration)
                    turn_left = not turn_left
                elif distance_at_left > minimum_clearance:
                    # Rotate left to avoid obstacle.
                    motorR.Drive(normal_motor_speed)
                    motorL.Drive(-normal_motor_speed)
                    time.sleep(avoidance_duration)
                    turn_left = False
                elif distance_at_right > minimum_clearance:
                    # Rotate right to avoid obstacle.
                    motorR.Drive(-normal_motor_speed)
                    motorL.Drive(normal_motor_speed)
                    time.sleep(avoidance_duration)
                    turn_left = True
                else:
                    # No clearance ahead, so reverse, then rotate either left or right for a longer duration.
                    print("Reversing ...")
                    motorR.Drive(-normal_motor_speed)
                    motorL.Drive(-normal_motor_speed)
                    time.sleep(avoidance_duration)

                    if turn_left:
                        print("Turning left ...")
                        motorR.Drive(normal_motor_speed)
                        motorL.Drive(-normal_motor_speed)
                    else:
                        print("Turning right ...")
                        motorR.Drive(-normal_motor_speed)
                        motorL.Drive(normal_motor_speed)

                    print("Brief pause ...")
                    time.sleep(avoidance_duration * 1.5)

                # Stop rotating.
                motorR.Drive(0)
                motorL.Drive(0)

                servo_wait_time = servo_sec_per_degree * 60
                time.sleep(servo_wait_time + comms_latency)
            else:
                motorR.Drive(normal_motor_speed)
                motorL.Drive(normal_motor_speed)

        # Short pause is necessary before next loop iteration
        time.sleep(0.01)
finally:
    car.Close()
    
