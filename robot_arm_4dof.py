# Arduino 4 degrees of freedom robot arm example using Robo-Tx framework
# (C) Kashif Baig
# 
# Deploy Robo-Tx firmware to Arduino (see link below). Make sure
# SELECTED_PROFILE is set to PROFILE_ROBOT_ARM_4DF in file Settings.h
# It is recommended to use the Sensor Shield v5.0 with a separate power
# supply for the servos (see images folder).
#
# https://github.com/kashif-baig/RoboTx_Firmware
# https://www.cohesivecomputing.co.uk/robotic-arm-control-using-python/
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
serial_port = "COM3"
serial_baud_rate = 115200

robotArm = RobotIO(serial_port, serial_baud_rate, dtr_enable)
try:
    print("Connecting ...")
    robotArm.Connect()
    print("OK.")

    # Thread to detect Enter key
    detectEnter = threading.Thread(target = input)
    detectEnter.start()

    mid_point = 93

    pan = robotArm.Servo1
    shoulder = robotArm.Servo2
    elbow = robotArm.Servo3
    pincer = robotArm.Servo4
    
    robotArm.ServoConfig.SetSpeedLimit(5, pan)
    robotArm.ServoConfig.SetSpeedLimit(3, shoulder, elbow, pincer)

    robotArm.ServoConfig.SetAngleLimits(20, 175, pan)
    robotArm.ServoConfig.SetAngleLimits(mid_point-30, 130, shoulder)
    robotArm.ServoConfig.SetAngleLimits(mid_point, 145, elbow)
    robotArm.ServoConfig.SetAngleLimits(mid_point, 125, pincer)

    # Reset joint positions
    # Mid point is where pan is centered, pincer is closed, shoulder and elbow are retracted.
    pan.SetPosition(mid_point)
    shoulder.SetPosition(mid_point)
    elbow.SetPosition(mid_point)
    pincer.SetPosition(mid_point)
    time.sleep(0.3)

    # Lower arm
    shoulder.SetPosition(mid_point + 40)
    elbow.SetPosition(mid_point - 30)

    # Open pincer
    pincer.SetPosition(pincer.Range.UpperLimit)
    time.sleep(0.9)

    # Extend elbow
    elbow.SetPosition(mid_point + 30)
    time.sleep(1.5)

    # Close pincer
    pincer.SetPosition(pincer.Range.LowerLimit)
    time.sleep(0.9)

    # Retract arm
    elbow.SetPosition(mid_point)
    shoulder.SetPosition(mid_point)
    time.sleep(1.4)

    # Pan anti-clockwise
    pan.SetPosition(pan.Range.UpperLimit)
    time.sleep(1.2)

    # Open pincer
    pincer.SetPosition(pincer.Range.UpperLimit)
    time.sleep(0.9)

    # Close pincer
    pincer.SetPosition(pincer.Range.LowerLimit)
    time.sleep(0.9)

    # Return pan to mid position.
    pan.SetPosition(mid_point)
    time.sleep(0.9)

    print("Press Enter to stop program.")

    while detectEnter.is_alive():
        # Short pause is necessary before next loop iteration
        time.sleep(0.05)
finally:
    # Close connection to Arduino
    robotArm.Close()
    print("Disconnecting.")
