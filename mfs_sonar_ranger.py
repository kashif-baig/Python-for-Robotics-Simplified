# Arduino Multi-function board example using Robo-Tx framework
# (C) Kashif Baig
# 
# Deploy Robo-Tx firmware to Arduino (see link below). Make sure
# SELECTED_PROFILE is set to PROFILE_MULTI_FUNC_SHIELD in file Settings.h
#
# https://github.com/kashif-baig/RoboTx_Firmware
# https://www.cohesivecomputing.co.uk/robo-tx/
#
# This example demonstrates distance measuring using a sonar module.
# Connect the module trigger pin to Arduino pin 6, and the module echo pin
# to Arduino pin 9. 
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
from RoboTx.Api import RobotIO,Input
from RoboTx import *

# If connecting directly to an Arduino using USB, you can find the serial port using the Arduino IDE.
dtr_enable = False
serial_port = "COM3"
serial_baud_rate = 115200

mfs = RobotIO(serial_port, serial_baud_rate, dtr_enable)
try:
    mfs.Connect()
    print("Press Enter to stop program.")

    # Thread to detect Enter key
    detectEnter = threading.Thread(target = input)
    detectEnter.start()

    # Start beeper repeating indefinitely: 50ms beep, 950ms delay.
    mfs.Trigger.Repeat(50, 950)

    while detectEnter.is_alive():
        mfs.Sonar.Ping()

        if mfs.Sonar.DistanceAcquired:
            # Get distance reported in centimeters.
            distance_cm = mfs.Sonar.GetDistance()
            print(distance_cm)

            if distance_cm != 0 and distance_cm < 200:
                off_period = (distance_cm * 10) - 30

                if off_period < 0:
                    off_period = 0
                    
                # Change beeper delay depending on distance.
                mfs.Trigger.SetOffPeriod(off_period)

            mfs.LedDisplay.Write(distance_cm)

        # Short pause is necessary before next loop iteration
        time.sleep(0.25)
finally:
    mfs.Close()

