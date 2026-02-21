# Arduino Multi-function board low light sensing example using Robo-Tx framework
# (C) Kashif Baig
# 
# Deploy Robo-Tx firmware to Arduino (see link below). Make sure
# SELECTED_PROFILE is set to PROFILE_MULTI_FUNC_SHIELD in file Settings.h
#
# https://github.com/kashif-baig/RoboTx_Firmware
# https://www.cohesivecomputing.co.uk/robo-tx/
#
# Connect an analog light sensor to Arduino pin A5 such that the analog value increases
# as light levels fall. If low light level exceeds a threshold, an LED on the multi-function
# board lights up.
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
from RoboTx.Api import RobotIO,Input,AnalogConverter
from RoboTx import *

# Callback function to convert raw analog value to percentage
def raw_to_percent(value):
    return (value * 100) / 1023

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

    lowLightIndicator = mfs.Switch1
    lightSensor = mfs.Analog.A5

    # Register callback function to convert raw analog value to percent value (between 0 and 100). 
    mfs.Analog.UseConverter(AnalogConverter(raw_to_percent), lightSensor)

    while detectEnter.is_alive():
        if lightSensor.Value > 50:
            lowLightIndicator.On()
        else:
            lowLightIndicator.Off()

        # Short pause is necessary before next loop iteration
        time.sleep(0.05)

finally:
    mfs.Close()
