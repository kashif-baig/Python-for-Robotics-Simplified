# Arduino Multi-function board blink example using Robo-Tx framework
# (C) Kashif Baig
# 
# Deploy Robo-Tx firmware to Arduino (see link below). Make sure
# SELECTED_PROFILE is set to PROFILE_MULTI_FUNC_SHIELD in file Settings.h
#
# https://github.com/kashif-baig/RoboTx_Firmware
# https://www.cohesivecomputing.co.uk/robo-tx/
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
serial_port = "COM13"
serial_baud_rate = 115200

mfs = RobotIO(serial_port, serial_baud_rate, dtr_enable)
try:
    mfs.Connect()
    print("Press Enter to stop program.")

    # Thread to detect Enter key
    detectEnter = threading.Thread(target = input)
    detectEnter.start()

    led1 = mfs.Switch1
    led1.On()

    blinkState = True
    
    while detectEnter.is_alive():

        if blinkState:
            led1.On()
        else:
            led1.Off()

        blinkState = not blinkState
        time.sleep(1)
finally:
    mfs.Close()

