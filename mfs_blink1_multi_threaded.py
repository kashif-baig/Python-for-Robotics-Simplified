# Arduino Multi-function board example using Robo-Tx framework
# (C) Kashif Baig
# 
# Deploy Robo-Tx firmware to Arduino (see link below). Make sure
# SELECTED_PROFILE is set to PROFILE_MULTI_FUNC_SHIELD in file Settings.h
#
# https://github.com/kashif-baig/RoboTx_Firmware
# https://www.cohesivecomputing.co.uk/robo-tx/
#
# This example uses multi-threading to blink an LED.
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
from RoboTx.Api import RobotIO,Input,Switch,ConnectionState
from RoboTx import *

# Define thread that blinks an LED
def thread_blink(conn :ConnectionState, led : Switch):
    blinkState = True
    while not conn.IsClosing:
        if blinkState:
            led.On()
        else:
            led.Off()
        time.sleep(1)
        blinkState = not blinkState

blinker = None

# If connecting directly to an Arduino using USB, you can find the serial port using the Arduino IDE.
dtr_enable = False
serial_port = "COM3"
serial_baud_rate = 115200

mfs = RobotIO(serial_port, serial_baud_rate, dtr_enable)
try:
    print("Connecting ...")
    mfs.Connect()
    print("OK.")
    print("Press Enter to stop program.")

    # Thread to detect Enter key
    detectEnter = threading.Thread(target = input)
    detectEnter.start()

    # Create and start the blink thread
    blinker = threading.Thread(target=thread_blink, args=(mfs.ConnectionState, mfs.Switch1,))
    blinker.start()

    while detectEnter.is_alive():
        # Short pause is necessary before next loop iteration
        time.sleep(0.025)
finally:
    # Notify blink thread that connection to Arduino is about to be closed.
    mfs.NotifyClosing()

    # Wait for blink thread to finish
    if blinker is not None:
        blinker.join()

    # Close connection to Arduino
    mfs.Close()
    print("Disconnecting.")
