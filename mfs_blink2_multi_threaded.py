# Arduino Multi-function board example using Robo-Tx framework
# (C) Kashif Baig
# 
# Deploy Robo-Tx firmware to Arduino (see link below). Make sure
# SELECTED_PROFILE is set to PROFILE_MULTI_FUNC_SHIELD in file Settings.h
#
# https://github.com/kashif-baig/RoboTx_Firmware
# https://www.cohesivecomputing.co.uk/robo-tx/
#
# This example uses multi-threading to blink an LED. Press button 1 to 
# enable or disable LED blinking.
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

# Define thread that blinks an LED
def thread_blink(mfs :RobotIO, context :dict[str, str]):
    blinkState = False

    while not mfs.ConnectionState.IsClosing:

        if context["blink_on"]=="False":
            if blinkState:
                mfs.Switch1.Off()
                blinkState = False
            # Since here the thread is not doing any work, just sleep a bit to free up CPU
            time.sleep(0.1)
            continue

        blinkState = not blinkState

        if blinkState:
            mfs.Switch1.On()
        else:
            mfs.Switch1.Off()
        time.sleep(1)

context = {
  "blink_on": "False"
}

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
    print("Press button 1 on shield to start/stop LED blinking.")
    print("Press Enter to stop program.")

    # Thread to detect Enter key
    detectEnter = threading.Thread(target = input)
    detectEnter.start()

    # Create and start the blink thread
    blinker = threading.Thread(target=thread_blink, args=(mfs,context,))
    blinker.start()

    while detectEnter.is_alive():
        # If a button was pressed, an input event will have been generated.
        input_event = mfs.Digital.GetInputEvent()

        # If button 1 pressed, tell background thread to start/stop LED blinking.
        if input_event == Input.BUTTON_1_PRESSED:
            if context["blink_on"]=="False":
                context["blink_on"]="True"
            else:
                context["blink_on"]="False"
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
