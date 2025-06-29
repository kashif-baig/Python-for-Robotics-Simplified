# Arduino Multi-function board basic example using Robo-Tx framework
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

import time
from pythonnet import load
load("coreclr")
import clr

# Use correct path for your OS platform.
clr.AddReference("./RoboTx/win-x64/RoboTx.Api")
from RoboTx.Api import RobotIO,Input
from RoboTx import *

# If connecting directly to an Arduino using USB, you can find
# the serial port using the Arduino IDE.
dtr_enable = False
serial_port = "COM3"
serial_baud_rate = 115200

mfs = RobotIO(serial_port, serial_baud_rate, dtr_enable)
try:
    print("Connecting ...")
    mfs.Connect()
    print("OK.")
    print("Hold button 3 on board to stop program.")

    # Analog inputs have already been enabled in the firmware profile, so no need to do here.
    # mfs.Analog.EnableInputsA(0,4,5) # Arduino pins A0, A4, A5

    # Digital pins have been defined and enabled in the profile, so no need to do here.
    # mfs.Digital.EnableInputs(0,1,2) # Arduino pins A1, A2, A3

    # Turn on Led1
    mfs.Switch1.On()

    # Can alias Robo-Tx objects
    led4 = mfs.Switch4
    led4.On()

    # Sound the beeper
    mfs.Trigger.Pulse()

    # Write to the LED display
    mfs.LedDisplay.Write(1234)

    # Report value of preset pot to console
    print(mfs.Analog.A0)

    while True:
        input_event = mfs.Digital.GetInputEvent()

        # Exit program if button 3 is held down
        if input_event == Input.BUTTON_3_SUSTAINED:
            break

        # Short pause is necessary before next loop iteration
        time.sleep(0.025)
finally:
    mfs.Close()
    print("Disconnecting.")
