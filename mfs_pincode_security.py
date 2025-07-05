# Arduino Multi-function board example using Robo-Tx framework
# (C) Kashif Baig
# 
# Deploy Robo-Tx firmware to Arduino (see link below). Make sure
# SELECTED_PROFILE is set to PROFILE_MULTI_FUNC_SHIELD in file Settings.h
#
# https://github.com/kashif-baig/RoboTx_Firmware
# https://www.cohesivecomputing.co.uk/robo-tx/
#
# This example simulates a security system that needs the correct 4 digit pin code to gain access.
# Attach an IR sensor to the socket on the board before running the source code and use 
# a 'Special for MP3' or similar IR remote that uses NEC protocol to enter a pin code.
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
    print("Connecting ...")
    mfs.Connect()
    print("OK.")
    print("Press Enter to stop program.")

    # Thread to detect Enter key
    detectEnter = threading.Thread(target = input)
    detectEnter.start()

    # Define IR command code to digit mapping
    ir_cmd_code_to_digit = {
        22: "0",
        12: "1",
        24: "2",
        94: "3",
        8: "4",
        28: "5",
        90: "6",
        66: "7",
        82: "8",
        74: "9",
    }

    pass_key = "1234"
    code_entered = ""

    while detectEnter.is_alive():

        # Get an infra-red command, if received.
        ir_command = mfs.Digital.GetIRCommand()

        if ir_command.Received:
            ir_state = "pressed" if ir_command.ButtonPressed else "released"
            print(f"IR Cmd: {ir_command.Code} {ir_state}")

            if ir_command.ButtonPressed and ir_command.Code in ir_cmd_code_to_digit:
                # Format code entered for display
                if len(code_entered) >= 4:
                    code_entered = ""
                
                code_entered += ir_cmd_code_to_digit[ir_command.Code]
                mfs.LedDisplay.Write(code_entered)

                if len(code_entered) == len(pass_key):
                    if code_entered == pass_key:
                        # Pin code accepted. Sound long beep.
                        mfs.Trigger.Pulse(500)
                    else:
                        # Wrong pin code. Sound 3 short beeps.
                        mfs.Trigger.RunPattern(50, 50, 3)
        
        # Short pause is necessary before next loop iteration
        time.sleep(0.025)
finally:
    mfs.Close()
    print("Disconnecting.")
