# Arduino Multi-function board example using Robo-Tx framework
# (C) Kashif Baig
# 
# Deploy Robo-Tx firmware to Arduino (see link below). Make sure
# SELECTED_PROFILE is set to PROFILE_MULTI_FUNC_SHIELD in file Settings.h
#
# https://github.com/kashif-baig/RoboTx_Firmware
# https://www.cohesivecomputing.co.uk/robo-tx/
#
# This example demonstrates a countdown timer. Buttons 1 and 2 set the minutes 
# and seconds. Button 3 starts/stops timer and cancels alarm. Hold button 3
# to reset to zero.
#
# Robo-Tx API online help: https://help.cohesivecomputing.co.uk/Robo-Tx
#
# All examples are provided as is and at user's own risk.

import threading
import time
from datetime import datetime,timedelta
from pythonnet import load
load("coreclr")
import clr

# Use correct path for your OS platform.
clr.AddReference("./RoboTx/win-x64/RoboTx.Api")
from RoboTx.Api import RobotIO,Input
from RoboTx import *

# Helper method to format time for display.
def to_time_format(seconds):
    return f"{seconds // 60:02}.{str(seconds % 60).zfill(2)}"

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

    countdown_running = False
    alarm_sound_active = False
    timer_value_seconds = 0           
    time_text = to_time_format(int(timer_value_seconds))

    mfs.LedDisplay.Write(time_text)
    target_time = datetime.now() + timedelta(seconds=timer_value_seconds)

    while detectEnter.is_alive():
        # If a button was pressed, an input event will have been generated.
        input_event = mfs.Digital.GetInputEvent()

        # If the alarm is sounding and button 3 pressed/released,
        if alarm_sound_active and input_event == Input.BUTTON_3_RELEASED:
            alarm_sound_active = False
            # Turn off alarm.
            mfs.Trigger.Off()
        elif not countdown_running:
            # If button A1 is pressed or held down, increment the minutes.
            if input_event == Input.BUTTON_1_PRESSED or input_event == Input.BUTTON_1_SUSTAINED:
                timer_value_seconds += 60
            # If button A2 is pressed or held down, increment the seconds.
            elif input_event == Input.BUTTON_2_PRESSED or input_event == Input.BUTTON_2_SUSTAINED:
                timer_value_seconds += 1
            # If button A3 is held down, reset the countdown time.
            elif input_event == Input.BUTTON_3_SUSTAINED:
                timer_value_seconds = 0
        else:
            # Do countdown.
            timer_value_seconds = (target_time - datetime.now()).total_seconds()

            # If the countdown time has lapsed, sound the alarm.
            if timer_value_seconds <= 0:
                countdown_running = False
                alarm_sound_active = True

                # Sound alarm
                mfs.Trigger.RunPattern(
                    50,     # beep for 50 milliseconds
                    50,     # silent for 50 milliseconds
                    4,      # repeat above cycle 4 times
                    3,      # loop 3 times. 0 = indefinite repetition
                    500     # wait 500 milliseconds between loop
                )
        
        # If button A3 is pressed/released, start or stop the countdown.
        if input_event == Input.BUTTON_3_RELEASED:
            if not countdown_running:
                # Re-calculate countdown.
                target_time = datetime.now() + timedelta(seconds=timer_value_seconds)
            countdown_running = not countdown_running and timer_value_seconds > 0

        timer_value_seconds = max(0, min(timer_value_seconds, 3599))

        time_text = to_time_format(int(timer_value_seconds))
        mfs.LedDisplay.Write(time_text)

        # Short pause is necessary before next loop iteration
        time.sleep(0.025)
finally:
    mfs.Close()
    print("Disconnecting.")
