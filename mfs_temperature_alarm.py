# Arduino Multi-function board example using Robo-Tx framework
# (C) Kashif Baig
# 
# Deploy Robo-Tx firmware to Arduino (see link below). Make sure
# SELECTED_PROFILE is set to PROFILE_MULTI_FUNC_SHIELD in file Settings.h
#
# https://github.com/kashif-baig/RoboTx_Firmware
# https://www.cohesivecomputing.co.uk/robo-tx/
#
# This example demonstrates a temperature alarm that escalates as the temperature rises.
# Correctly connect a genuine LM35 sensor to Arduino pin A4.
#
# Robo-Tx API online help: https://help.cohesivecomputing.co.uk/Robo-Tx
#
# All examples are provided as is and at user's own risk.

import threading
import time
from datetime import datetime
from pythonnet import load
load("coreclr")
import clr

# Use correct path for your OS platform.
clr.AddReference("./RoboTx/win-x64/RoboTx.Api")
from RoboTx.Api import RobotIO,Input,AnalogConverter
from RoboTx import *

# Helper function to map one range of values to another.
def map_range(value, from_low, from_high, to_low, to_high):
    if value < from_low:
        value = from_low
    elif value > from_high:
        value = from_high

    return int(((value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low) + 0.5)

# Convert raw analog value to temperature
def raw_to_temperature(value):
    return (value * 500) / 1023

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

    # Register a function to convert raw analog value to temperature value. 
    mfs.Analog.UseConverter(AnalogConverter(raw_to_temperature), mfs.Analog.A4)

    # Define temperature thresholds
    alarm_range_min_temp = 22
    alarm_range_max_temp = 28

    min_temp_exceed_time = None
    min_temp_exceeded = False

    alarm_sounding = False

    # Initialize temperature readings buffer.
    temp_buffer = []

    while detectEnter.is_alive():
        # Read the analog value that has been converted to temperature.
        temp = mfs.Analog.A4.Value

        if temp > 0:
            temp_buffer.append(temp)
            # Maintain a buffer of the last 20 readings
            if len(temp_buffer) > 20:
                temp_buffer.pop(0)

        # Calculate average temperature.
        temp_sum = sum(temp_buffer)
        temp_avg = temp_sum / len(temp_buffer) if temp_buffer else 0

        # Check if temperature exceeds minimum threshold
        if temp_avg > alarm_range_min_temp:
            if not min_temp_exceeded:
                min_temp_exceeded = True
                min_temp_exceed_time = datetime.now()
            elif (datetime.now() - min_temp_exceed_time).total_seconds() > 5:
                # Minimum temperature has been exceed for more than 5 seconds.
                if not alarm_sounding:
                    alarm_sounding = True
                    # Start beeper repeating indefinitely: 300ms beep, 4500ms delay.
                    mfs.Trigger.Repeat(300, 4500)
                else:
                    # Set delay between beeps, shortening as temperature rises (from 4500ms to 400ms).
                    mfs.Trigger.SetOffPeriod(map_range(temp_avg, alarm_range_min_temp, alarm_range_max_temp, 4500, 400))
        else:
            if alarm_sounding:
                # Turn off alarm.
                mfs.Trigger.Off()
            min_temp_exceeded = False
            alarm_sounding = False

        # Display the average temperature
        mfs.LedDisplay.Write(f"{temp_avg:.1f}".rjust(5))
        time.sleep(0.25)
finally:
    mfs.Close()

