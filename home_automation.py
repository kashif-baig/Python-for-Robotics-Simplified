# Home automation model house example using Robo-Tx framework
# (C) Kashif Baig
# 
# Deploy Robo-Tx firmware to Arduino (see link below). Make sure
# SELECTED_PROFILE is set to PROFILE_HOME_AUTOMATION in file Settings.h
#
# https://github.com/kashif-baig/RoboTx_Firmware
# https://www.cohesivecomputing.co.uk/robo-tx/
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

# Callback function to convert raw analog value to temperature
def raw_to_temperature(value):
    return (value * 500) / 1023

# Callback function to convert raw analog value to percentage
def raw_to_percent(value):
    return 100-((value * 100) / 1023)

# If connecting directly to an Arduino using USB, you can find the serial port using the Arduino IDE.
dtr_enable = False
serial_port = "COM7"
serial_baud_rate = 115200

home = RobotIO(serial_port, serial_baud_rate, dtr_enable)
try:
    home.Connect()
    print("Press Enter to stop program.")

    # Thread to detect Enter key
    detectEnter = threading.Thread(target = input)
    detectEnter.start()

    # -----------------------------------------------------------------------------------------
    # Assign sensor variables
    light_sensor = home.Analog.A2
    rain_sensor = home.Analog.A1
    temperature_sensor = home.Analog.A0

    # Assign digital input variables
    button = home.Digital.IN0

    # Register callback functions to convert raw analog value to meaningful values.
    home.Analog.UseConverter(AnalogConverter(raw_to_percent), light_sensor)
    home.Analog.UseConverter(AnalogConverter(raw_to_temperature), temperature_sensor)

    # Assign switch and motor variables
    indoor_light = home.Switch1
    indoor_dimmable_light = home.Motor2
    front_light = home.Switch2
    relay_switch = home.Switch3
    fan = home.Motor1

    # Assign door and window variables
    window = home.Servo1
    door = home.Servo2
    home.ServoConfig.SetSpeedLimit(10, window, door)
    
    win_open_position = 150
    win_closed_position = 32
    home.ServoConfig.SetAngleLimits(win_closed_position, win_open_position, window)

    door_open_position = 175
    door_closed_position = 92
    home.ServoConfig.SetAngleLimits(door_closed_position, door_open_position, door)

    # Assign display and beeper variables
    lcd_display = home.Display
    beeper = home.Trigger

    # Assign sonar module variable
    sonar = home.Sonar

    # -----------------------------------------------------------------------------------------
    
    # Set door and window to closed position
    door.SetPosition(door_closed_position)
    window.SetPosition(win_closed_position)

    # Show a message on the LCD display
    lcd_display.PrintAt(0, 0, "Welcome to the")
    lcd_display.PrintAt(0, 1, "smart home")

    # Sound the beeper for 250 milliseconds
    #beeper.Pulse(250)

    # Run the fan at 50% for 0.75 seconds
    #fan.DriveForDuration(50, 0.75)

    while detectEnter.is_alive():
        sonar.Ping()

        if button.Value == True:
            
            front_light.OnForDuration(1)
            lcd_display.WakeUp()

            print(datetime.now().time(),
                round(temperature_sensor.Value, 1),
                round(light_sensor.Value, 1),
                rain_sensor.Value,
                sonar.GetDistance())
        #else:
        #    front_light.Off()

        # Check if IR remote button was pressed, and display its code.
        ir_command = home.Digital.GetIRCommand()
        if ir_command.Received and ir_command.ButtonPressed :
            print(ir_command.Code)

        time.sleep(0.05)
finally:
    home.Close()

