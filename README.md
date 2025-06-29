# Python for Robotics Simplified

A hands-on roadmap to kick-off your robotics journey by combining Python, Arduino and the Robo-Tx framework. The coding projects here show why Python’s clear syntax and cross-platform nature make it perfect for learning robotics concepts, how affordable Arduino boards provide the hardware foundation, and how [Robo-Tx firmware](https://github.com/kashif-baig/RoboTx_Firmware) plus a [.NET library](https://github.com/kashif-baig/RoboTx.Api-Solution) bridges them into a seamless development experience.

## Features
- Control up to **2 DC motors**, **4 servos**, **8 analog sensors** and **5 digital inputs**
- Support for **IR remote**, **I²C colour sensor**, **sonar module**
- Drive **4 digital switch outputs** (LEDs, relays, solenoids)
- Write to **4-digit LED display**, **I²C 16×2 LCD** and **active beeper**
- Digital pulse counting on analog pin A2 for advanced timing/flow measurements

## Key Components
- **Microcontroller**: Arduino Uno with L298 motor expansion board (see image below), running Robo-Tx firmware to control I/O services over serial
- **Sensors**: Devices (light, temperature, proximity, etc.) that feed data into your code
- **Actuators**: Motors, relays, LEDs and beepers that let your robot move and signal
- **Power Supply**: Lightweight, regulated source suited to your motors and electronics to keep mobile robots agile

![Arduino Uno motor expansion board image.](https://www.cohesivecomputing.co.uk/cms/wp-content/uploads/2025/04/daokai_L298_motor_shield.jpg "Arduino Uno motor expansion board.")

## Requirements
- A Computer connected (by USB or Bluetooth) to Arduino for running Python code
- [Arduino IDE](https://www.arduino.cc/en/software) (to upload [Robo-Tx firmware](https://github.com/kashif-baig/RoboTx_Firmware))
- [.NET framework](https://dotnet.microsoft.com/en-us/download) (needed for [Robo-Tx library](https://help.cohesivecomputing.co.uk/Robo-Tx))
- [Python 3.x](https://www.python.org/downloads/) + `pip`
- [pythonnet](https://pypi.org/project/pythonnet/) package (to interface with .NET-based Robo-Tx library)
- [VSCode](https://code.visualstudio.com/download) (to write and run Python code)

## Getting Started
1. Flash the [Robo-Tx firmware](https://github.com/kashif-baig/RoboTx_Firmware) onto your Arduino via the Arduino IDE. Make sure to set SELECTED_PROFILE to an appropriate profile in Settings.h
2. In your Python environment:
   ```pip install pythonnet```
3. Import and initialize the Robo-Tx library in your Python script:
```
from pythonnet import load
load("coreclr")
import clr

# Use correct path for your OS platform.
clr.AddReference("./RoboTx/win-x64/RoboTx.Api")
from RoboTx.Api import RobotIO,Input
from RoboTx import *

dtr_enable = False
serial_port = "COM3"
serial_baud_rate = 115200

car = RobotIO(serial_port, serial_baud_rate, dtr_enable)
```
4. Write code to read sensors or drive motors directly from Python.
```
try:
   print("Connecting ...")
   car.Connect()
   print("OK.")

   # optionally assign reference variables to motors and sensors
   motorL = car.Motor1
   motorR = car.Motor2
   lightSensor = car.Analog.A5

   # add your code in a while loop

finally:
   car.Close()
   print("Disconnecting.")
```
To find out more, visit [Python for Robotics Simplified](https://www.cohesivecomputing.co.uk/python-robotics-simplified/)