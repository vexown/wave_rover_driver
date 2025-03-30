# WAVE ROVER driver

This project, WAVE ROVER driver, is based on the [ugv_base_general](https://github.com/waveshareteam/ugv_base_general) repository by waveshareteam.

This project is licensed under the GNU General Public License v3.0, as is the original `ugv_base_general` project. See license information at the end of this readme.

# WHAT IS THE WAVE ROVER?
See the Waveshare wiki for the description of the Wave Rover - https://www.waveshare.com/wiki/WAVE_ROVER

## Basic Description
The Waveshare UGV robots utilize both an upper computer and a lower computer. This repository contains the program running on the lower computer, which is typically a ESP32 with **General Driver for Robots**

## Driver Board
Wave Rover utilizes the Waveshare **General Driver for Robots** - you can buy it here (in case you burn it and need a replacement) - https://www.waveshare.com/general-driver-for-robots.htm and you can find plenty of info and tutorials on the wiki page: https://www.waveshare.com/wiki/General_Driver_for_Robots 

### LOWER COMPUTER
The program running on the lower computer is either named [ugv_base_ros](https://github.com/effectsmachine/ugv_base_ros.git) or [ugv_base_general](https://github.com/effectsmachine/ugv_base_general.git) depending on the type of robot driver being used. In the case of WAVE ROVER we are basing the implementation on the ugv_base_general firmware.

**This repo contains the implementation for the ESP-32 lower computer of the WAVE ROVER**. The code is initially based on the original Waveshare repo and then modified by me to fit my needs. See https://github.com/waveshareteam/ugv_base_general/ for the original Waveshare code.

### UPPER COMPUTER

The upper computer communicates with the lower computer (the robot's driver based on ESP32) by sending JSON commands via GPIO UART. The host controller, which employs a [Jetson Orin](https://github.com/waveshareteam/ugv_jetson) or a [Raspberry Pi](https://github.com/waveshareteam/ugv_rpi) based on the type of upper computer being used, handles AI vision and strategy planning, while the sub-controller, utilizing an ESP32, manages motion control and sensor data processing. This setup ensures efficient collaboration and enhanced performance.

## Features
- Closed-loop Speed Control with PID
- Web App Based on ESP32
- IMU
- OLED Screen
- LED Lights(12V switches) Control
- Control via JSON Commands
- Supports Camera PT
- Supports RoArm-M2
- Control and Communicate via ESP-NOW

## Configure and build the firmware

### Arudino setup

Guide based on https://www.waveshare.com/wiki/How_To_Install_Arduino_IDE

1. Install **[Arduino IDE](https://www.arduino.cc/en/software)** 

2. In Arduino IDE go to File->Preferences and add the following link in the "Additional boards manager URLs": https://dl.espressif.com/dl/package_esp32_index.json

3. Download ESP32 development package from Waveshare: https://files.waveshare.com/wiki/RoArm-M2-S/esp32-2.0.11.zip and unzip it in:
   Windows: `C:\Users\[username]\AppData\Local\Arduino15\packages\`
   Linux: `~\.arudino15\packages\`

(you should end up with a new `esp32` folder in the `packages` directory)

4. Download required libraries from Waveshare: https://files.waveshare.com/upload/a/ae/General-Libraries.zip and unzip all of them to the `libraries` folder in your Sketchbook location (check File->Preferences in Arduino IDE to find it).

**Note, there is a few missing libraries/dependencies in this package which you need to install/update via the Arduino Library Manager**
Here is the list:
- ArduinoJson
- Adafruit_SSD1306
- ESP32Encoder

(you should end up with bunch of new library folders in the `libraries` directory)

4. Open the **WaveRoverDriver.ino** file in Arduino to load the project into the workspace

5. Select the ESP32 Dev Module as the Board: Tools->Board->Search for esp32 in the Board Manager and Install it->Go back to Tools->Board and Select the **ESP32 Dev Module** under the esp32 family

6. Connect the driver board to the PC via the USB-C port that is in the middle of the board (there is 2).

7. Select the newly appeared COM port in Arduino: Tools->Port

8. Verify & Upload (Flash) the new firmware


### Basic Use
See the Wiki: https://www.waveshare.com/wiki/WAVE_ROVER

## License
ugv_base_general for the Waveshare UGV Robots: an open source robotics platform for the Robots based on **General Driver**.
Copyright (C) 2024 [Waveshare](https://www.waveshare.com/)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/gpl-3.0.txt>.
