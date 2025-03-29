# GENERAL INFORMATION - WHAT IS THE WAVE ROVER?
See the Waveshare wiki for the description of the Wave Rover - https://www.waveshare.com/wiki/WAVE_ROVER

## Basic Description
The Waveshare UGV robots utilize both an upper computer and a lower computer. This repository contains the program running on the lower computer, which is typically a ESP32 with **General Driver for Robots**.  

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
1. You need to install **[Arduino IDE](https://www.arduino.cc/en/software)** and **[ESP32 Board](https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide-windows-instructions/)** first.

2. Then, copy `SCServo` folder into `C:\Users\[username]\AppData\Local\Arduino15\libraries\`

3. Install the following libraries using the Arduino IDE **Library Manager**:

- ArduinoJson
- LittleFS
- Adafruit_SSD1306
- INA219_WE
- ESP32Encoder
- PID_v2
- SimpleKalmanFilter
- Adafruit_ICM20X
- Adafruit_ICM20948
- Adafruit_Sensor

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
