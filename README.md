# WAVE ROVER driver
WAVE ROVER driver project, is based on the [ugv_base_general](https://github.com/waveshareteam/ugv_base_general) repository by waveshareteam.

# WHAT IS THE WAVE ROVER?
In short, Wave Rover is a Mobile Robot. It runs on 4 wheel drive, features both lower level board (ESP32) for fundemental rover control as well as higher level upper board (e.g Raspberry Pi) for things like cameras, AI processing and stuff like that. It runs on some LiPo cells with BMS and charging control board "UPS Module"(also from Waveshare). Great thing about this platform is that everything is open source, you can modify it as much as you want.

See the Waveshare wiki for a more in-depth description of the Wave Rover - https://www.waveshare.com/wiki/WAVE_ROVER

## Basic Description
The Waveshare UGV robots utilize both an upper computer and a lower computer. This repository contains the program running on the lower computer, which is typically a ESP32 with **General Driver for Robots**

## Driver Board
Wave Rover utilizes the Waveshare **General Driver for Robots** - you can buy it here (in case you burn it and need a replacement) - https://www.waveshare.com/general-driver-for-robots.htm and you can find plenty of info and tutorials on the wiki page: https://www.waveshare.com/wiki/General_Driver_for_Robots 

### LOWER COMPUTER
The program running on the lower computer is either named [ugv_base_ros](https://github.com/effectsmachine/ugv_base_ros.git) or [ugv_base_general](https://github.com/effectsmachine/ugv_base_general.git) depending on the type of robot driver being used. In the case of WAVE ROVER we are basing the implementation on the ugv_base_general firmware.

**This repo contains the implementation for the ESP-32 lower computer of the WAVE ROVER**. The code is initially based on the original Waveshare repo and then modified by me to fit my needs. See https://github.com/waveshareteam/ugv_base_general/ for the original Waveshare code.

### UPPER COMPUTER

The upper computer communicates with the lower computer (the robot's driver based on ESP32) by sending JSON commands via GPIO UART. The host controller, which employs a [Jetson Orin](https://github.com/waveshareteam/ugv_jetson) or a [Raspberry Pi](https://github.com/waveshareteam/ugv_rpi) based on the type of upper computer being used, handles AI vision and strategy planning, while the sub-controller, utilizing an ESP32, manages motion control and sensor data processing. This setup ensures efficient collaboration and enhanced performance.

### Communication between UPPER Computer (Raspberry Pi) and LOWER Computer (ESP32)

It can be done in many ways but one of the simplest methods is to use UART communication.
When the Pi is inserted directly into the 40-pin GPIO socket (upside down) available on the Lower Computer board it can communicate with ESP32
via pins:
ESP32UE: U0TX (PIN 35, GPIO 1) <-----> Pi: P_RX (PIN 10, GPIO15, UART RX)
ESP32UE: U0RX (PIN 34, GPIO 3) <-----> Pi: P_TX (PIN 8, GPIO 14, UART TX)

Pin numbers taken directly from schematic: 
https://files.waveshare.com/upload/3/37/General_Driver_for_Robots.pdf (REMEMBER THAT PI IS UPSIDE DOWN)

Also for reference Raspberry Pi 4B pinout data:
https://i0.wp.com/randomnerdtutorials.com/wp-content/uploads/2023/03/Raspberry-Pi-Pinout-Random-Nerd-Tutorials.png?quality=100&strip=all&ssl=1

And ESP32UE datasheet:
https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32e_esp32-wroom-32ue_datasheet_en.pdf


## Configure and build the firmware

So origially the Waveshare team provided the firmware for the Lower Computer (ESP32) as an Arduino program.
However, I prefer having more control over the code and therefore I ported the code to ESP-IDF framework.
I mostly reworked the basic functionality to work in ESP-IDF and then started adding my own stuff, ditching the Arduino completely.

I like having scripts for everything. So to build, you just need to run the ./Build script.

It should take care of all dependencies, including apt packages and esp-idf itself. Hopefully it doesnt just "work on my machine".

## License
This project is licensed under the GNU General Public License v3.0, as is the original `ugv_base_general` project

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
