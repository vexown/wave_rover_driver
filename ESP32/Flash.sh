#!/bin/bash

# ****************************************************************************
# Flash.sh
#
# Description:
# This script automates the process of flashing a built project to an ESP32 device.
# It performs the following steps:
# 1. Sets up the ESP-IDF environment
# 2. Scans for available serial ports (/dev/ttyUSB*)
# 3. Prompts the user to select a port
# 4. Connects to the selected ESP32 device
# 5. Flashes the built firmware to the device
# 6. Prompts the user to press any key to exit the script
#
# Usage:
#   ./Flash.sh
#
# ****************************************************************************
DEPS_DIR="../Dependencies"
ESP_IDF_DIR="$DEPS_DIR/esp-idf"

# Scan for available serial ports
echo "Scanning for available serial ports..."
ports=($(ls /dev/ttyUSB* 2> /dev/null))

if [ ${#ports[@]} -eq 0 ]; then
    echo "Error: No ESP32 devices found on /dev/ttyUSB* ports."
    echo "Please ensure your device is connected and drivers are installed."
    exit 1
fi

# Prompt user to select a port
echo "Available ports:"
for i in "${!ports[@]}"; do
    echo "  $((i+1))) ${ports[$i]}"
done

read -p "Select the port number to flash: " port_choice

# Validate input
if ! [[ "$port_choice" =~ ^[0-9]+$ ]] || [ "$port_choice" -lt 1 ] || [ "$port_choice" -gt ${#ports[@]} ]; then
    echo "Error: Invalid selection."
    exit 1
fi

selected_port=${ports[$((port_choice-1))]}
echo "Selected port: $selected_port"

# Set up ESP-IDF environment and flash
cd "$ESP_IDF_DIR"
. ./export.sh
cd ../../ESP32/Application

echo "Flashing to $selected_port..."
idf.py -p "$selected_port" flash

echo "Done"

# Prompt the user to press any key to exit
echo "Press any key to exit the script."

# Handle user input
read -n 1 -s  # Read a single keypress silently
echo "Exiting the script."