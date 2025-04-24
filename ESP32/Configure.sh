#!/bin/bash

# ****************************************************************************
# Configure.sh
#
# Description:
# This script automates the process of configuring an ESP32 project.
# It performs the following steps:
# 1. Sets up the ESP-IDF environment
# 2. Launches the ESP-IDF configuration menu (menuconfig)
# 3. Saves configuration changes to sdkconfig
# 4. Prompts the user to press any key to exit the script
#
# Usage:
#   ./Configure.sh
#
# ****************************************************************************
DEPS_DIR="../Dependencies"
ESP_IDF_DIR="$DEPS_DIR/esp-idf"

cd "$ESP_IDF_DIR"
. ./export.sh
cd ../../ESP32/Application

echo "Launching ESP-IDF menuconfig..."
idf.py menuconfig

echo "Configuration completed"

# Prompt the user to press any key to exit
echo "Press any key to exit the script."

# Handle user input
read -n 1 -s  # Read a single keypress silently
echo "Exiting the script."