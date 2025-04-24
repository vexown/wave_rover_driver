#!/bin/bash

# ****************************************************************************
# InitTarget.sh
#
# Description:
# This script sets the target SoC for the project. It needs to be run only once.
#
# Usage:
#   ./InitTarget.sh
#
# ****************************************************************************
DEPS_DIR="../Dependencies"
ESP_IDF_DIR="$DEPS_DIR/esp-idf"

cd "$ESP_IDF_DIR"
. ./export.sh
cd ../../ESP32/Application

echo "---------------------------------------------------"
echo "Available targets:"
echo "---------------------------------------------------"
idf.py set-target --help
echo "---------------------------------------------------"

# Set the target to ESP32 (default)
echo "---------------------------------------------------"
echo "Setting the target to ESP32..."
echo "---------------------------------------------------"
idf.py set-target esp32

echo "---------------------------------------------------"
echo "Operation completed"
echo "---------------------------------------------------"

# Prompt the user to press any key to exit
echo "Press any key to exit the script."

# Handle user input
read -n 1 -s  # Read a single keypress silently
echo "Exiting the script."