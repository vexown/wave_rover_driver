#!/bin/bash

# ****************************************************************************
# Clean.sh
#
# Description:
# This script cleans the build directory to start fresh.
# It performs the following steps:
# 1. Sets up the ESP-IDF environment
# 2. Removes all build artifacts
# 3. Prompts the user to press any key to exit the script
#
# Usage:
#   ./Clean.sh
#
# ****************************************************************************
DEPS_DIR="../Dependencies"
ESP_IDF_DIR="$DEPS_DIR/esp-idf"

cd "$ESP_IDF_DIR"
. ./export.sh
cd ../../ESP32/Application

echo "Cleaning the build directory..."
idf.py fullclean

echo "Clean completed"

# Prompt the user to press any key to exit
echo "Press any key to exit the script."

# Handle user input
read -n 1 -s  # Read a single keypress silently
echo "Exiting the script."