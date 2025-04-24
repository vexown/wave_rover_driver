#!/bin/bash

# ****************************************************************************
# ESP-IDF_Terminal.sh
#
# Description:
# This script opens an interactive terminal with the ESP-IDF environment
# variables and paths properly configured. It performs the following:
# 1. Sets up the ESP-IDF environment by sourcing export.sh
# 2. Opens an interactive shell with ESP-IDF tools in PATH
# 3. Allows running any ESP-IDF commands (like idf.py) directly
#
# This is equivalent to the "Open ESP-IDF Terminal" feature in VS Code
#
# Usage:
#   ./ESP-IDF_Terminal.sh
#
# ****************************************************************************
DEPS_DIR="../Dependencies"
ESP_IDF_DIR="$DEPS_DIR/esp-idf"

# Source the ESP-IDF environment setup script
cd "$ESP_IDF_DIR"
. ./export.sh
cd ../../ESP32/Application

echo "****************************************************************************"
echo "                         ESP-IDF Terminal"
echo ""
echo "The ESP-IDF environment has been set up."
echo "You can now run ESP-IDF commands like 'idf.py build', 'idf.py flash', etc."
echo "Type 'exit' to close this terminal."
echo "****************************************************************************"
echo ""

# Launch an interactive shell with the ESP-IDF environment
exec $SHELL