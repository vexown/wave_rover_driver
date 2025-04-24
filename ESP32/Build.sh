#!/bin/bash

# ****************************************************************************
# Build.sh
#
# Description:
# This script automates the process of building a project using ESP-IDF.
# It performs the following steps:
# 1. Checks for and installs required system packages if missing
# 2. Ensures ESP-IDF dependency is initialized as a git submodule
# 3. Verifies ESP-IDF is at the correct version
# 4. Installs the ESP32 toolchain
# 5. Sets the ESP32 as the target
# 6. Builds the project using idf.py and checks its exit code
# 7. Copies output files to the output directory (TODO)
#
# Usage:
#   ./Build.sh
#
# ****************************************************************************

# If a command in a pipeline fails, the whole pipeline fails
set -o pipefail

# Store the script's directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Check for required system packages
echo "Checking for required system packages..."

# Check if we're on a system with apt
if command -v apt-get &> /dev/null; then
    MISSING_PACKAGES=""

    # List of required packages
    REQUIRED_PACKAGES=(
        "git"
        "wget"
        "flex"
        "bison"
        "gperf"
        "python3"
        "python3-pip"
        "python3-venv"
        "cmake"
        "ninja-build"
        "ccache"
        "libffi-dev"
        "libssl-dev"
        "dfu-util"
        "libusb-1.0-0"
    )

    # Check each required package
    for pkg in "${REQUIRED_PACKAGES[@]}"; do
        if ! dpkg-query -W -f='${Status}' "$pkg" 2>/dev/null | grep -q "ok installed"; then
            echo "Package $pkg is not installed."
            MISSING_PACKAGES="$MISSING_PACKAGES $pkg"
        fi
    done

    # Install missing packages if any
    if [ ! -z "$MISSING_PACKAGES" ]; then
        echo "Installing missing packages:$MISSING_PACKAGES"
        sudo apt-get update
        sudo apt-get install -y $MISSING_PACKAGES
        echo "Required packages installed."
    else
        echo "All required packages are already installed."
    fi
else
    echo "Warning: This script cannot check for packages on non-Debian based systems."
    echo "Please ensure you have these packages installed:"
    echo "- git"
    echo "- wget"
    echo "- flex"
    echo "- bison"
    echo "- gperf"
    echo "- python3"
    echo "- python3-pip"
    echo "- python3-venv"
    echo "- cmake"
    echo "- ninja-build"
    echo "- ccache"
    echo "- libffi-dev"
    echo "- libssl-dev"
    echo "- dfu-util"
    echo "- libusb-1.0-0"
fi

# Check if git submodules are initialized
echo "Checking git submodules..."
# Create Dependencies directory if it doesn't exist
DEPS_DIR="../Dependencies"
if [ ! -d "$DEPS_DIR" ]; then
    mkdir -p "$DEPS_DIR"
    echo "Created Dependencies directory"
fi

# Initialize all git submodules if not already done
if [ ! -d "$DEPS_DIR/esp-idf/.git" ]; then
    echo "Initializing git submodules..."
    git submodule update --init --recursive "$DEPS_DIR/esp-idf"
fi

# Verify ESP-IDF is at the correct commit
ESP_IDF_DIR="$DEPS_DIR/esp-idf"
ESP_IDF_TAG="v5.4"
ACTUAL_TAG=$(cd "$ESP_IDF_DIR" && git describe --tags --exact-match 2>/dev/null || echo "unknown")
if [ "$ACTUAL_TAG" != "$ESP_IDF_TAG" ]; then
    echo "ESP-IDF is not at the expected version. Updating..."
    (cd "$ESP_IDF_DIR" && git fetch && git checkout "$ESP_IDF_TAG")
fi

# Install the ESP32 toolchain
echo "Installing ESP32 toolchain..."
cd "$ESP_IDF_DIR"
./install.sh esp32

# Make the idf.py script available in the PATH
. ./export.sh

# Return to the Application directory
cd ../../ESP32/Application

# Define the log file (adjust the path as needed)
LOG_FILE="../build.log"

echo "Building project... This may take a while, be patient."
echo "Starting build at $(date)" > "$LOG_FILE"

# Run the build command, pipe output to tee for both terminal and log file
idf.py build 2>&1 | tee -a "$LOG_FILE"
BUILD_RESULT=${PIPESTATUS[0]}  # Capture the exit status of idf.py build

echo "Build finished at $(date), exit code: $BUILD_RESULT" >> "$LOG_FILE"

if [ $BUILD_RESULT -eq 0 ]; then
    echo "Build succeeded. Log saved to $LOG_FILE"
else
    echo "Build failed with exit code $BUILD_RESULT. See $LOG_FILE for details."
fi

