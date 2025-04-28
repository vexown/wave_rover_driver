#!/bin/bash

# ****************************************************************************
# UnitTests_Build.sh
#
# Description:
# This script automates the setup and build process for ESP32 unit tests.
# It performs the following steps:
# 1. Checks and installs required system dependencies (Debian-based systems).
# 2. Initializes and updates the ESP-IDF submodule to the correct version.
# 3. Installs the ESP32 toolchain using the ESP-IDF install script.
# 4. Exports necessary ESP-IDF environment variables.
# 5. Copies the ESP-IDF unit test application template if not already present.
# 6. Modifies the CMakeLists.txt to include custom application components.
# 7. Builds the unit tests for the selected target or all targets.
# 8. Logs the build output to unit_tests_build.log.
#
# Usage:
# ./UnitTests_Build.sh
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
DEPS_DIR="../../Dependencies"
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

# Move to the script's directory
cd "$SCRIPT_DIR"

# Copy the unit-test-app template from ESP-IDF if UnitTests dir is empty or doesn't contain CMakeLists.txt
# This prevents overwriting existing user modifications on subsequent runs,
# but ensures the template is present if missing.
if [ ! -f "$SCRIPT_DIR/CMakeLists.txt" ]; then
    echo "Copying unit-test-app template from $ESP_IDF_DIR/tools/unit-test-app..."
    # Copy contents of the source directory into the target directory
    cp -r "$ESP_IDF_DIR/tools/unit-test-app/"* "$SCRIPT_DIR/"
    if [ $? -ne 0 ]; then
        echo "Error: Failed to copy unit-test-app template."
        exit 1
    fi
else
    echo "UnitTests directory already contains CMakeLists.txt, skipping copy."
fi

# Modify the CMakeLists.txt in the copied UnitTests directory
CMAKE_FILE="$SCRIPT_DIR/CMakeLists.txt" # Absolute path

# Check if the modification is already done to avoid unnecessary changes/errors
if ! grep -q 'list(APPEND EXTRA_COMPONENT_DIRS "../Application/components")' "$CMAKE_FILE"; then
    echo "Modifying $CMAKE_FILE to include application components..."

    # Define the search and replace strings - need to escape $ { } for sed literal matching
    SEARCH_LINE='^list(APPEND EXTRA_COMPONENT_DIRS [^)]*)' # Regex to match any such line
    REPLACE_LINE='list(APPEND EXTRA_COMPONENT_DIRS "../Application/components")' # Path relative to CMakeLists.txt location

    # Use sed to perform the replacement in-place, create backup .bak file
    sed -i.bak "s|$SEARCH_LINE|$REPLACE_LINE|g" "$CMAKE_FILE"

    # Check if sed command was successful
    if [ $? -ne 0 ]; then
        echo "Error: Failed to modify $CMAKE_FILE. Check the file content and sed command."
        # Optional: restore backup if needed, otherwise leave .bak file for inspection
        # mv "$CMAKE_FILE.bak" "$CMAKE_FILE"
        exit 1
    else
        echo "$CMAKE_FILE modified successfully."
        # Remove the backup file on success
        rm "$CMAKE_FILE.bak"
    fi
else
     echo "$CMAKE_FILE already configured for application components."
fi

# Find components with a 'test' subdirectory
APP_COMPONENTS_DIR="$SCRIPT_DIR/../Application/components"
TEST_COMPONENTS=()
echo "Searching for testable components in $APP_COMPONENTS_DIR..."
if [ -d "$APP_COMPONENTS_DIR" ]; then
    for component_dir in "$APP_COMPONENTS_DIR"/*/; do
        if [ -d "${component_dir}test" ]; then
            component_name=$(basename "$component_dir")
            TEST_COMPONENTS+=("$component_name")
            echo "Found: $component_name"
        fi
    done
else
    echo "Error: Application components directory not found at $APP_COMPONENTS_DIR"
    exit 1
fi

# Check if any test components were found
if [ ${#TEST_COMPONENTS[@]} -eq 0 ]; then
    echo "No components with a 'test' subdirectory found in $APP_COMPONENTS_DIR."
    # Decide if you want to exit or default to 'all'
    # exit 1
    echo "Defaulting to build all components."
    IDF_TEST_ARG="-T all"
else
    # Present options to the user
    echo "----------------------------------------"
    echo "Available test components:"
    for i in "${!TEST_COMPONENTS[@]}"; do
        echo "$((i+1))) ${TEST_COMPONENTS[$i]}"
    done
    echo "A) All listed components"
    echo "----------------------------------------"

    # Get user input
    read -p "Enter numbers/letter of components to build (space-separated, or 'A' for all): " user_choice

    SELECTED_COMPONENTS_STR=""
    # Parse user input
    if [[ "$user_choice" =~ ^[Aa]$ ]]; then
        echo "Building all listed components."
        SELECTED_COMPONENTS_STR=$(IFS=" "; echo "${TEST_COMPONENTS[*]}")
    else
        read -ra choices <<< "$user_choice"
        for choice in "${choices[@]}"; do
            if [[ "$choice" =~ ^[0-9]+$ ]] && [ "$choice" -ge 1 ] && [ "$choice" -le ${#TEST_COMPONENTS[@]} ]; then
                index=$((choice-1))
                SELECTED_COMPONENTS_STR+="${TEST_COMPONENTS[$index]} "
            else
                echo "Invalid selection: $choice. Please enter numbers from the list or 'A'."
                exit 1
            fi
        done
        # Trim trailing space
        SELECTED_COMPONENTS_STR=$(echo "$SELECTED_COMPONENTS_STR" | sed 's/ *$//')
        echo "Building selected components: $SELECTED_COMPONENTS_STR"
    fi

    if [ -z "$SELECTED_COMPONENTS_STR" ]; then
         echo "No components selected. Exiting."
         exit 1
    fi
    IDF_TEST_ARG="-T \"$SELECTED_COMPONENTS_STR\""
fi

#echo "Running idf.py fullclean..."
#idf.py fullclean

# Define the log file (adjust the path as needed)
LOG_FILE="$SCRIPT_DIR/unit_tests_build.log"

echo "Building unit tests... This may take a while, be patient."
echo "Build command: idf.py $IDF_TEST_ARG build"
echo "Starting unit test build at $(date)" > "$LOG_FILE"
echo "Build command: idf.py $IDF_TEST_ARG build" >> "$LOG_FILE"

# Run the build command for unit tests, pipe output to tee for both terminal and log file
# Use eval to correctly handle the quoted argument string
eval idf.py $IDF_TEST_ARG build 2>&1 | tee -a "$LOG_FILE"
BUILD_RESULT=${PIPESTATUS[0]}  # Capture the exit status of idf.py build

echo "Unit test build finished at $(date), exit code: $BUILD_RESULT" >> "$LOG_FILE"

if [ $BUILD_RESULT -eq 0 ]; then
    echo "=================================================="
    echo "Unit test build succeeded. Log saved to $LOG_FILE"
    echo "You can ignore the warnings about OTA partition overflow, apparently they are normal idk, see partition_table_unit_test_app.csv in esp-idf for details."
else
    echo "Unit test build failed with exit code $BUILD_RESULT. See $LOG_FILE for details."
fi