# ****************************************************************************
# CMakeLists.txt
#
# Description:
# This is the top-level CMake configuration file for the ESP-IDF project.
# It defines the project name and integrates with the ESP-IDF build system.
# This file is responsible for:
# 1. Setting the minimum CMake version required
# 2. Including the ESP-IDF build system
# 3. Defining the project name used for build artifacts
#
# The ESP-IDF build system handles all the complex tasks of:
# - Compiling the ESP-IDF components
# - Linking the application with ESP-IDF libraries
# - Creating flashable binary files
# - Providing project-specific build targets
#
# ****************************************************************************

# Specify the minimum CMake version required to build this project
cmake_minimum_required(VERSION 3.16)

# Include the ESP-IDF build system from the IDF_PATH environment variable
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Define the project name - affects output filenames and other identifiers
project(wave_rover_driver)
