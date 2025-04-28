#============================================================================
# CMakeLists.txt for ESP32 Unit Testing with Unity Framework
#
# This CMakeLists.txt file is specifically configured for building and running
# unit tests for an ESP32 project component using the Unity testing framework,
# integrated with the ESP-IDF build system.
#
# This configuration facilitates the creation of a dedicated test application
# that can be flashed and run on an ESP32 target (or simulator) to execute the
# defined unit tests and report the results via the serial console.
#============================================================================

idf_component_register(SRCS "test_oled_display.c"
                        INCLUDE_DIRS "."
                        REQUIRES unity oled_display)