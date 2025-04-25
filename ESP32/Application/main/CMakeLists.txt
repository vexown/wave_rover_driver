# ****************************************************************************
# CMakeLists.txt (Component Level)
#
# Description:
# This is the component-level CMake configuration file for the main application.
# It registers the application's source files with the ESP-IDF build system.
# This file is responsible for:
# 1. Specifying which source files (.c) belong to this component
# 2. Defining any include directories needed for header files
# 3. Optionally specifying component dependencies (not used here)
#
# All components in ESP-IDF projects require their own CMakeLists.txt file
# to be recognized by the build system.
#
# ****************************************************************************

# Register this component with ESP-IDF build system:
# - SRCS: Lists all source files to be compiled
# - INCLUDE_DIRS: Specifies directories for header files (empty in this case)
idf_component_register(SRCS "app_main.c"
                       INCLUDE_DIRS ""
                       REQUIRES spi_flash 
                                esp_common
                                Common
                                motor_control
                                oled_display
                                web_server    
                                esp_http_server
                                esp_https_ota  
                      )

# Embed the certificate file
target_add_binary_data(${COMPONENT_TARGET} "certs/apache-selfsigned.crt" TEXT EMBED_IN_RODATA)


