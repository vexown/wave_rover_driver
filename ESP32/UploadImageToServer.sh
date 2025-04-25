#!/bin/bash

# ****************************************************************************
# UploadImageToServer.sh
#
# Description:
# This script copies the compiled firmware binary from the build output directory
# to the web server directory used for Over-The-Air (OTA) updates.
# It requires sudo privileges to copy the file to the web server directory
# and to restart the Apache service.
# ****************************************************************************

set -euo pipefail

# Define source and destination file paths
SOURCE_FILE="./Application/build/wave_rover_driver.bin" # the app binary is in this location by default after build
DEST_FILE="/var/www/html/firmware_wave_rover_driver.bin" # the location where the apache2 server serves files

# Check if the source file exists
if [ ! -f "$SOURCE_FILE" ]; then
    echo "Error: Source file not found at $SOURCE_FILE" >&2
    exit 1
fi

# Copy the file to the OTA server hosted via the apache2 HTTPS server
echo "Copying $SOURCE_FILE to $DEST_FILE..."
if sudo cp "$SOURCE_FILE" "$DEST_FILE"; then
    echo "File copied successfully."

    # Optional: Set appropriate permissions/ownership for the web server
    # echo "Setting permissions for $DEST_FILE..."
    # sudo chown www-data:www-data "$DEST_FILE"
    # sudo chmod 644 "$DEST_FILE"

    # Restart Apache if copy was successful
    echo "Restarting apache2 service..."
    if sudo systemctl restart apache2; then
        echo "Apache2 restarted successfully."
    else
        echo "Error: Failed to restart apache2." >&2
        exit 1
    fi
    exit 0
else
    echo "Error: Failed to copy file." >&2
    exit 1
fi