/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

/**
 * ****************************************************************************
 * app_main.c
 *
 * Description:
 * This is the main application file for the Moduri application.
 * It serves as the entry point and handles system initialization.
 * 
 * The application performs the following tasks:
 * 1. Prints a welcome message
 * 2. Displays detailed chip information (CPU cores, features, revision)
 * 3. Shows flash memory size and type
 * 4. Reports minimum free heap size
 * 5. Initializes the SW components
 *
 * This file implements the main control flow for the Moduri application.
 * ****************************************************************************
 */

/*******************************************************************************/
/*                                INCLUDES                                     */
/*******************************************************************************/
#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h" 
#include "esp_http_client.h"
#include "esp_https_ota.h" 
#include "Common.h"
#include "motor_control.h"
#include "oled_display.h"
#include "web_server.h"

/*******************************************************************************/
/*                                 MACROS                                      */
/*******************************************************************************/
#define MOTOR_TEST_PWM 150 // Define a PWM value for testing (0-255)
#define MOTOR_TEST_DELAY_MS 2000 // Delay in milliseconds for each movement
#define MOTOR_STOP_DELAY_MS 1000 // Delay in milliseconds for stops

#define FIRMWARE_UPGRADE_URL "https://192.168.1.194/firmware_wave_rover_driver.bin"

/*******************************************************************************/
/*                               DATA TYPES                                    */
/*******************************************************************************/

/*******************************************************************************/
/*                        GLOBAL FUNCTION DECLARATIONS                         */
/*******************************************************************************/
/* External symbol declarations for the OTA server certificate */
extern const uint8_t server_cert_pem_start[] asm("_binary_apache_selfsigned_crt_start");
extern const uint8_t server_cert_pem_end[]   asm("_binary_apache_selfsigned_crt_end");

/*******************************************************************************/
/*                        STATIC FUNCTION DECLARATIONS                         */
/*******************************************************************************/

/*******************************************************************************/
/*                            STATIC VARIABLES                                 */
/*******************************************************************************/
static const char *OTA_TAG = "OTA_UPDATE";

/*******************************************************************************/
/*                            GLOBAL VARIABLES                                 */
/*******************************************************************************/
esp_err_t oled_err = ESP_FAIL; 

/*******************************************************************************/
/*                        STATIC FUNCTION DEFINITIONS                          */
/*******************************************************************************/

/*******************************************************************************/
/*                        GLOBAL FUNCTION DEFINITIONS                          */
/*******************************************************************************/

void perform_ota_update() 
{
    ESP_LOGI(OTA_TAG, "Starting OTA update from URL: %s", FIRMWARE_UPGRADE_URL);

    esp_http_client_config_t http_config = 
    {
        .url = FIRMWARE_UPGRADE_URL,
        .cert_pem = (char *)server_cert_pem_start,
        .timeout_ms = 10000, // Set a specific timeout in milliseconds (e.g., 10 seconds)
        .keep_alive_enable = true,
    };

    esp_https_ota_config_t ota_config = 
    {
        .http_config = &http_config,
    };

    esp_err_t ret = esp_https_ota(&ota_config);
    if (ret == ESP_OK) 
    {
        ESP_LOGI(OTA_TAG, "OTA Update Successful, Rebooting...");
        esp_restart();
    } 
    else 
    {
        ESP_LOGE(OTA_TAG, "OTA Update Failed: %s", esp_err_to_name(ret));
        // Handle failure (e.g., update OLED, log error)
        if (oled_err == ESP_OK) 
        { // Check if OLED is available
             oled_write_string(3, "OTA Failed!");
             oled_refresh();
        }
    }
}

/**
 * ****************************************************************************
 * Function: app_main
 * 
 * Description: Main application entry point. This function is called by the 
 *              ESP-IDF framework after initialization. It runs in the context
 *              of a default high-priority FreeRTOS main task which is created 
 *              by the ESP-IDF framework and runs your app_main() function.
 *              app_main() can be used to start other FreeRTOS tasks that your
 *              application requires.
 * 
 * Parameters:
 *   - none
 * 
 * Returns: void
 * ****************************************************************************
 */
void app_main(void)
{
    LOG("Welcome to Moduri Application!\n");

    /******************************* Platform Information *******************************/
    esp_chip_info_t chip_info;
    uint32_t flash_size;

    /* Get chip information */
    esp_chip_info(&chip_info);

    /* Print chip information */
    LOG("This is %s chip with %d CPU core(s), %s%s%s%s, ",
            CONFIG_IDF_TARGET,
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
            (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
            (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    LOG("silicon revision v%d.%d, ", major_rev, minor_rev);

    /* Get flash size */
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) 
    {
        LOG("Get flash size failed");
        return;
    }

    /* Print flash size */
    LOG("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
        (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    LOG("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

    /******************************* Initialize Components *******************************/
    LOG("Initializing Motor Control...");
    esp_err_t motor_err = motor_init();
    if (motor_err != ESP_OK) {
        LOG("Motor initialization failed: %s", esp_err_to_name(motor_err));
        // Decide how to handle failure, maybe halt or restart
        return; // Stop further execution if motor init fails
    }
    LOG("Motor Control Initialized.");

    LOG("Initializing OLED Display...");
    oled_err = oled_init();
    if (oled_err != ESP_OK) {
        LOG("OLED initialization failed: %s", esp_err_to_name(oled_err));
        // Continue execution even if OLED fails, but log the error
    } else {
        LOG("OLED Display Initialized.");
        // Display Welcome Message on OLED
        oled_clear_buffer();
        oled_write_string(0, "WAVE ROVER"); // Line 0
        oled_write_string(1, "ESP-IDF vX.Y"); // Line 1 (Replace X.Y with actual version if desired)
        oled_write_string(2, "Initializing..."); // Line 2
        oled_err = oled_refresh();
        if (oled_err != ESP_OK) {
            LOG("OLED refresh failed: %s", esp_err_to_name(oled_err));
        }
    }

    LOG("Initializing Web Server...");
    esp_err_t web_err = web_server_init(); // Initialize WiFi AP and HTTP Server
    if (web_err != ESP_OK) {
        LOG("Web Server initialization failed: %s", esp_err_to_name(web_err));
        // Decide how to handle failure, maybe halt or restart
        // For now, continue execution but log the error
    } else {
        LOG("Web Server Initialized.");
        // Update OLED with AP SSID and IP address
        if (oled_err == ESP_OK) { // Check if OLED init was successful
            // Use the WIFI_AP_SSID defined in web_server.c
            // Note: Max 16 chars fit well on 128x32 with 8x8 font
            char line2_buf[24]; // Increased size from 20 to 24
            // Use the actual string instead of the macro here
            snprintf(line2_buf, sizeof(line2_buf), "AP: %s", "WAVE_ROVER_ESP32"); 
            // Truncate if too long for display (optional, depends on desired look)
            // line2_buf[16] = '\0'; // Example truncation if needed

            oled_write_string(2, line2_buf); // Update line 2 with SSID
            oled_write_string(3, "IP: 192.168.4.1"); // Default AP IP on line 3
            oled_refresh();
       }
    }

    LOG("Entering idle loop. Connect to WiFi AP and control via web server.");

    /* Task loop */
    while(1) 
    {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Keep the task alive but idle
    }
}