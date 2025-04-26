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

/*******************************************************************************/
/*                               DATA TYPES                                    */
/*******************************************************************************/

/*******************************************************************************/
/*                        GLOBAL FUNCTION DECLARATIONS                         */
/*******************************************************************************/

/*******************************************************************************/
/*                        STATIC FUNCTION DECLARATIONS                         */
/*******************************************************************************/

/*******************************************************************************/
/*                            STATIC VARIABLES                                 */
/*******************************************************************************/

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
        oled_write_string(1, "FW v00.02"); // Line 1, version number (MAJOR.MINOR, 2 digits each, e.g., 00.01)
        oled_write_string(2, "Initializing..."); // Line 2
        oled_err = oled_refresh();
        if (oled_err != ESP_OK) {
            LOG("OLED refresh failed: %s", esp_err_to_name(oled_err));
        }
    }

    LOG("Initializing Web Server...");
    esp_err_t web_err = web_server_init(); // Initialize WiFi STA and HTTP Server
    if (web_err != ESP_OK) {
        LOG("Web Server initialization failed: %s", esp_err_to_name(web_err));
        // Decide how to handle failure, maybe halt or restart
        // For now, continue execution but log the error
    } 
    else 
    {
        LOG("Web Server Initialized.");
    }

    LOG("Entering idle loop. Connect to WiFi AP and control via web server.");

    /* Task loop */
    while(1) 
    {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Keep the task alive but idle
    }
}