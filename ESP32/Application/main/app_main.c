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
#include <esp_wifi.h>
#include "Common.h"
#include "motor_control.h"
#include "oled_display.h"
#include "web_server.h"

/*******************************************************************************/
/*                                 MACROS                                      */
/*******************************************************************************/
#define FW_VERSION "01.07" // Firmware version (MAJOR.MINOR)

/*******************************************************************************/
/*                               DATA TYPES                                    */
/*******************************************************************************/

/*******************************************************************************/
/*                        GLOBAL FUNCTION DECLARATIONS                         */
/*******************************************************************************/

/*******************************************************************************/
/*                            GLOBAL VARIABLES                                 */
/*******************************************************************************/
esp_err_t oled_err = ESP_FAIL; 

/*******************************************************************************/
/*                        STATIC FUNCTION DECLARATIONS                         */
/*******************************************************************************/

/**
 * @brief Prints system information to various outputs.
 *
 * @details This function gathers and displays system information, including
 *          ESP32 chip details (model, cores, revision), flash memory size,
 *          and the current firmware version. The information is outputted to
 *          the serial monitor, the OLED display (if initialized and available),
 *          and potentially made available via a web server interface.
 *
 * @param none
 *
 * @return void
 */
static void print_system_info(void);

/**
 * @brief Initializes the components of the application.
 *
 * @details This function initializes the various components of the application,
 *          including motor control, OLED display, and web server. It handles
 *          errors during initialization and logs the status of each component.
 *
 * @param none
 *
 * @return void
 */
static void init_components(void);

/*******************************************************************************/
/*                            STATIC VARIABLES                                 */
/*******************************************************************************/

/*******************************************************************************/
/*                        GLOBAL FUNCTION DEFINITIONS                          */
/*******************************************************************************/

/**
 * @brief Main application entry point.
 * @details This function is called by the ESP-IDF framework after initialization. 
 *          It runs in the context of a default high-priority FreeRTOS main task 
 *          which is created by the ESP-IDF framework and runs your app_main() function.
 *          app_main() can be used to start other FreeRTOS tasks that your
 *          application requires.
 * @param none
 * @return void
 */
void app_main(void)
{
    /* Initialize the components */
    init_components();

    /* Print system information to the console, OLED, and web server */
    print_system_info();

    LOG("Entering idle loop. Connect to WiFi and control via web server.");

    /* Task loop */
    while(1) 
    {
        printf("app_main(), just hanging around...\n");
        vTaskDelay(pdMS_TO_TICKS(1000)); // Move into the blocked state allowing other tasks to run
    }
}


/*******************************************************************************/
/*                        STATIC FUNCTION DEFINITIONS                          */
/*******************************************************************************/

static void init_components(void)
{
    /******************************* Motor Control *******************************/
    LOG("Initializing Motor Control...");
    esp_err_t motor_err = motor_init();
    if (motor_err != ESP_OK)
    {
        LOG("Motor initialization failed: %s", esp_err_to_name(motor_err)); // Log the error but continue execution (TODO: Decide how to handle failure)
    }
    else
    {
        LOG("Motor Control Initialized.");
    }

    /******************************* OLED Display *******************************/
    LOG("Initializing OLED Display...");
    // oled_err is global, so it's updated directly
    oled_err = oled_init();
    if (oled_err != ESP_OK)
    {
        LOG("OLED initialization failed: %s", esp_err_to_name(oled_err)); // Log the error but continue execution (TODO: Decide how to handle failure)
    }
    else
    {
        LOG("OLED Display Initialized.");
    }

    /******************************* Web Server *******************************/
    LOG("Initializing Web Server...");
    esp_err_t web_err = web_server_init();
    if (web_err != ESP_OK)
    {
        LOG("Web Server initialization failed: %s", esp_err_to_name(web_err)); // Log the error but continue execution (TODO: Decide how to handle failure)
    }
    else
    {
        LOG("Web Server Initialized.");
    }
}


static void print_system_info(void)
{
    /*** Display a Welcome Message on the OLED display, on the Web Server, and on the Serial Monitor ***/

    esp_chip_info_t chip_info;
    uint32_t flash_size;

    LOG("Welcome to Moduri Application!\n");

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
        /* Consider how to handle this error more robustly if needed */
        flash_size = 0; // Set a default value or handle differently
    }

    /* Print flash size */
    LOG("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024), (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    LOG("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

    uint8_t mac_addr[6];
    (void)esp_wifi_get_mac(ESP_IF_WIFI_STA, mac_addr); // Get the MAC address of the station interface (for ESP-NOW)
    ESP_LOGI("MAC_INFO", "My MAC Address is: %02X:%02X:%02X:%02X:%02X:%02X", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

    if (oled_err == ESP_OK)
    {
        /* Display welcome message and initial status on OLED */
        oled_clear_buffer();
        oled_write_string(0, "WAVE ROVER");      // Line 0
        char version_str[32];
        snprintf(version_str, sizeof(version_str), "version: %s", FW_VERSION);
        oled_write_string(1, version_str);        // Line 1

        /* Get the current IP address and WiFi connection status */
        char ip_addr_str[16];
        char status_str[20];

        /* Get the IP address (will be "No IP" if not connected yet) */
        /* web_server_get_ip returns ESP_FAIL if not connected, but still provides "No IP" string */
        web_server_get_ip(ip_addr_str, sizeof(ip_addr_str));

        /* Determine initial status string based on current event group state */
        /* This reflects the status *at the time print_system_info is called */
        if (web_server_is_connected()) 
        {
            snprintf(status_str, sizeof(status_str), "WiFi: Online");
        } 
        else 
        {
            snprintf(status_str, sizeof(status_str), "WiFi: Offline");
        }

        oled_write_string(2, status_str); // Line 2
        oled_write_string(3, ip_addr_str); // Line 3

        esp_err_t refresh_err = oled_refresh();
        if (refresh_err != ESP_OK)
        {
            LOG("OLED refresh failed: %s", esp_err_to_name(refresh_err)); // Log the error but continue execution
        }
    }

    /* Buffer to hold the information string */
    char info_buffer[256];

    /* Format the information string */
    /* Note: snprintf is used to format the string safely. The size of the buffer is passed to prevent buffer overflow.
     * This is better way to handle strings than using sprintf, which can lead to buffer overflow if not careful. */
    snprintf(info_buffer, sizeof(info_buffer),
            "Hello from Wave Rover!\n"
            "FW: %s\n"
            "Chip: %s\n"
            "Cores: %d\n"
            "Sillicon Revision: %d.%d\n"
            "Flash: %" PRIu32 "MB %s\n"
            "MAC: %02X:%02X:%02X:%02X:%02X:%02X",
            FW_VERSION,
            CONFIG_IDF_TARGET,
            chip_info.cores,
            major_rev, minor_rev, // Use calculated revisions
            flash_size / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external",
            mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

    /* Print the welcome message on the web server */
    web_server_print(info_buffer);
}

