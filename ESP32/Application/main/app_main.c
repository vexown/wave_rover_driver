/*******************************************************************************/
/*                                INCLUDES                                     */
/*******************************************************************************/

/* C Standard Libraries */
#include <stdio.h>
#include <inttypes.h>

/* ESP-IDF includes */
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_flash_partitions.h"
#include "esp_ota_ops.h"

/* Project includes */
#include "Common.h"
#include "motor_control.h"
#include "oled_display.h"
#include "web_server.h"
#include "NaviLogging.h"
#include "i2c_manager.h"
#include "IMU.h"

/*******************************************************************************/
/*                                 MACROS                                      */
/*******************************************************************************/
#define FW_VERSION "01.10" // Firmware version (MAJOR.MINOR)

#define TASK_APP_MAIN_PERIOD_TICKS pdMS_TO_TICKS(2000)

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
    /********************* Task Initialization ***************************/ 
    /* Initialize the components */
    init_components();

    /* Print system information to the console, OLED, and web server */
    print_system_info();

    /* Variables for the stability check */
    int stable_cycles_count = 0;
    const int required_stable_cycles = 3; // Wait for 3 cycles (3 * TASK_APP_MAIN_PERIOD_TICKS)
    bool app_validated = false;
    bool is_pending_verification = false;
    esp_ota_img_states_t ota_state;

    /* Check the state of the running partition */
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_err_t err = esp_ota_get_state_partition(running, &ota_state);
    if (err == ESP_OK) 
    {
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) 
        {
            LOG_TO_RPI("Firmware is pending verification. Starting stability check.\n");
            is_pending_verification = true;
        }
        else
        {
            LOG_TO_RPI("Firmware is already verified or not pending verification. Current state: %d\n", ota_state);
        }
    }
    else 
    {
        LOG_TO_RPI("Failed to get OTA state for running partition: %s\n", esp_err_to_name(err));
    }

    /********************* Task Loop ********************************/
    while(1) 
    {
        /* If we're running a new OTA updated application, we perform a stability check to make sure it's not crashing
         *      - If the application is stable for a certain number of cycles, we mark it as valid.
         *      - If the app crashes, the bootloader will automatically revert to the previous version. */
        if (is_pending_verification && !app_validated) 
        {
            stable_cycles_count++;
            LOG_TO_RPI("Stability check: cycle %d of %d\n", stable_cycles_count, required_stable_cycles);
            if (stable_cycles_count >= required_stable_cycles) 
            {
                LOG_TO_RPI("Application has been stable, marking as valid.\n");
                esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
                if (err == ESP_OK) 
                {
                    LOG_TO_RPI("App marked as valid. Rollback is now cancelled.\n");
                } 
                else 
                {
                    LOG_TO_RPI("Failed to mark app as valid: %s\n", esp_err_to_name(err));
                }
                app_validated = true; // Once the app is marked as valid, we don't need to perform stability checks anymore
            }
        }
        vTaskDelay(TASK_APP_MAIN_PERIOD_TICKS); // Move into the blocked state allowing other tasks to run
    }
}


/*******************************************************************************/
/*                        STATIC FUNCTION DEFINITIONS                          */
/*******************************************************************************/

static void init_components(void)
{
    /******************************* Motor Control *******************************/
    LOG_TO_RPI("Initializing Motor Control...");
    esp_err_t motor_err = motor_init();
    if (motor_err != ESP_OK)
    {
        LOG_TO_RPI("Motor initialization failed: %s", esp_err_to_name(motor_err)); // Log the error but continue execution (TODO: Decide how to handle failure)
    }
    else
    {
        LOG_TO_RPI("Motor Control Initialized.");
    }

    /******************************* I2C Manager *******************************/
    LOG_TO_RPI("Initializing I2C Manager...");
    esp_err_t i2c_err = i2c_manager_init(I2C_MANAGER_DEFAULT_PORT, I2C_MANAGER_DEFAULT_SDA, I2C_MANAGER_DEFAULT_SCL);
    if (i2c_err != ESP_OK)
    {
        LOG_TO_RPI("I2C Manager initialization failed: %s", esp_err_to_name(i2c_err)); // Log the error but continue execution (TODO: Decide how to handle failure)
    }
    else
    {
        LOG_TO_RPI("I2C Manager Initialized.");
    }

    /******************************* OLED Display *******************************/
    LOG_TO_RPI("Initializing OLED Display...");
    // oled_err is global, so it's updated directly
    oled_err = oled_init();
    if (oled_err != ESP_OK)
    {
        LOG_TO_RPI("OLED initialization failed: %s", esp_err_to_name(oled_err)); // Log the error but continue execution (TODO: Decide how to handle failure)
    }
    else
    {
        LOG_TO_RPI("OLED Display Initialized.");
    }

    /******************************* Web Server *******************************/
    LOG_TO_RPI("Initializing Web Server...");
    esp_err_t web_err = web_server_init();
    if (web_err != ESP_OK)
    {
        LOG_TO_RPI("Web Server initialization failed: %s", esp_err_to_name(web_err)); // Log the error but continue execution (TODO: Decide how to handle failure)
    }
    else
    {
        LOG_TO_RPI("Web Server Initialized.");
    }

    /******************************* NaviLogging *******************************/
    LOG_TO_RPI("Initializing NaviLogging (ESP-NOW receiver)...");
    esp_err_t navi_err = NaviLogging_init();
    if (navi_err != ESP_OK)
    {
        LOG_TO_RPI("NaviLogging initialization failed: %s", esp_err_to_name(navi_err)); // Log the error but continue execution
    }
    else
    {
        LOG_TO_RPI("NaviLogging Initialized.");
    }

    /******************************* IMU Initialization *******************************/
    LOG_TO_RPI("Initializing IMU...");
    esp_err_t imu_err = imu_init();
    if (imu_err != ESP_OK)
    {
        LOG_TO_RPI("IMU initialization failed: %s", esp_err_to_name(imu_err)); // Log the error but continue execution
    }
    else
    {
        LOG_TO_RPI("IMU Initialized.");
    }
}


static void print_system_info(void)
{
    /*** Display a Welcome Message on the OLED display, on the Web Server, and on the Serial Monitor ***/

    esp_chip_info_t chip_info;
    uint32_t flash_size;

    LOG_TO_RPI("Welcome to Moduri Application!\n");

    /* Get chip information */
    esp_chip_info(&chip_info);

    /* Print chip information */
    LOG_TO_RPI("This is %s chip with %d CPU core(s), %s%s%s%s, ",
            CONFIG_IDF_TARGET,
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
            (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
            (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    LOG_TO_RPI("silicon revision v%d.%d, ", major_rev, minor_rev);

    /* Get flash size */
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK)
    {
        LOG_TO_RPI("Get flash size failed");
        /* Consider how to handle this error more robustly if needed */
        flash_size = 0; // Set a default value or handle differently
    }

    /* Print flash size */
    LOG_TO_RPI("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024), (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    LOG_TO_RPI("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

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
            LOG_TO_RPI("OLED refresh failed: %s", esp_err_to_name(refresh_err)); // Log the error but continue execution
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

