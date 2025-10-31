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
#include "comms_uart.h"
#include "motor_control.h"
#include "oled_display.h"
#include "web_server.h"
#include "NaviLogging.h"
#include "i2c_manager.h"
#include "IMU.h"
#include "esp_now_comm.h"
#include "esp_now_comm_callbacks.h"
#include "wifi_manager.h"

/*******************************************************************************/
/*                                 MACROS                                      */
/*******************************************************************************/
#define FW_VERSION "01.16" // Firmware version (MAJOR.MINOR)

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
 * @brief Callback for WiFi disconnection - stops motors for safety
 * 
 * @return ESP_OK if motors stopped successfully, ESP_FAIL otherwise
 */
static esp_err_t on_wifi_disconnect(void);

/**
 * @brief Callback for WiFi status display updates - updates OLED
 * 
 * @param status_line Status message to display
 * @param detail_line Detail message to display (e.g., IP address)
 */
static void on_wifi_status_update(const char *status_line, const char *detail_line);

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
    /******************************* NVS Flash *******************************/
    LOG_TO_RPI("Initializing NVS Flash...");
    /* NVS (Non-Volatile Storage) is a partition in the ESP32's flash memory
     * used to store key-value pairs persistently across reboots. It's commonly
     * used for storing configuration data, calibration values, or WiFi credentials.
     * The ESP-IDF WiFi library uses it here, potentially to store station credentials,
     * allowing the device to reconnect automatically after a restart.
     * This is apparently required by Wi-Fi stack for storing credentials and PHY calibration data. */
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) 
    {
        /* NVS partition was truncated/corrupted - erase it and reinitialize */
        LOG_TO_RPI("NVS partition corrupted/out of date, erasing...");
        nvs_err = nvs_flash_erase();
        if (nvs_err != ESP_OK) 
        {
            LOG_TO_RPI("NVS erase failed: %s", esp_err_to_name(nvs_err));
        }
        else
        {
            nvs_err = nvs_flash_init();
        }
    }
    if (nvs_err != ESP_OK)
    {
        LOG_TO_RPI("NVS initialization failed: %s", esp_err_to_name(nvs_err));
    }
    else
    {
        LOG_TO_RPI("NVS Flash Initialized.");
    }

    /******************************* UART Communication *******************************/
    LOG_TO_RPI("Initializing UART Communication...");
    esp_err_t uart_err = comms_uart_init();
    if (uart_err != ESP_OK)
    {
        LOG_TO_RPI("UART initialization failed: %s", esp_err_to_name(uart_err)); // Log the error but continue execution (TODO: Decide how to handle failure)
    }
    else
    {
        LOG_TO_RPI("UART Communication Initialized.");
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

    /******************************* TCP/IP & Event Loop *******************************/
    LOG_TO_RPI("Initializing network stack...");
    /* Initialize the TCP/IP stack (ESP-IDF uses lwIP for this) */
    /* ESP-NETIF (Network Interface) library provides an abstraction layer for the application on top of the TCP/IP stack. 
     * ESP-IDF currently implements ESP-NETIF for the lwIP TCP/IP stack only.
     * See documentation for details: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/network/esp_netif.html 
     **/
    esp_err_t netif_err = esp_netif_init();
    if (netif_err != ESP_OK)
    {
        LOG_TO_RPI("Network interface initialization failed: %s", esp_err_to_name(netif_err));
    }
    
    /* Initialize and start the default system event loop. */
    /* This loop is used by various ESP-IDF components (e.g., WiFi, TCP/IP)
     * to post events and allows application code to register handlers
     * that react to these events asynchronously. It facilitates communication
     * between different parts of the system.
     **/
    esp_err_t event_loop_err = esp_event_loop_create_default();
    if (event_loop_err != ESP_OK && event_loop_err != ESP_ERR_NO_MEM)
    {
        LOG_TO_RPI("Event loop creation failed: %s", esp_err_to_name(event_loop_err));
    }
    /* Create a default WiFi station interface */
    /* The API creates esp_netif object with default WiFi station config,
     * attaches the netif to wifi and registers wifi handlers to the default event loop.
     * The return value is a pointer to the created esp_netif instance (not used here).
     * (default event loop needs to be created prior to calling this API)
     **/
    (void)esp_netif_create_default_wifi_sta(); 

    LOG_TO_RPI("Network stack initialized.");

    /******************************* WiFi Initialization *******************************/
    LOG_TO_RPI("Initializing WiFi...");
    wifi_manager_callbacks_t wifi_callbacks = {
        .on_disconnect = on_wifi_disconnect,
        .on_status_update = on_wifi_status_update
    };
    
    esp_err_t wifi_err = wifi_manager_init(&wifi_callbacks);
    if (wifi_err != ESP_OK)
    {
        LOG_TO_RPI("WiFi initialization failed: %s", esp_err_to_name(wifi_err)); // Log the error but continue execution
    }
    else
    {
        LOG_TO_RPI("WiFi Initialized.");
    }

    /******************************* ESP-NOW Initialization *******************************/
    /* IMPORTANT: ESP-NOW initializes immediately after WiFi driver starts.
     * ESP-NOW does NOT require WiFi to be connected to an AP.
     * It only requires the WiFi radio to be initialized and running in STA mode.
     * 
     * This MUST be before web_server_init() because web_server_init() blocks waiting
     * for WiFi AP connection, which should happen in parallel with ESP-NOW operation.
     */
    LOG_TO_RPI("Initializing ESP-NOW Communication...");
    esp_now_comm_config_t config = 
    {
        .on_recv = on_data_recv_callback,    /* Called when data is received */
        .on_send = on_data_send_callback,    /* Called after send attempt completes */
        .mac_addr = {0}                      /* WiFi radio is initialized, MAC will be retrieved */
    };

    esp_err_t esp_now_err = esp_now_comm_init(&config);
    if (esp_now_err != ESP_OK)
    {
        LOG_TO_RPI("ESP-NOW initialization failed: %s", esp_err_to_name(esp_now_err));
        // Note: Don't call web_server_print here yet since web server isn't initialized
    }
    else
    {
        LOG_TO_RPI("ESP-NOW Communication Initialized.");
        
        /* Retrieve and log the WiFi channel that ESP-NOW will use */
        uint8_t primary_channel = 0;
        wifi_second_chan_t secondary_channel = WIFI_SECOND_CHAN_NONE;
        esp_err_t channel_err = wifi_manager_get_channel(&primary_channel, &secondary_channel);
        
        if (channel_err == ESP_OK)
        {
            LOG_TO_RPI("ESP-NOW will use WiFi Channel: %d (use this channel on other devices)", primary_channel);
            
            if (secondary_channel != WIFI_SECOND_CHAN_NONE)
            {
                LOG_TO_RPI("Secondary Channel for HT40: %d", secondary_channel);
            }
        }
        else
        {
            LOG_TO_RPI("Warning: Could not retrieve WiFi channel for ESP-NOW");
        }
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
    LOG_TO_RPI("My MAC Address is: %02X:%02X:%02X:%02X:%02X:%02X", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

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

static esp_err_t on_wifi_disconnect(void)
{
    /* Immediately stop the motors on disconnection (to prevent leaving them running in a
     * potentially uncontrolled state when the device is disconnected from WiFi) */
    return motor_stop();
}

static void on_wifi_status_update(const char *status_line, const char *detail_line)
{
    /* Update OLED display with WiFi status if OLED is available */
    if (oled_err == ESP_OK) 
    {
        oled_write_string(2, status_line);
        oled_write_string(3, detail_line);
        oled_refresh();
    }
}