/******************************************************************************
 * @file web_server.c
 * @brief Template component for ESP-IDF projects
 * 
 ******************************************************************************/

/*******************************************************************************/
/*                                 INCLUDES                                    */
/*******************************************************************************/
/*    Include headers required for the definitions/implementation in *this*    */
/* source file. This typically includes this module's own header("template.h") */
/*      and any headers needed for function bodies, static variables, etc.     */
/*******************************************************************************/
/* C Standard Libraries */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* ESP-IDF Includes */
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h" 
#include "freertos/semphr.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "esp_mac.h"
#include "esp_check.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include <lwip/sockets.h>

/* Project Includes */
#include "web_server.h"
#include "motor_control.h"
#include "oled_display.h"
#include "WiFi_Credentials.h" // WiFi credentials (not committed to the repo, you must provide it locally)
#include "HTML_page.h"

/*******************************************************************************/
/*                                  MACROS                                     */
/*******************************************************************************/
/** Tag for logging (used in ESP_LOGI, ESP_LOGE, etc.) Example usage: 
 *  ESP_LOGI(TAG, "Log message which will be appended to the tag"); */
#define TAG "WEB_SERVER"

/* Max retry attempts for connection */
#define WIFI_MAXIMUM_RETRY 5

/* WiFi Event Group Definitions */
/* BITX masks are defined in esp_bit_defs.h and simplify pointing to a specific bit in the event group */
#define WIFI_CONNECTED_BIT BIT0 // We have connected to the AP specified in the WiFi credentials
#define WIFI_FAIL_BIT      BIT1 // We have failed to connect to the AP after max retries

/* OTA HTTPS server URL to the firmware binary. For the firmware binary to be
 * placed there, use UploadImageToServer.sh script. (build the firmware first) */
#define FIRMWARE_UPGRADE_URL "https://192.168.1.194/firmware_wave_rover_driver.bin"

/* Motor control parameters */
#define MOTOR_CONTROL_PWM 255 //TODO - make this configurable on the web page?

/* Max length for the message to be displayed on the web page */
#define WEBSERVER_PRINT_MAX_LEN 512 // in bytes

/*******************************************************************************/
/*                                DATA TYPES                                   */
/*******************************************************************************/

/*******************************************************************************/
/*                     GLOBAL FUNCTION DECLARATIONS                            */
/*******************************************************************************/
/*  for function defined in some other .c files, to be used here. Use extern.  */
/*******************************************************************************/

/*******************************************************************************/
/*                     GLOBAL VARIABLES DECLARATIONS                           */
/*******************************************************************************/
/*  for variables defined in some other .c files, to be used here. Use extern. */
/*******************************************************************************/
/* Provides access to the global oled_err status variable, which indicates
 * whether the OLED display is initialized successfully or not. */
extern esp_err_t oled_err;

/* External symbol declarations for the OTA server certificate */
extern const uint8_t server_cert_pem_start[] asm("_binary_apache_selfsigned_crt_start");
extern const uint8_t server_cert_pem_end[]   asm("_binary_apache_selfsigned_crt_end");

/*******************************************************************************/
/*                     GLOBAL VARIABLES DEFINITIONS                            */
/*******************************************************************************/
/*    for variables defined in this .c file, to be used in other .c files.     */
/*******************************************************************************/

/*******************************************************************************/
/*                     STATIC FUNCTION DECLARATIONS                            */
/*******************************************************************************/

/**
 * @brief Initiates the Over-The-Air (OTA) update process.
 *
 * @details This function manages the sequence of operations necessary for updating the device's firmware via OTA.
 * It is intended for internal use, often triggered by external events like a web server request.
 * Be aware that a successful firmware update initiated by this function may result in the device rebooting.
 *
 * @param void
 * @return void
 */
static void perform_ota_update(void);

/**
 * @brief Event handler for WiFi events.
 *
 * @details This function is registered to handle events generated by the WiFi driver,
 * such as station start and disconnection. It manages connection retries and
 * updates the WiFi event group status (WIFI_CONNECTED_BIT, WIFI_FAIL_BIT).
 *
 * @param arg User data pointer passed during registration (not used here).
 * @param event_base The base ID of the event (e.g., WIFI_EVENT).
 * @param event_id The specific ID of the event (e.g., WIFI_EVENT_STA_START).
 * @param event_data Data associated with the event (e.g., disconnection reason).
 * @return void
 */
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);

/**
 * @brief Event handler for IP stack events.
 *
 * @details This function is registered to handle events from the TCP/IP stack,
 * specifically when the station interface gets an IP address (IP_EVENT_STA_GOT_IP).
 * It updates the WiFi event group, stores the obtained IP address, and updates
 * the OLED display if available.
 *
 * @param arg User data pointer passed during registration (not used here).
 * @param event_base The base ID of the event (e.g., IP_EVENT).
 * @param event_id The specific ID of the event (e.g., IP_EVENT_STA_GOT_IP).
 * @param event_data Data associated with the event (contains IP information).
 * @return void
 */
static void ip_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);

/**
 * @brief Initializes the ESP32 WiFi in Station (STA) mode.
 *
 * @details Configures and starts the WiFi driver to connect to a predefined
 * Access Point (AP) using credentials from WiFi_Credentials.h. It sets up
 * event handlers for WiFi and IP events and waits for a connection or failure.
 * Updates OLED display with connection status.
 *
 * @param void
 * @return esp_err_t ESP_OK on successful connection, ESP_FAIL otherwise.
 */
static esp_err_t wifi_init_sta(void);

/**
 * @brief HTTP GET handler for the root ("/") URI.
 *
 * @details Sends the main HTML page (defined in HTML_page.h) as a response
 * to GET requests on the root path.
 *
 * @param req Pointer to the HTTP request structure.
 * @return esp_err_t ESP_OK on success, or an error code if sending fails.
 */
static esp_err_t root_get_handler(httpd_req_t *req);

/**
 * @brief HTTP GET handler for the "/control" URI.
 *
 * @details Parses the 'dir' query parameter from the request URL (e.g., /control?dir=forward)
 * and calls the corresponding motor control function. Sends a simple text response.
 *
 * @param req Pointer to the HTTP request structure.
 * @return esp_err_t ESP_OK on success, or an error code if processing fails.
 */
static esp_err_t control_get_handler(httpd_req_t *req);

/**
 * @brief HTTP GET handler for the "/ota" URI.
 *
 * @details Initiates the Over-The-Air (OTA) update process by calling
 * `perform_ota_update`. Sends a response indicating the process has started.
 * Note: The device may reboot during the OTA process.
 *
 * @param req Pointer to the HTTP request structure.
 * @return esp_err_t ESP_OK on success.
 */
static esp_err_t ota_get_handler(httpd_req_t *req);

/**
 * @brief HTTP GET handler for the "/print" URI.
 *
 * @details Sends the current message stored internally.
 *
 * @param req Pointer to the HTTP request structure.
 * @return esp_err_t ESP_OK on success.
 */
static esp_err_t print_get_handler(httpd_req_t *req);

/**
 * @brief Starts the HTTP web server.
 *
 * @details Initializes the HTTP server with default configuration and registers
 * the URI handlers for "/", "/control", and "/ota".
 *
 * @param void
 * @return esp_err_t ESP_OK on success, or an error code if the server fails to start.
 */
static esp_err_t start_webserver(void);

/*******************************************************************************/
/*                             STATIC VARIABLES                                */
/*******************************************************************************/
/* FreeRTOS Event Group for WiFi events */
/* For meaning of each bit, see the WiFi Even Group macros */
static EventGroupHandle_t WiFi_EventGroup;

/* HTTP server handle */
static httpd_handle_t server = NULL;

/* Connection retry counter */
static int s_retry_num = 0;

/* Variable to store the assigned IP address after connection */
static char STA_IP_Addr_String[16] = "0.0.0.0";

/* Buffer to store the latest message */
static char web_server_print_buffer[WEBSERVER_PRINT_MAX_LEN] = "Initializing...";

/* Sequence Number Handling */
static uint32_t lastProcessedSequenceNumber = 0; // Track the last processed sequence number
static SemaphoreHandle_t sequenceMutex = NULL; // Mutex to protect sequence number access

/**
 * @brief URI Definitions
 *
 * A Uniform Resource Identifier (URI) is a sequence of characters that uniquely
 * identifies a resource, typically over a network like the internet. It's a
 * fundamental concept in web technologies. URLs (Uniform Resource Locators)
 * are the most common type of URI.
 *
 * In the context of this web server, URIs represent the specific paths
 * (e.g., "/", "/status", "/config") that clients can request. The definitions
 * below map these URI paths to corresponding handler functions. When the
 * server receives an HTTP request matching one of these URIs, the associated
 * handler function is executed to process the request and generate a response.
 */
static const httpd_uri_t uri_root = 
{
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = root_get_handler,
    .user_ctx  = NULL
};
static const httpd_uri_t uri_control = 
{
    .uri       = "/control",
    .method    = HTTP_GET,
    .handler   = control_get_handler,
    .user_ctx  = NULL
};
static const httpd_uri_t uri_ota = 
{
    .uri       = "/ota",
    .method    = HTTP_GET,
    .handler   = ota_get_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t uri_status =
{
    .uri       = "/print",
    .method    = HTTP_GET,
    .handler   = print_get_handler,
    .user_ctx  = NULL
};

/*******************************************************************************/
/*                     GLOBAL FUNCTION DEFINITIONS                             */
/*******************************************************************************/

esp_err_t web_server_init(void) 
{
    esp_err_t ret = ESP_OK;

    sequenceMutex = xSemaphoreCreateMutex();
    if (sequenceMutex == NULL) 
    {
        ESP_LOGE(TAG, "Failed to create sequence mutex");
        return ESP_FAIL;
    }

    /* #01 - Initialize NVS Flash */
    /* NVS (Non-Volatile Storage) is a partition in the ESP32's flash memory
    * used to store key-value pairs persistently across reboots. It's commonly
    * used for storing configuration data, calibration values, or WiFi credentials.
    * The ESP-IDF WiFi library uses it here, potentially to store station credentials,
    * allowing the device to reconnect automatically after a restart.
    * This is apparently required by Wi-Fi stack for storing credentials and PHY calibration data. */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) 
    {
        /* Try clearing the NVS partition and re-initializing */
        ESP_LOGI(TAG, "NVS Flash init failed, erasing and re-initializing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    /* Check if NVS initialization was successful. If not, log the error and return. */
    ESP_RETURN_ON_ERROR(ret, TAG, "NVS Flash init failed");

    /* #02 - Initialize WiFi in Station mode (STA). This means the ESP32 will connect to an existing WiFi network. */
    ESP_LOGI(TAG, "Initializing WiFi Station...");
    ret = wifi_init_sta();
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "WiFi STA initialization failed!");
        return ret;
    }

    /* #03 - Start the web server only if WiFi connected successfully */
    ESP_LOGI(TAG, "Starting Web Server...");
    ret = start_webserver();
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "Web Server start failed!");
        return ret;
    }

    ESP_LOGI(TAG, "Web Server started successfully on IP: %s", STA_IP_Addr_String);

    return ret; // This should always be ESP_OK, errors would have been caught earlier
}

void web_server_print(const char *message)
{
    const char *message_to_add;
    const char *error_msg = "Error: NULL message received";

    /* If the message is NULL, add a message indicating the error. */
    message_to_add = (message == NULL) ? error_msg : message;

    /* Check how much space is left in the buffer. strlen() returns the length of the string, 
       ending with a null terminator so it should tell us the length of the actual string
       content in the buffer, not the buffer size itself. */
    size_t current_len = strlen(web_server_print_buffer);
    size_t message_to_add_len = strlen(message_to_add);
    size_t remaining_space = WEBSERVER_PRINT_MAX_LEN - current_len;

    const char *prefix = "";
    size_t prefix_len = 0;

    /* Static variable to keep track of the message sequence number */
    static uint32_t message_sequence_number = 0;
    message_sequence_number++; // Increment for each new message

    /* Buffer to hold the formatted prefix */
    char prefix_buf[20]; // Large enough for "\n[Msg 4294967295] "

    /* Determine the prefix based on the current length of the buffer */
    if (current_len > 0)
    {
        /* Format the prefix string with the sequence number */
        snprintf(prefix_buf, sizeof(prefix_buf), "\n[Msg %lu] ", message_sequence_number);
        prefix = prefix_buf; // Point to the formatted prefix
        prefix_len = strlen(prefix);
    }
    else
    {
        /* For the very first message, just add the sequence number without a newline */
        snprintf(prefix_buf, sizeof(prefix_buf), "[Msg %lu] ", message_sequence_number);
        prefix = prefix_buf; // Point to the formatted prefix
        prefix_len = strlen(prefix);
    }

    /* Calculate the total space needed for the prefix, message, and null terminator */
    size_t required_space = prefix_len + message_to_add_len + 1; // +1 for null terminator

    /* Check if there's enough space for the prefix and the new message */
    if (remaining_space >= required_space)
    {
        /* Append the prefix and the message using snprintf for safety */
        snprintf(web_server_print_buffer + current_len, remaining_space, "%s%s", prefix, message_to_add);
    }
    else
    {
        /* Not enough space. Clear the buffer and add the new message. */
        ESP_LOGW(TAG, "Web server print buffer full. Clearing and adding new message.");
        /* Check if the new message itself fits in the buffer */
        if (message_to_add_len + 1 > WEBSERVER_PRINT_MAX_LEN)
        {
            ESP_LOGW(TAG, "New message too long for buffer, truncating.");
            /* Copy as much as possible, ensuring null termination */
            strncpy(web_server_print_buffer, message_to_add, WEBSERVER_PRINT_MAX_LEN - 1);
            web_server_print_buffer[WEBSERVER_PRINT_MAX_LEN - 1] = '\0';
        }
        else
        {
            /* Copy the new message to the beginning of the cleared buffer */
            strcpy(web_server_print_buffer, message_to_add);
        }
    }

    /* Ensure null termination as a final safeguard, although snprintf/strcpy/strncpy should handle it. */
    web_server_print_buffer[WEBSERVER_PRINT_MAX_LEN - 1] = '\0';
}

bool web_server_is_connected(void)
{
    if (WiFi_EventGroup == NULL) 
    {
        return false; // Event group not initialized
    }

    /* Check if the bit indicating connection is set in the event group */
    EventBits_t bits = xEventGroupGetBits(WiFi_EventGroup);
    return (bits & WIFI_CONNECTED_BIT) != 0;
}

esp_err_t web_server_get_ip(char *ip_buffer, size_t buffer_size)
{
    if (ip_buffer == NULL || buffer_size == 0) 
    {
        return ESP_ERR_INVALID_ARG;
    }

    /* Based on the connection status, either provide the IP address or a value indicating disconnection */
    if (web_server_is_connected()) 
    {
        strncpy(ip_buffer, STA_IP_Addr_String, buffer_size - 1);
        ip_buffer[buffer_size - 1] = '\0'; // Ensure null termination

        return ESP_OK;
    } 
    else 
    {
        strncpy(ip_buffer, "No IP", buffer_size - 1);
        ip_buffer[buffer_size - 1] = '\0';

        return ESP_FAIL;
    }
}

/*******************************************************************************/
/*                     STATIC FUNCTION DEFINITIONS                             */
/*******************************************************************************/

static void perform_ota_update()
{
    ESP_LOGI(TAG, "Starting OTA update from URL: %s", FIRMWARE_UPGRADE_URL);

    /* Display OTA update status on OLED if available */
    if (oled_err == ESP_OK) 
    {
        oled_clear_buffer();
        oled_write_string(0, "WAVE ROVER");
        oled_write_string(1, "OTA Update...");
        oled_write_string(2, "Downloading...");
        oled_write_string(3, ""); // Clear line 3
        oled_refresh();
    }

    /* #01 - Define the HTTP client configuration */
    esp_http_client_config_t http_config =
    {
        .url = FIRMWARE_UPGRADE_URL, // URL to the firmware binary (hosted on our HTTPS OTA server, currently using Apache hosted on my PC)
        .cert_pem = (char *)server_cert_pem_start, // Server certificate for HTTPS connection (crucial for TLS operations)
        .timeout_ms = 10000, // Set a specific timeout in milliseconds for the network operations (10s)
        .keep_alive_enable = true, // Enable keep-alive for the HTTP connection
    };

    /* #02 - Set up OTA configuration with the previously defined HTTP client config */
    esp_https_ota_config_t ota_config =
    {
        .http_config = &http_config,
    };

    /* #03 - Initialize and perform the OTA update */
    /* This function allocates HTTPS OTA Firmware upgrade context, establishes HTTPS connection, reads image data from HTTP stream 
     * and writes it to OTA partition and finishes HTTPS OTA Firmware upgrade operation.
     * This API handles the entire OTA operation, so if this API is being used then no other APIs from `esp_https_ota` component should be called.
     * If more information and control is needed during the HTTPS OTA process, then one can use `esp_https_ota_begin` and subsequent APIs. 
     * If this API returns successfully, esp_restart() must be called to boot from the new firmware image.
     **/
    esp_err_t ret = esp_https_ota(&ota_config);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "OTA Update Successful, Rebooting...");
        /* Display "OTA Success!" briefly before reboot */
        if (oled_err == ESP_OK) 
        {
            oled_write_string(2, "OTA Success!");
            oled_write_string(3, "Rebooting...");
            oled_refresh();
            vTaskDelay(pdMS_TO_TICKS(500)); // Short delay to show message
        }

        /* Restart the device to boot from the new firmware */
        esp_restart();
    }
    else
    {
        ESP_LOGE(TAG, "OTA Update Failed: %s", esp_err_to_name(ret));
        /* Display "OTA Failed!" message on OLED if available along with error code */
        if (oled_err == ESP_OK)
        {
            char err_buf[20]; // Buffer to hold error message
            snprintf(err_buf, sizeof(err_buf), "Err: %s", esp_err_to_name(ret));
            err_buf[sizeof(err_buf)-1] = '\0'; // Ensure null termination

            oled_write_string(2, "OTA Failed!");
            oled_write_string(3, err_buf);
            oled_refresh();
        }
    }
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    /* Suppress unused parameter warnings */
    (void)arg;
    (void)event_data;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) 
    {
        /* Clear connected bit on start attempt */
        if (WiFi_EventGroup != NULL) 
        {
            xEventGroupClearBits(WiFi_EventGroup, WIFI_CONNECTED_BIT);
        }
        /* Connect only after the WIFI_EVENT_STA_START event is received, which indicates
         * that the WiFi driver is ready to connect to an Access Point (AP). */
        esp_wifi_connect();
        ESP_LOGI(TAG, "WIFI_EVENT_STA_START: Connecting...");
    } 
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) 
    {
        /* Immediately stop the motors on disconnection (to prevent leaving them running in a
         * potentially uncontrolled state when the device is disconnected from WiFi) */
        if(motor_stop() != ESP_OK) 
        {
            ESP_LOGE(TAG, "Failed to stop motors on WiFi disconnection");
            /* We must stop the motors by all means, so if we didn't manage to stop them
             * using the stop function, we cause a reset to start in a safe state */
            esp_restart();
        }

        /* Clear connected bit on disconnect */
        if (WiFi_EventGroup != NULL) 
        {
            xEventGroupClearBits(WiFi_EventGroup, WIFI_CONNECTED_BIT);
        }
        /* In case of disconnection, attempt to connect again for maximum of WIFI_MAXIMUM_RETRY times */
        if (s_retry_num < WIFI_MAXIMUM_RETRY) 
        {
            esp_wifi_connect();

            s_retry_num++;
            ESP_LOGI(TAG, "Retry connection to the AP (%d/%d)", s_retry_num, WIFI_MAXIMUM_RETRY);
        } 
        else 
        {
            /* After all retires failed, set the event group bit to indicate failure */
            if (WiFi_EventGroup != NULL) 
            {
                xEventGroupSetBits(WiFi_EventGroup, WIFI_FAIL_BIT);
            }
            ESP_LOGE(TAG, "Connect to the AP failed after %d retries", WIFI_MAXIMUM_RETRY);

            /* Update OLED display with failure message */
            if (oled_err == ESP_OK) 
            {
                oled_write_string(2, "WiFi Failed!");
                oled_write_string(3, ""); // Clear IP line
                oled_refresh();
            }
        }
    } /* add handling of other WiFi event cases if needed */
    
}

static void ip_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    /* Suppress unused parameter warnings */
    (void)arg;

    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) 
    {
        /* Cast event_data to the appropriate type */
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;

        /* Log the obtained IP address */
        ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));

        /* Reset retry counter on successful connection */
        s_retry_num = 0;

        /* Convert IP address to string format and store it in STA_IP_Addr_String */
        snprintf(STA_IP_Addr_String, sizeof(STA_IP_Addr_String), IPSTR, IP2STR(&event->ip_info.ip));

        /* Set the WiFi event group bit to indicate successful connection and clear the fail bit */
        if (WiFi_EventGroup != NULL) 
        {
            xEventGroupClearBits(WiFi_EventGroup, WIFI_FAIL_BIT); // Clear potential previous failure
            xEventGroupSetBits(WiFi_EventGroup, WIFI_CONNECTED_BIT);
        }
        /* Update OLED display with connection status and IP address */
        if (oled_err == ESP_OK) 
        {
            /* Add "IP:" prefix to the IP address */
            char line3_buf[20];
            snprintf(line3_buf, sizeof(line3_buf), "IP:%s", STA_IP_Addr_String);

            oled_write_string(2, "WiFi Connected"); // Display status on line 2
            oled_write_string(3, line3_buf); // Display IP on line 3
            oled_refresh();
        }
    } /* add handling of other IP event cases if needed */
}

static esp_err_t wifi_init_sta(void)
{
    /* Initialize the WiFi event group which will be used to signal connection status */
    WiFi_EventGroup = xEventGroupCreate();
    if (WiFi_EventGroup == NULL) 
    {
        ESP_LOGE(TAG, "Failed to create WiFi event group");
        return ESP_FAIL;
    }

    /* #01 - Initialize the TCP/IP stack (ESP-IDF uses lwIP for this) */
    /* ESP-NETIF (Network Interface) library provides an abstraction layer for the application on top of the TCP/IP stack. 
     * ESP-IDF currently implements ESP-NETIF for the lwIP TCP/IP stack only.
     * See documentation for details: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/network/esp_netif.html 
     **/
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "esp_netif_init failed");

    /* #02 - Initialize and start the default system event loop. */
    /* This loop is used by various ESP-IDF components (e.g., WiFi, TCP/IP)
     * to post events and allows application code to register handlers
     * that react to these events asynchronously. It facilitates communication
     * between different parts of the system.
     **/
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "esp_event_loop_create_default failed");

    /* #03 - Create a default WiFi station interface */
    /* The API creates esp_netif object with default WiFi station config,
     * attaches the netif to wifi and registers wifi handlers to the default event loop.
     * The return value is a pointer to the created esp_netif instance (not used here).
     * (default event loop needs to be created prior to calling this API)
     **/
    (void)esp_netif_create_default_wifi_sta(); 

    /* #04 - Get the default WiFi initialization config and use it to initialize the WiFi driver */
    /* The esp_wifi_init() function initializes the WiFi driver with the provided configuration.
     * The default configuration is used here, which is suitable for most applications.
     * It can be customized if needed by modifying the wifi_init_config_t structure.
     **/    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&cfg), TAG, "esp_wifi_init failed");

    /* #05 - Register the event handlers for WiFi and IP events */
    /* The event handlers are registered to handle specific events related to WiFi and IP.
     * The wifi_event_handler() function handles WiFi events (e.g., connection, disconnection),
     * while the ip_event_handler() function handles IP events (e.g., obtaining an IP address).
     * The event handlers are registered with the default event loop created earlier.
     **/
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(WIFI_EVENT,
                                                            ESP_EVENT_ANY_ID,    // Any event related to WiFi
                                                            &wifi_event_handler, // Function to handle the registered event
                                                            NULL,                // No argument passed to the handler
                                                            &instance_any_id), TAG, "Register WIFI_EVENT failed");
    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(IP_EVENT,
                                                            IP_EVENT_STA_GOT_IP, // Event when the station gets an IP address
                                                            &ip_event_handler,   // Function to handle the registered event
                                                            NULL,                // No argument passed to the handler
                                                            &instance_got_ip), TAG, "Register IP_EVENT failed");

    /* #06 - Define the WiFi STA mode configuration */
    /* This config is different from the wifi_init_config_t used before, which is for the driver initialization. 
     * This one is specifically for configuring the parameters of either STA or AP mode (STA+AP mode is also possible).
     **/
    wifi_config_t wifi_config = 
    {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            /* Authmode threshold defaults to WPA2 PSK. */
            .threshold.authmode = WIFI_AUTH_WPA2_PSK, // Or adjust as needed (WIFI_AUTH_WPA_WPA2_PSK, etc.)
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH, // Enable WPA3 support if needed
        },
    };

    /* #07 - Finally, using the defined STA configuration, set the WiFi mode and start the WiFi driver */
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "esp_wifi_set_mode failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &wifi_config), TAG, "esp_wifi_set_config failed");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "esp_wifi_start failed");

    /* #08 - Connection should now be triggered by the wifi_event_handler via esp_wifi_connect() once the WIFI_EVENT_STA_START
     * event is received. The connection process will be handled in the event handler.
     * The handler will also set the bits in the WiFi_EventGroup to indicate success or failure.
     **/
    ESP_LOGI(TAG, "WiFi initialized in STA mode, attempting to connect to SSID: %s", WIFI_SSID);

    /* #09 - Wait for the connection to complete or fail */
    /* The xEventGroupWaitBits() function blocks the calling task until one of the specified bits is set in the event group.
     * In this case, it waits for either the WIFI_CONNECTED_BIT or WIFI_FAIL_BIT to be set.
     * The pdFALSE flag indicates that the bits should not be cleared on exit, and portMAX_DELAY means wait indefinitely.
     **/
    EventBits_t bits = xEventGroupWaitBits(WiFi_EventGroup,
                                            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                            pdFALSE,        // Don't clear bits on exit
                                            pdFALSE,        // Wait for EITHER bit
                                            portMAX_DELAY); // Wait indefinitely

    if (bits & WIFI_CONNECTED_BIT) 
    {
        ESP_LOGI(TAG, "Connected to ap SSID:%s", WIFI_SSID);
        return ESP_OK;
    } 
    else if (bits & WIFI_FAIL_BIT) 
    {
        ESP_LOGE(TAG, "Failed to connect to SSID:%s", WIFI_SSID);
        return ESP_FAIL;
    } 
    else 
    {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
        return ESP_FAIL;
    }
}

static esp_err_t start_webserver(void)
{
    if(sequenceMutex == NULL) 
    {
        ESP_LOGE(TAG, "Sequence mutex not initialized (should be created in web_server_init)");
        return ESP_FAIL;
    }

    /* #01 - Initialize the HTTP config structure with default values */
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    /* Enable Least Recently Used (LRU) connection purge.
     * When the maximum number of connections is reached,
     * the server will automatically close the oldest idle connection
     * to allow a new connection, instead of rejecting it. */
    config.lru_purge_enable = true;

    /* #02 - Start the HTTP server using the defined configuration */
    /* httpd_start creates an instance of HTTP server, allocate memory/resources for it 
     * depending upon the specified configuration and outputs a handle to the server 
     * instance. The server has both, a listening socket (TCP) for HTTP traffic, 
     * and a control socket (UDP) for control signals, which are selected in a round robin
     * fashion in the server task loop. TCP traffic is parsed as HTTP requests and, depending on the 
     * requested URI, user registered handlers are invoked which are supposed to send back HTTP response packets.
     * See details at: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/protocols/esp_http_server.html
     * Note - the 'd' in httpd ESP-IDF stands for "daemon", which is a essentialy a background process that handles requests. */
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    esp_err_t ret = httpd_start(&server, &config);
    if (ret == ESP_OK) 
    {
        
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &uri_root);
        httpd_register_uri_handler(server, &uri_control);
        httpd_register_uri_handler(server, &uri_ota);
        httpd_register_uri_handler(server, &uri_status);

        return ESP_OK;
    }
    else 
    {
        ESP_LOGE(TAG, "Failed to start server: %s", esp_err_to_name(ret));
        return ret;
    }
}

static esp_err_t root_get_handler(httpd_req_t *req)
{
    esp_err_t ret;

    /* Set the response content type to HTML */
    ret = httpd_resp_set_type(req, "text/html");
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed to set response type: %s", esp_err_to_name(ret));
        /* Attempt to send a 500 Internal Server Error if possible */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to set response type");
        return ret; // Return the specific error code
    }

    /* Send the HTML page content */
    ret = httpd_resp_send(req, HTML_PAGE, HTTPD_RESP_USE_STRLEN); // Use strlen for const char*
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed to send response body: %s", esp_err_to_name(ret));
        /* If sending the body fails, the connection might be closed or in an error state.
           No further response might be possible, but we log the error. */
        return ret; // Return the specific error code
    }

    ESP_LOGI(TAG, "Root page served successfully");
    return ESP_OK; // Indicate success
}

static esp_err_t control_get_handler(httpd_req_t *req)
{
    char* buf = NULL;               // Buffer for URL query string
    size_t query_len;               // Length of the URL query string
    char dir[10] = {0};             // Buffer for direction parameter
    char seq_str[12] = {0};         // Buffer for sequence number string (large enough for uint32_t)
    uint32_t receivedSeq = 0;       // Received sequence number
    bool process_command = false;   // Flag to decide if motor command should be executed

    /* Get the length of the URL query string */
    query_len = httpd_req_get_url_query_len(req) + 1; // +1 for null terminator

    /* Check if the query is not empty (not counting null terminator that we added) */
    if (query_len > 1) 
    {
        /* Allocate memory for the query string for processing */
        buf = malloc(query_len);
        if (buf == NULL) 
        {
            ESP_LOGE(TAG, "Failed to allocate memory for query string");
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Memory allocation failed");
            return ESP_ERR_NO_MEM; // Return memory error
        }

        /* Copy the URL query string into the buffer */
        if(httpd_req_get_url_query_str(req, buf, query_len) == ESP_OK) 
        {
            ESP_LOGI(TAG, "Found URL query => %s", buf);
            
            /* Parse the query string to extract the 'dir' parameter value */
            if (httpd_query_key_value(buf, "dir", dir, sizeof(dir)) == ESP_OK) 
            {
                ESP_LOGI(TAG, "Got parameter dir=%s", dir);
            } 
            else 
            {
                ESP_LOGW(TAG, "Parameter 'dir' not found in query");
                /* No motor control command to process, but we still need to check for 'seq' 
                 * because we need to keep track of the sequence number for future commands. */
            }

            /* Parse the query string to extract the 'seq' parameter value */
            if (httpd_query_key_value(buf, "seq", seq_str, sizeof(seq_str)) == ESP_OK)
            {
                /* Convert the sequence number string to an unsigned long */
                char *endptr; // This will point to the first character AFTER the number. If no number is found, it will point to seq_str.
                receivedSeq = strtoul(seq_str, &endptr, 10);

                /* Check if the conversion was successful and if the entire string was processed */
                /* - if *endptr is NOT a null terminator it means that the string had non-numeric characters after the number
                 * - if endptr == seq_str it means that no conversion was performed (i.e., the string was not a valid number) */
                if (*endptr != '\0' || endptr == seq_str)
                {
                    ESP_LOGW(TAG, "Invalid sequence number format: %s", seq_str);
                } 
                else 
                {
                    ESP_LOGI(TAG, "Got parameter dir=%s, seq=%lu", dir, receivedSeq);

                    /* Critical Section: Check if the received sequence number is greater than the last processed one */
                    if (xSemaphoreTake(sequenceMutex, portMAX_DELAY) == pdTRUE)
                    {
                        if (receivedSeq > lastProcessedSequenceNumber)
                        {
                            ESP_LOGD(TAG, "Processing Seq %lu (Last was %lu)", receivedSeq, lastProcessedSequenceNumber);
                            lastProcessedSequenceNumber = receivedSeq;
                            process_command = true; // Order is correct, process the command
                        }
                        else
                        {
                            ESP_LOGW(TAG, "Ignoring out-of-order/duplicate Seq %lu (Last processed: %lu)", receivedSeq, lastProcessedSequenceNumber);
                            process_command = false;
                        }
                        xSemaphoreGive(sequenceMutex);
                    } 
                    else 
                    {
                        ESP_LOGE(TAG, "Failed to take sequence mutex!");
                        process_command = false; // Safety: don't process if mutex fails
                    }
                    /* End of Critical Section */
                }
            }
            else
            {
                ESP_LOGW(TAG, "Parameter 'seq' not found - ignoring command");
                process_command = false; // Require sequence number
            }
        } 
        else 
        {
            ESP_LOGE(TAG, "Failed to get URL query string");
        }

        /* Free the allocated memory for the query string. Since buf is a local variable we don't need to set it to NULL  
         * after freeing it. The pointer will be invalidated anyway. */
        free(buf);
    } 
    else 
    {
        ESP_LOGI(TAG, "No URL query found");
    }

    /* Execute the motor control command but only if the sequence number is valid and the command is recognized (valid direction) */
    if (process_command && strlen(dir) > 0)
    {
        if (strcmp(dir, "forward") == 0)        { motor_move_forward(MOTOR_CONTROL_PWM); }
        else if (strcmp(dir, "backward") == 0)  { motor_move_backward(MOTOR_CONTROL_PWM); }
        else if (strcmp(dir, "left") == 0)      { motor_turn_left(MOTOR_CONTROL_PWM); }
        else if (strcmp(dir, "right") == 0)     { motor_turn_right(MOTOR_CONTROL_PWM); }
        else if (strcmp(dir, "stop") == 0)      { motor_stop(); }
        else { ESP_LOGW(TAG, "Unknown direction: %s", dir); }
    }
    else
    {
        ESP_LOGW(TAG, "Command not processed due to invalid sequence or missing direction");
    }

    /* Send a response back to the client using the same request context.
     * This is crucial for completing the HTTP request-response cycle.
     * By default, httpd_resp_send sends a "200 OK" status code, indicating
     * success to the client's fetch API. The content ("Command received")
     * provides confirmation text which the client-side JavaScript logs.
     * 
     * Ultimately, the "200 OK" status code is crucial for the client to know
     * that the request was processed successfully. The content is not
     * essential but is useful for debugging/logging purposes.
     **/
    char response_buf[80]; // Buffer for the response message

    /* Construct the response message based on the command processing result */
    if (process_command) // Seq was parsed and command was processed
    {
        snprintf(response_buf, sizeof(response_buf), "Command %s (Seq %lu) processed", dir, receivedSeq);
    } 
    else if (receivedSeq > 0) // Seq was parsed but deemed out of order
    { 
        snprintf(response_buf, sizeof(response_buf), "Command %s (Seq %lu) ignored (out of order)", dir, receivedSeq);
    } 
    else // No query, missing seq, or invalid seq format
    { 
        strncpy(response_buf, "Command ignored (missing/invalid seq)", sizeof(response_buf) - 1);
    }

    /* Ensure null termination as snprintf might truncate without null-terminating if buffer is exactly full. */
    response_buf[sizeof(response_buf) - 1] = '\0';

    /* Send the constructed response back to the client */
    httpd_resp_send(req, response_buf, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t ota_get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Received OTA update request");

    /* Send a simple text response back to the client using the same request context.
     * This is crucial for completing the HTTP request-response cycle.
     * By default, httpd_resp_send sends a "200 OK" status code, indicating
     * success to the client's fetch API. The content ("Command received")
     * provides confirmation text which the client-side JavaScript logs.
     * 
     * Ultimately, the "200 OK" status code is crucial for the client to know
     * that the request was processed successfully. The content is not
     * essential but is useful for debugging/logging purposes.
     **/
    httpd_resp_send(req, "OTA update process initiated...", HTTPD_RESP_USE_STRLEN);

    /* Call the OTA update function */
    /* Note: This function may cause the device to reboot immediately after the update,
     * so any code after this call may not be executed. */
    perform_ota_update();

    return ESP_OK; // Note: Code here might not be reached if OTA causes a reboot immediately
}

static esp_err_t print_get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Serving status request");
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, web_server_print_buffer, HTTPD_RESP_USE_STRLEN);
    
    return ESP_OK;
}



