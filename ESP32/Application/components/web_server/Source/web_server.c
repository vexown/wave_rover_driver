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
#include "wifi_manager.h"
#include "HTML_page.h"

/*******************************************************************************/
/*                                  MACROS                                     */
/*******************************************************************************/
/** Tag for logging (used in ESP_LOGI, ESP_LOGE, etc.) Example usage: 
 *  ESP_LOGI(TAG, "Log message which will be appended to the tag"); */
#define TAG "WEB_SERVER"

/* Max retry attempts for connection */
#define WIFI_MAXIMUM_RETRY 5

/* OTA HTTPS server URL to the firmware binary. For the firmware binary to be
 * placed there, use UploadImageToServer.sh script. (build the firmware first) */
#define FIRMWARE_UPGRADE_URL "https://192.168.50.194/firmware_wave_rover_driver.bin"

/* Motor control parameters */
#define MOTOR_CONTROL_PWM 255 //TODO - make this configurable on the web page?

/* Max length for the message to be displayed on the web page */
#define WEBSERVER_PRINT_MAX_LEN 2048 // in bytes

/* WebSocket macros */
#define WS_MAX_CLIENTS 4

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
extern const uint8_t server_cert_pem_start[] asm("_binary_apache_ip_cert_crt_start");
extern const uint8_t server_cert_pem_end[]   asm("_binary_apache_ip_cert_crt_end");

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
 * @brief HTTP GET handler for the "/reset" URI.
 *
 * @details Initiates a software reset of the device.
 *
 * @param req Pointer to the HTTP request structure.
 * @return esp_err_t ESP_OK on success.
 */
static esp_err_t reset_get_handler(httpd_req_t *req);

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


static esp_err_t ws_handler(httpd_req_t *req);


static void ws_add_client(int sockfd);


static void ws_remove_client(int sockfd);

/*******************************************************************************/
/*                             STATIC VARIABLES                                */
/*******************************************************************************/
/* HTTP server handle */
static httpd_handle_t server = NULL;

/* Buffer to store the latest message */
static char web_server_print_buffer[WEBSERVER_PRINT_MAX_LEN] = "Initializing...";

/* Sequence Number Handling */
static uint32_t lastProcessedSequenceNumber = 0; // Track the last processed sequence number
static SemaphoreHandle_t sequenceMutex = NULL; // Mutex to protect sequence number access

/* WebSocket client tracking */
static int ws_client_fds[WS_MAX_CLIENTS] = { -1, -1, -1, -1 };
static SemaphoreHandle_t ws_clients_mutex = NULL;

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

static const httpd_uri_t uri_reset =
{
    .uri       = "/reset",
    .method    = HTTP_GET,
    .handler   = reset_get_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t uri_ws =
{
    .uri                      = "/ws",
    .method                   = HTTP_GET,
    .handler                  = ws_handler,
    .user_ctx                 = NULL,
    .is_websocket             = true,
    .handle_ws_control_frames = true,
};

/*******************************************************************************/
/*                     GLOBAL FUNCTION DEFINITIONS                             */
/*******************************************************************************/

esp_err_t web_server_init(void) 
{
    esp_err_t ret = ESP_OK;

    sequenceMutex = xSemaphoreCreateMutex();
    ws_clients_mutex = xSemaphoreCreateMutex();
    if (sequenceMutex == NULL || ws_clients_mutex == NULL)
    {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Starting Web Server...");
    ret = start_webserver();
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "Web Server start failed!");
        return ret;
    }

    ESP_LOGI(TAG, "Web Server started successfully");
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

/* Broadcast IMU orientation (Euler angles) JSON to all connected WS clients */
void web_server_ws_broadcast_imu_orientation(float roll, float pitch, float yaw)
{
    if (server == NULL) return;

    char msg[140];
    int n = snprintf(msg, sizeof(msg),
                     "{\"type\":\"imu_orientation\",\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f}",
                     roll, pitch, yaw);
    if (n <= 0) return;

    httpd_ws_frame_t frame = {
        .final = true,
        .fragmented = false,
        .type = HTTPD_WS_TYPE_TEXT,
        .payload = (uint8_t *)msg,
        .len = (size_t)n
    };

    int active_clients = 0;
    if (ws_clients_mutex) xSemaphoreTake(ws_clients_mutex, portMAX_DELAY);
    for (int i = 0; i < WS_MAX_CLIENTS; ++i)
    {
        int fd = ws_client_fds[i];
        if (fd != -1)
        {
            active_clients++;
            esp_err_t err = httpd_ws_send_frame_async(server, fd, &frame);
            if (err != ESP_OK)
            {
                /* Drop bad clients */
                ESP_LOGW(TAG, "Failed to send WebSocket frame to client %d, removing client", fd);
                ws_client_fds[i] = -1;
            }
        }
    }
    if (ws_clients_mutex) xSemaphoreGive(ws_clients_mutex);

    /* Log occasionally for debugging (every 1000 calls = ~50 seconds at 50ms period) */
    static int orientation_debug_counter = 0;
    if (++orientation_debug_counter >= 1000) 
    {
        ESP_LOGI(TAG, "Broadcasting IMU orientation data to %d WebSocket clients", active_clients);
        orientation_debug_counter = 0;
    }
}

esp_err_t web_server_get_ip(char *ip_buffer, size_t buffer_size)
{
    if (ip_buffer == NULL || buffer_size == 0)
    {
        return ESP_FAIL;
    }

    if (WiFi_EventGroup == NULL || !(xEventGroupGetBits(WiFi_EventGroup) & WIFI_CONNECTED_BIT))
    {
        strncpy(ip_buffer, "No IP", buffer_size - 1);
        ip_buffer[buffer_size - 1] = '\0';
        return ESP_FAIL;
    }

    strncpy(ip_buffer, STA_IP_Addr_String, buffer_size - 1);
    ip_buffer[buffer_size - 1] = '\0';
    return ESP_OK;
}

bool web_server_is_connected(void)
{
    if (WiFi_EventGroup == NULL)
    {
        return false;
    }
    return (xEventGroupGetBits(WiFi_EventGroup) & WIFI_CONNECTED_BIT) != 0;
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
        httpd_register_uri_handler(server, &uri_reset);
        httpd_register_uri_handler(server, &uri_ws);

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
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, web_server_print_buffer, HTTPD_RESP_USE_STRLEN);
    
    return ESP_OK;
}

static esp_err_t reset_get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Received reset request");
    
    /* Send a response back to the client indicating the reset is being performed */
    httpd_resp_send(req, "Resetting device...", HTTPD_RESP_USE_STRLEN);

    /* Short delay to allow the HTTP response to be sent before reset */
    vTaskDelay(pdMS_TO_TICKS(100));

    /* Perform the reset */
    esp_restart(); // This will reboot the device immediately

    return ESP_OK; // Note: Code here will not be reached
}

/* Helpers to manage WS client list */
static void ws_add_client(int sockfd)
{
    if (ws_clients_mutex) xSemaphoreTake(ws_clients_mutex, portMAX_DELAY);
    for (int i = 0; i < WS_MAX_CLIENTS; ++i)
    {
        if (ws_client_fds[i] == sockfd) { if (ws_clients_mutex) xSemaphoreGive(ws_clients_mutex); return; }
    }
    for (int i = 0; i < WS_MAX_CLIENTS; ++i)
    {
        if (ws_client_fds[i] == -1)
        {
            ws_client_fds[i] = sockfd;
            break;
        }
    }
    if (ws_clients_mutex) xSemaphoreGive(ws_clients_mutex);
}

static void ws_remove_client(int sockfd)
{
    if (ws_clients_mutex) xSemaphoreTake(ws_clients_mutex, portMAX_DELAY);
    for (int i = 0; i < WS_MAX_CLIENTS; ++i)
    {
        if (ws_client_fds[i] == sockfd)
        {
            ws_client_fds[i] = -1;
            break;
        }
    }
    if (ws_clients_mutex) xSemaphoreGive(ws_clients_mutex);
}

/* WebSocket handler */
static esp_err_t ws_handler(httpd_req_t *req)
{
    /* HTTP GET means handshake is being done */
    if (req->method == HTTP_GET)
    {
        /* Add client immediately after successful handshake */
        int sockfd = httpd_req_to_sockfd(req);
        ws_add_client(sockfd);
        ESP_LOGI(TAG, "WebSocket client connected, sockfd: %d", sockfd);
        return ESP_OK;
    }

    httpd_ws_frame_t ws_pkt = {
        .final = true,
        .fragmented = false,
        .type = HTTPD_WS_TYPE_TEXT,
        .payload = NULL,
        .len = 0
    };

    /* First get frame length */
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) return ret;

    /* Allocate and receive payload if any */
    uint8_t *buf = NULL;
    if (ws_pkt.len)
    {
        buf = (uint8_t *)malloc(ws_pkt.len + 1);
        if (!buf) return ESP_ERR_NO_MEM;
        ws_pkt.payload = buf;
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret != ESP_OK) { free(buf); return ret; }
        buf[ws_pkt.len] = 0;
    }

    int sockfd = httpd_req_to_sockfd(req);

    /* Track clients; handle close frames */
    if (ws_pkt.type == HTTPD_WS_TYPE_CLOSE)
    {
        ws_remove_client(sockfd);
        ESP_LOGI(TAG, "WebSocket client disconnected, sockfd: %d", sockfd);
    }
    /* Note: Client was already added during handshake, no need to add again */

    if (buf) free(buf);
    return ESP_OK;
}



