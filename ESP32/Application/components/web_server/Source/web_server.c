// filepath: /home/blankmcu/Repos/wave_rover_driver/ESP32/Application/components/web_server/Source/web_server.c
#include "web_server.h"
#include "motor_control.h" // Include motor control functions
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h" // Needed for STA mode
#include "esp_http_server.h"
#include "esp_mac.h"
#include "esp_check.h"
#include <string.h>
#include <stdlib.h> // For atoi
#include "freertos/FreeRTOS.h" // Needed for event groups
#include "freertos/task.h"
#include "freertos/event_groups.h" // Needed for event groups
#include <lwip/sockets.h> // For inet_ntoa

// Declare the OTA update function from app_main.c
extern void perform_ota_update();
// Declare oled functions needed here (or pass IP back via callback/queue)
extern esp_err_t oled_write_string(uint8_t line, const char* text);
extern esp_err_t oled_refresh(void);
extern esp_err_t oled_err; // Access global oled_err status

// --- Station Mode Configuration ---
#include "WiFi_Credentials.h" // Include WiFi credentials header (not committed to the repo, you must provide it locally)
#define WIFI_MAXIMUM_RETRY 5 // Max retry attempts for connection

// --- Event Group ---
/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;
/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

// Motor control parameters (adjust PWM value as needed)
#define MOTOR_CONTROL_PWM 180 // Example PWM value for movement

static const char *TAG = "WEB_SERVER_STA"; // Changed tag slightly
static httpd_handle_t server = NULL;
static int s_retry_num = 0; // Connection retry counter
static char sta_ip_addr_str[16] = "0.0.0.0"; // To store the assigned IP

// Basic HTML Page with Motor Control Buttons and OTA Button
const char *HTML_PAGE = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<title>Wave Rover Control</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
body { font-family: Arial, sans-serif; text-align: center; }
.btn { display: inline-block; padding: 15px 25px; font-size: 24px; cursor: pointer;
       text-align: center; text-decoration: none; outline: none; color: #fff;
       background-color: #4CAF50; border: none; border-radius: 15px; box-shadow: 0 9px #999; margin: 10px; }
.btn:active { background-color: #3e8e41; box-shadow: 0 5px #666; transform: translateY(4px); }
.stop { background-color: #f44336; }
.ota { background-color: #ff9800; } /* Style for OTA button */
.grid-container { display: grid; grid-template-columns: auto auto auto; padding: 10px; justify-content: center; }
.grid-item { padding: 20px; font-size: 30px; text-align: center; }
</style>
<script>
function sendCmd(dir) {
  fetch('/control?dir=' + dir)
    .then(response => response.text())
    .then(data => console.log(data))
    .catch(error => console.error('Error:', error));
}
function startOta() {
  // Optionally display a confirmation message
  if (confirm('Start Firmware Update? The device will reboot.')) {
    fetch('/ota')
      .then(response => response.text())
      .then(data => {
         console.log(data);
         alert('OTA process started. Check device logs.'); // Inform user
      })
      .catch(error => {
         console.error('Error:', error);
         alert('Failed to start OTA.'); // Inform user of failure
      });
  }
}
</script>
</head>
<body>
<h1>Wave Rover Control</h1>
<div class="grid-container">
  <div class="grid-item"></div>
  <div class="grid-item"><button class="btn" onclick="sendCmd('forward')">Forward</button></div>
  <div class="grid-item"></div>
  <div class="grid-item"><button class="btn" onclick="sendCmd('left')">Left</button></div>
  <div class="grid-item"><button class="btn stop" onclick="sendCmd('stop')">Stop</button></div>
  <div class="grid-item"><button class="btn" onclick="sendCmd('right')">Right</button></div>
  <div class="grid-item"></div>
  <div class="grid-item"><button class="btn" onclick="sendCmd('backward')">Backward</button></div>
  <div class="grid-item"></div>
</div>
<hr>
<div>
  <button class="btn ota" onclick="startOta()">Update Firmware</button>
</div>
</body>
</html>
)rawliteral";

// Event handler for WiFi events
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                            int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "WIFI_EVENT_STA_START: Connecting...");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < WIFI_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Retry connection to the AP (%d/%d)", s_retry_num, WIFI_MAXIMUM_RETRY);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGE(TAG, "Connect to the AP failed after %d retries", WIFI_MAXIMUM_RETRY);
            // Update OLED on failure
            if (oled_err == ESP_OK) {
                oled_write_string(2, "WiFi Failed!");
                oled_write_string(3, ""); // Clear IP line
                oled_refresh();
            }
        }
    }
}

// Event handler for IP events
static void ip_event_handler(void* arg, esp_event_base_t event_base,
                           int32_t event_id, void* event_data)
{
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0; // Reset retry counter on successful connection
        snprintf(sta_ip_addr_str, sizeof(sta_ip_addr_str), IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);

        // Update OLED with assigned IP
        if (oled_err == ESP_OK) {
            char line3_buf[20];
            snprintf(line3_buf, sizeof(line3_buf), "IP:%s", sta_ip_addr_str);
            oled_write_string(2, "WiFi Connected"); // Update status
            oled_write_string(3, line3_buf); // Show assigned IP
            oled_refresh();
        }
    }
}


// Initialize WiFi in Station mode
static esp_err_t wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "esp_netif_init failed");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "esp_event_loop_create_default failed");
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&cfg), TAG, "esp_wifi_init failed");

    // Register event handlers
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(WIFI_EVENT,
                                                            ESP_EVENT_ANY_ID,
                                                            &wifi_event_handler,
                                                            NULL,
                                                            &instance_any_id), TAG, "Register WIFI_EVENT failed");
    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(IP_EVENT,
                                                            IP_EVENT_STA_GOT_IP,
                                                            &ip_event_handler,
                                                            NULL,
                                                            &instance_got_ip), TAG, "Register IP_EVENT failed");

    // Configure WiFi Station
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            /* Authmode threshold defaults to WPA2 PSK. */
            .threshold.authmode = WIFI_AUTH_WPA2_PSK, // Or adjust as needed (WIFI_AUTH_WPA_WPA2_PSK, etc.)
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH, // Enable WPA3 support if needed
        },
    };
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "esp_wifi_set_mode failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &wifi_config), TAG, "esp_wifi_set_config failed");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "esp_wifi_start failed");

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event handlers (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE, // Don't clear bits on exit
            pdFALSE, // Wait for EITHER bit
            portMAX_DELAY); // Wait indefinitely

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to ap SSID:%s", WIFI_SSID);
        return ESP_OK; // Successfully connected
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Failed to connect to SSID:%s", WIFI_SSID);
        return ESP_FAIL; // Failed to connect
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT"); // Should not happen
        return ESP_FAIL;
    }
}

// HTTP GET handler for root URL ("/")
static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, HTML_PAGE, HTTPD_RESP_USE_STRLEN); // Use strlen for const char*
    return ESP_OK;
}

// HTTP GET handler for "/control" URL
static esp_err_t control_get_handler(httpd_req_t *req)
{
    char*  buf;
    size_t buf_len;
    char dir[10] = {0}; // Buffer for direction parameter

    // Get URI query parameter 'dir'
    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        if(httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "Found URL query => %s", buf);
            // Parse the direction parameter
            if (httpd_query_key_value(buf, "dir", dir, sizeof(dir)) == ESP_OK) {
                ESP_LOGI(TAG, "Got parameter dir=%s", dir);

                // Control the motor based on the direction
                if (strcmp(dir, "forward") == 0) {
                    motor_move_forward(MOTOR_CONTROL_PWM);
                } else if (strcmp(dir, "backward") == 0) {
                    motor_move_backward(MOTOR_CONTROL_PWM);
                } else if (strcmp(dir, "left") == 0) {
                    motor_turn_left(MOTOR_CONTROL_PWM);
                } else if (strcmp(dir, "right") == 0) {
                    motor_turn_right(MOTOR_CONTROL_PWM);
                } else if (strcmp(dir, "stop") == 0) {
                    motor_stop();
                } else {
                    ESP_LOGW(TAG, "Unknown direction: %s", dir);
                }
            } else {
                ESP_LOGW(TAG, "Parameter 'dir' not found in query");
            }
        } else {
             ESP_LOGE(TAG, "Failed to get URL query string");
        }
        free(buf);
    } else {
         ESP_LOGI(TAG, "No URL query found");
         // Optional: Stop motor if no direction is given?
         // motor_stop();
    }

    // Send response
    httpd_resp_send(req, "Command received", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// HTTP GET handler for "/ota" URL
static esp_err_t ota_get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Received OTA update request");

    // Send response to client first, as perform_ota_update might block/reboot
    httpd_resp_send(req, "OTA update process initiated...", HTTPD_RESP_USE_STRLEN);

    // Call the OTA update function (defined in app_main.c)
    // This function will handle the update and reboot if successful
    perform_ota_update();

    // Note: Code here might not be reached if OTA causes a reboot immediately
    return ESP_OK;
}


// URI definitions
static const httpd_uri_t uri_root = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = root_get_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t uri_control = {
    .uri       = "/control",
    .method    = HTTP_GET,
    .handler   = control_get_handler,
    .user_ctx  = NULL
};

// Add URI definition for OTA
static const httpd_uri_t uri_ota = {
    .uri       = "/ota",
    .method    = HTTP_GET,
    .handler   = ota_get_handler,
    .user_ctx  = NULL
};

// Function to start the web server
static esp_err_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true; // Enable LRU purge for inactive connections

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    esp_err_t ret = httpd_start(&server, &config);
    if (ret == ESP_OK) {
        // Set URI handlers
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &uri_root);
        httpd_register_uri_handler(server, &uri_control);
        httpd_register_uri_handler(server, &uri_ota); // Register OTA handler
        return ESP_OK;
    }

    ESP_LOGE(TAG, "Error starting server!");
    return ret;
}

// Public function to initialize the web server component
esp_err_t web_server_init(void) {
    // Initialize NVS (needed for WiFi STA mode)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(ret, TAG, "NVS Flash init failed");

    // Initialize WiFi Station
    ESP_LOGI(TAG, "Initializing WiFi Station...");
    ret = wifi_init_sta(); // Call STA init function
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi STA initialization failed!");
        return ret; // Stop if WiFi connection fails
    }

    // Start the web server only if WiFi connected successfully
    ESP_LOGI(TAG, "Starting Web Server...");
    ret = start_webserver();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Web Server start failed!");
        // Optional: Stop WiFi?
        // esp_wifi_stop();
        return ret;
    }

    ESP_LOGI(TAG, "Web Server started successfully on IP: %s", sta_ip_addr_str);
    return ESP_OK;
}