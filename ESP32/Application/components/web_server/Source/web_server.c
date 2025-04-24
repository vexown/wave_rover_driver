#include "web_server.h"
#include "motor_control.h" // Include motor control functions
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "esp_mac.h"
#include "esp_check.h"
#include <string.h>
#include <stdlib.h> // For atoi

// WiFi AP Configuration - based on wifi_ctrl.h defaults
#define WIFI_AP_SSID      "WAVE_ROVER_ESP32"
#define WIFI_AP_PASSWORD  "password123" // Must be at least 8 characters
#define WIFI_AP_CHANNEL   1
#define WIFI_AP_MAX_CONN  4

// Motor control parameters (adjust PWM value as needed)
#define MOTOR_CONTROL_PWM 180 // Example PWM value for movement

static const char *TAG = "WEB_SERVER";
static httpd_handle_t server = NULL;
static esp_netif_t *ap_netif = NULL; // Store netif pointer

// Basic HTML Page with Motor Control Buttons
// Uses simple GET requests with query parameters
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
</body>
</html>
)rawliteral";

// Event handler for WiFi events
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "Station " MACSTR " joined, AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "Station " MACSTR " left, AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_START) {
         esp_netif_ip_info_t ip_info;
         esp_netif_get_ip_info(ap_netif, &ip_info);
         ESP_LOGI(TAG, "SoftAP started. SSID:%s Password:%s Channel:%d",
                  WIFI_AP_SSID, WIFI_AP_PASSWORD, WIFI_AP_CHANNEL);
         ESP_LOGI(TAG, "AP IP Address: " IPSTR, IP2STR(&ip_info.ip));
         // Optionally update OLED here if needed
    } else if (event_id == WIFI_EVENT_AP_STOP) {
         ESP_LOGI(TAG, "SoftAP stopped");
    }
}

// Initialize WiFi in SoftAP mode
static esp_err_t wifi_init_softap(void)
{
    esp_err_t ret = nvs_flash_init(); // Initialize NVS
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(ret, TAG, "NVS Flash init failed");

    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "esp_netif_init failed");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "Event loop create failed");

    ap_netif = esp_netif_create_default_wifi_ap(); // Create default AP netif
    ESP_RETURN_ON_FALSE(ap_netif, ESP_FAIL, TAG, "Create AP netif failed");

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&cfg), TAG, "WiFi init failed");

    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL), TAG, "Register WiFi event handler failed");

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_AP_SSID,
            .password = WIFI_AP_PASSWORD,
            .ssid_len = strlen(WIFI_AP_SSID),
            .channel = WIFI_AP_CHANNEL,
            .authmode = WIFI_AUTH_WPA2_PSK, // Use WPA2-PSK
            .max_connection = WIFI_AP_MAX_CONN,
            // .pmf_cfg = { .required = false }, // Optional: Protected Management Frames
        },
    };
    if (strlen(WIFI_AP_PASSWORD) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN; // Use OPEN auth if no password
    }

    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_AP), TAG, "Set WiFi mode failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &wifi_config), TAG, "Set WiFi config failed");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "WiFi start failed");

    ESP_LOGI(TAG, "wifi_init_softap finished.");
    return ESP_OK;
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
    char buf[64]; // Buffer for query string
    esp_err_t ret = ESP_FAIL;
    int pwm = MOTOR_CONTROL_PWM; // Use defined PWM value

    // Get query string
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        char param[16];
        // Parse the 'dir' parameter
        if (httpd_query_key_value(buf, "dir", param, sizeof(param)) == ESP_OK) {
            ESP_LOGI(TAG, "Received control command: dir=%s", param);
            if (strcmp(param, "forward") == 0) {
                ret = motor_set_speed(pwm, pwm);
            } else if (strcmp(param, "backward") == 0) {
                ret = motor_set_speed(-pwm, -pwm);
            } else if (strcmp(param, "left") == 0) {
                ret = motor_set_speed(-pwm, pwm); // Adjust based on motor orientation if needed
            } else if (strcmp(param, "right") == 0) {
                ret = motor_set_speed(pwm, -pwm); // Adjust based on motor orientation if needed
            } else if (strcmp(param, "stop") == 0) {
                ret = motor_stop();
            } else {
                ESP_LOGW(TAG, "Unknown direction: %s", param);
                ret = ESP_ERR_INVALID_ARG;
            }
        } else {
             ESP_LOGW(TAG, "Parameter 'dir' not found in query: %s", buf);
             ret = ESP_ERR_NOT_FOUND;
        }
    } else {
        ESP_LOGW(TAG, "Query string not found");
        ret = ESP_ERR_NOT_FOUND;
    }

    // Respond to the client
    if (ret == ESP_OK) {
        httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    } else if (ret == ESP_ERR_INVALID_ARG || ret == ESP_ERR_NOT_FOUND) {
        httpd_resp_send_404(req);
    } else {
        httpd_resp_send_500(req);
    }
    return ESP_OK; // Return OK regardless of motor control result for HTTP handling
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
        return ESP_OK;
    }

    ESP_LOGE(TAG, "Error starting server!");
    return ret;
}

// Public function to initialize the web server component
esp_err_t web_server_init(void) {
    esp_err_t ret = wifi_init_softap();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize WiFi SoftAP");
        return ret;
    }

    ret = start_webserver();
     if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        // Consider stopping WiFi if server fails?
        return ret;
    }

    ESP_LOGI(TAG, "Web server initialized successfully");
    return ESP_OK;
}