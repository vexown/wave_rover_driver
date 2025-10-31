/*******************************************************************************/
/*                                INCLUDES                                     */
/*******************************************************************************/
#pragma once

#include "esp_err.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************/
/*                                DATA TYPES                                   */
/*******************************************************************************/

/**
 * @brief Callback function type for WiFi disconnection event
 * 
 * @details This callback is invoked when WiFi disconnects. The application
 *          can use this to implement safety measures (e.g., stop motors).
 * 
 * @return ESP_OK if handled successfully, ESP_FAIL otherwise
 */
typedef esp_err_t (*wifi_disconnect_cb_t)(void);

/**
 * @brief Callback function type for status display updates
 * 
 * @param status_line Status message (e.g., "WiFi Connected", "WiFi Failed!")
 * @param detail_line Detail message (e.g., IP address, empty string)
 */
typedef void (*wifi_status_display_cb_t)(const char *status_line, const char *detail_line);

/**
 * @brief WiFi manager configuration structure
 */
typedef struct {
    wifi_disconnect_cb_t on_disconnect;        /**< Optional callback for disconnection handling */
    wifi_status_display_cb_t on_status_update; /**< Optional callback for status display updates */
} wifi_manager_callbacks_t;

/*******************************************************************************/
/*                     GLOBAL VARIABLES DECLARATIONS                           */
/*******************************************************************************/

/**
 * @brief WiFi event group for connection state management
 */
extern EventGroupHandle_t WiFi_EventGroup;

/**
 * @brief Event group bits for WiFi connection status
 */
#define WIFI_CONNECTED_BIT BIT0 // We have connected to the AP specified in the WiFi credentials
#define WIFI_FAIL_BIT      BIT1 // We have failed to connect to the AP after max retries

/**
 * @brief Buffer to store the current IP address in STA mode (e.g., "192.168.1.100")
 */
extern char STA_IP_Addr_String[16];

/*******************************************************************************/
/*                        GLOBAL FUNCTION DECLARATIONS                         */
/*******************************************************************************/

/**
 * @brief Get the WiFi channel currently in use
 *
 * @details Retrieves the primary and secondary WiFi channels. The channel information
 *          is useful when configuring ESP-NOW communication on other devices to ensure
 *          they operate on the same channel.
 *
 * @param[out] primary Pointer to store the primary channel number (1-13 for 2.4GHz)
 * @param[out] second Pointer to store the secondary channel (for HT40 mode)
 * @return ESP_OK on success, ESP_FAIL otherwise
 */
esp_err_t wifi_manager_get_channel(uint8_t *primary, wifi_second_chan_t *second);

/**
 * @brief Initialize WiFi in station mode with optional callbacks
 *
 * @details This function initializes the WiFi driver, sets up event handlers
 *          for WiFi and IP events, configures the station mode with the
 *          credentials from WiFi_Credentials.h (WIFI_SSID, WIFI_PASSWORD), and waits
 *          for the connection to complete. All WiFi-related initialization
 *          logic is encapsulated in this component.
 *
 * @param callbacks Optional callbacks for disconnect and status updates (can be NULL)
 * @return ESP_OK on successful connection to the AP
 * @return ESP_FAIL on failure after WIFI_MAXIMUM_RETRY attempts
 */
esp_err_t wifi_manager_init(const wifi_manager_callbacks_t *callbacks);

/**
 * @brief Deinitialize WiFi
 *
 * @details This function stops the WiFi driver and cleans up associated resources.
 *
 * @return ESP_OK on success
 */
esp_err_t wifi_manager_deinit(void);

#ifdef __cplusplus
}
#endif
