/******************************************************************************
 * @file esp_now_comm.c
 * @brief ESP-NOW wireless communication component implementation
 * 
 ******************************************************************************/

/*******************************************************************************/
/*                                 INCLUDES                                    */
/*******************************************************************************/
#include "esp_now_comm.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_log.h"
#include "string.h"

/*******************************************************************************/
/*                                  MACROS                                     */
/*******************************************************************************/
#define TAG "ESP_NOW_COMM"

/*******************************************************************************/
/*                                DATA TYPES                                   */
/*******************************************************************************/

/*******************************************************************************/
/*                     GLOBAL FUNCTION DECLARATIONS                            */
/*******************************************************************************/

/*******************************************************************************/
/*                     GLOBAL VARIABLES DECLARATIONS                           */
/*******************************************************************************/

/*******************************************************************************/
/*                     GLOBAL VARIABLES DEFINITIONS                            */
/*******************************************************************************/

/*******************************************************************************/
/*                     STATIC FUNCTION DECLARATIONS                            */
/*******************************************************************************/
/**
 * @brief Callback for ESP-NOW send completion
 * 
 * @param[in] mac_addr MAC address of the peer
 * @param[in] status Send status result
 * 
 * @return None
 */
static void esp_now_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status);

/**
 * @brief Callback for ESP-NOW data reception
 * 
 * @param[in] recv_info Reception info containing source MAC
 * @param[in] data Pointer to received data
 * @param[in] len Length of received data
 * 
 * @return None
 */
static void esp_now_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);

/*******************************************************************************/
/*                             STATIC VARIABLES                                */
/*******************************************************************************/
/**
 * Global configuration structure storing initialization parameters and callbacks
 */
static esp_now_comm_config_t g_config = {0};

/**
 * Counter tracking number of registered peers
 */
static uint8_t g_peer_count = 0;

/*******************************************************************************/
/*                     GLOBAL FUNCTION DEFINITIONS                             */
/*******************************************************************************/

esp_err_t esp_now_comm_init(esp_now_comm_config_t *config)
{
    if (!config) 
    {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;

    /* #01 - Copy user configuration to global config */
    memcpy(&g_config, config, sizeof(esp_now_comm_config_t));

    /* #02 - IMPORTANT: WiFi MUST already be initialized by calling code (e.g., web_server_init)
     * This component assumes:
     *   - esp_netif_init() has been called
     *   - esp_event_loop_create_default() has been called
     *   - esp_wifi_init() has been called
     *   - WiFi mode has been set to WIFI_MODE_STA
     *   - esp_wifi_start() has been called
     * 
     * If we initialize WiFi here, it will conflict with web_server component which also
     * initializes WiFi, causing ESP_ERR_INVALID_STATE errors.
     * 
     * Responsibility: Caller must initialize WiFi before calling esp_now_comm_init()
     */
    
    /* #04 - Retrieve MAC address from already-initialized WiFi */
    ret = esp_wifi_get_mac(WIFI_IF_STA, g_config.mac_addr);
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed to get MAC address: %s (WiFi must be initialized first)", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Retrieved MAC address from WiFi: %02x:%02x:%02x:%02x:%02x:%02x",
             g_config.mac_addr[0], g_config.mac_addr[1], g_config.mac_addr[2],
             g_config.mac_addr[3], g_config.mac_addr[4], g_config.mac_addr[5]);

    /* #05 - Initialize ESP-NOW protocol (WiFi must be running first) */
    ret = esp_now_init();
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "esp_now_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* #06 - Register send completion and receive data callbacks
     * 
     * These callbacks forward ESP-NOW events to user-defined handlers (if provided).
     * The callbacks are bridge functions between the ESP-NOW stack and application logic.
     * 
     * Send callback:
     *   - Invoked asynchronously when a transmission attempt completes (success or failure)
     *   - Reports MAC-layer transmission status (ACK received or transmission failed)
     *   - ESP_NOW_SEND_SUCCESS: MAC-layer frame transmitted and ACK received from peer
     *   - ESP_NOW_SEND_FAIL: No ACK received after max retries (peer offline or out of range)
     *   - Note: Success means radio delivery only, NOT application-level confirmation
     * 
     * Receive callback:
     *   - Invoked whenever any ESP32 sends a message directed at this device's MAC
     *   - Note: No peer registration required to receive from a sender
     *   - Any device that knows this device's MAC can send to it (open to all)
     *   - Peer registration is only required for SENDING, not for RECEIVING
     */
    ret = esp_now_register_send_cb(esp_now_send_cb);
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "esp_now_register_send_cb failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = esp_now_register_recv_cb(esp_now_recv_cb);
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "esp_now_register_recv_cb failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "ESP-NOW communication initialized successfully");
    ESP_LOGI(TAG, "Device MAC: %02x:%02x:%02x:%02x:%02x:%02x",
             g_config.mac_addr[0], g_config.mac_addr[1], g_config.mac_addr[2],
             g_config.mac_addr[3], g_config.mac_addr[4], g_config.mac_addr[5]);

    return ESP_OK;
}

esp_err_t esp_now_comm_add_peer(const uint8_t *mac_addr)
{
    if (!mac_addr || g_peer_count >= ESP_NOW_COMM_MAX_PEERS) 
    {
        return ESP_ERR_INVALID_ARG;
    }

    /* #01 - Configure peer information structure */
    esp_now_peer_info_t peer = 
    {
        .peer_addr = {0},       /* ESPNOW peer MAC address that is also the MAC address of station or softap */
        .lmk = {0},             /* [currently unused] ESPNOW peer local master key that is used to encrypt data */
        .channel = 0,           /* Wi-Fi channel that peer uses to send/receive ESPNOW data. If the value is 0,
                                     use the current channel which station or softap is on. Otherwise, it must be
                                     set as the channel that station or softap is on. */
        .ifidx = WIFI_IF_STA,   /* Wi-Fi interface that peer uses to send/receive ESPNOW data */
        .encrypt = false,       /* [currently unused] ESPNOW data that this peer sends/receives is encrypted or not */
        .priv = NULL            /* [currently unused] ESPNOW peer private data (generic pointer for application-specific custom data) */
    };
    memcpy(peer.peer_addr, mac_addr, 6); // Set peer MAC address of the device we want to communicate with

    /* #02 - Register peer with ESP-NOW */
    esp_err_t ret = esp_now_add_peer(&peer);
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "esp_now_add_peer failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* #03 - Increment peer count and log the MAC address of the added peer */
    g_peer_count++;
    ESP_LOGI(TAG, "Peer added: %02x:%02x:%02x:%02x:%02x:%02x", 
             mac_addr[0], mac_addr[1], mac_addr[2], 
             mac_addr[3], mac_addr[4], mac_addr[5]);

    return ESP_OK;
}

esp_err_t esp_now_comm_remove_peer(const uint8_t *mac_addr)
{
    if (!mac_addr) 
    {
        return ESP_ERR_INVALID_ARG;
    }

    /* #01 - Unregister peer from ESP-NOW */
    esp_err_t ret = esp_now_del_peer(mac_addr);
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "esp_now_del_peer failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* #02 - Decrement current registered peer count and log the MAC address of the removed peer */
    if (g_peer_count > 0) 
    {
        g_peer_count--;
    }
    ESP_LOGI(TAG, "Peer removed: %02x:%02x:%02x:%02x:%02x:%02x", 
             mac_addr[0], mac_addr[1], mac_addr[2], 
             mac_addr[3], mac_addr[4], mac_addr[5]);
    
    return ESP_OK;
}

esp_err_t esp_now_comm_send(const uint8_t *mac_addr, const uint8_t *data, int len)
{
    if (!data || len <= 0 || len > ESP_NOW_COMM_PAYLOAD_SIZE) 
    {
        return ESP_ERR_INVALID_ARG;
    }

    /* Send the provided uint8_t array as ESP-NOW data, with the given length to the specified MAC address (must be registered as a peer first) */
    return esp_now_send(mac_addr, data, len);
}

esp_err_t esp_now_comm_get_mac(uint8_t *mac_addr)
{
    if (!mac_addr) 
    {
        return ESP_ERR_INVALID_ARG;
    }

    /* Return cached MAC address from global config (no hardware call) */
    memcpy(mac_addr, g_config.mac_addr, 6);
    return ESP_OK;
}

esp_err_t esp_now_comm_deinit(void)
{
    /* Deinitialize the ESP-NOW protocol stack
     * This releases ESP-NOW resources and stops receiving packets */
    esp_now_deinit();
    
    /* Stop the WiFi driver
     * This powers down the WiFi radio */
    esp_wifi_stop();
    
    ESP_LOGI(TAG, "ESP-NOW communication deinitialized");
    return ESP_OK;
}

/*******************************************************************************/
/*                     STATIC FUNCTION DEFINITIONS                             */
/*******************************************************************************/

static void esp_now_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    /* Invoke user callback if registered */
    if (g_config.on_send) 
    {
        g_config.on_send(mac_addr, status);
    }
}

static void esp_now_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    /* Invoke user callback if registered */
    if (g_config.on_recv) 
    {
        g_config.on_recv(recv_info->src_addr, data, len);
    }
}