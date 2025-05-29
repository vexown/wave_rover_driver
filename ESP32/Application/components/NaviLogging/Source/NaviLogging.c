/******************************************************************************
 * @file NaviLogging.c
 * @brief Component for receiving navigation data via ESP-NOW.
 * 
 ******************************************************************************/

/*******************************************************************************/
/*                                 INCLUDES                                    */
/*******************************************************************************/
/*    Include headers required for the definitions/implementation in *this*    */
/* source file. This typically includes this module's own header("template.h") */
/*      and any headers needed for function bodies, static variables, etc.     */
/*******************************************************************************/
/* ESP-IDF Includes */
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "string.h"
#include "web_server.h"

/* Project Includes */
#include "NaviLogging.h"
/*******************************************************************************/
/*                                  MACROS                                     */
/*******************************************************************************/
/** Tag for logging (used in ESP_LOGI, ESP_LOGE, etc.) Example usage: 
 *  ESP_LOGI(TAG, "Log message which will be appended to the tag"); */
#define TAG "NAVILOGGING"

#define NAVILOGGING_RECEIVE_TIMEOUT_MS 10000 // Timeout for new data (10 seconds)

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

/*******************************************************************************/
/*                     GLOBAL VARIABLES DEFINITIONS                            */
/*******************************************************************************/
/*    for variables defined in this .c file, to be used in other .c files.     */
/*******************************************************************************/

/*******************************************************************************/
/*                     STATIC FUNCTION DECLARATIONS                            */
/*******************************************************************************/

/**
 * @brief Callback function for ESP-NOW receive data.
 * 
 * This function is called when an ESP-NOW message is received, providing the receive info
 * (including MAC address) and the data payload.
 * 
 * @param recv_info Pointer to the receive information structure containing MAC address and other info.
 * @param data Pointer to the received data.
 * @param data_len Length of the received data.
 */
static void esp_now_receive_callback(const esp_now_recv_info_t *recv_info, const uint8_t *data, int data_len);

/*******************************************************************************/
/*                             STATIC VARIABLES                                */
/*******************************************************************************/
/* Variable for storing received coordinates */
static navi_coordinates_type last_coordinates = {0.0, 0.0, 0.0};

/* Flag to signal if new data is available */
static bool new_data_available = false;

/**
 * MAC address of the ESP-NOW peer device.
 * 
 * In ESP-NOW, a "peer" is another ESP32 device that this device will communicate with.
 * For receiving data, this MAC address represents the sender device that we expect
 * to receive data from. You can set this to a specific MAC address for unicast
 * communication, or use the broadcast address to receive from any ESP-NOW device.
 * 
 * How to find the MAC address of an ESP32 - just print it out on the given device (but AFTER WiFi is initialized):
        #include <esp_wifi.h> 
 
        uint8_t mac_addr[6];
        esp_wifi_get_mac(ESP_IF_WIFI_STA, mac_addr);
        ESP_LOGI("MAC_INFO", "My MAC Address is: %02X:%02X:%02X:%02X:%02X:%02X", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
 
 * Then copy that MAC address and use it to initialize the `peer_mac` array below.
 * For example, if the MAC is AA:BB:CC:11:22:33, then:
 * static uint8_t peer_mac[ESP_NOW_ETH_ALEN] = {0xAA, 0xBB, 0xCC, 0x11, 0x22, 0x33};
 *
 * Value {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} is the broadcast address,
 * which allows receiving data from all ESP-NOW devices in range.
 * For unicast (one-to-one) communication, replace it with the specific MAC address of the sender.
 */
static uint8_t sender_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Broadcast - receive from any device

/*******************************************************************************/
/*                     GLOBAL FUNCTION DEFINITIONS                             */
/*******************************************************************************/

esp_err_t NaviLogging_init(void)
{
    char print_buffer[256]; // Buffer for web_server_print

    snprintf(print_buffer, sizeof(print_buffer), "Initializing NaviLogging component for receiving data");
    web_server_print(print_buffer);

    /* Initialize ESP-NOW. This function is not open-source so we don't really know the details of what it does but yea */
    esp_err_t ret = esp_now_init();
    if (ret != ESP_OK) 
    {
        snprintf(print_buffer, sizeof(print_buffer), "NaviLogging: esp_now_init failed: %s", esp_err_to_name(ret));
        web_server_print(print_buffer);
        ESP_ERROR_CHECK(ret); // This will halt if ret is not ESP_OK
    }

    /* Register the ESP-NOW receive callback function.
     * This function will be called whenever data is received via ESP-NOW.
     * The callback will handle the incoming data and update the last_coordinates variable.*/
    ret = esp_now_register_recv_cb(esp_now_receive_callback);
    if (ret != ESP_OK) 
    {
        snprintf(print_buffer, sizeof(print_buffer), "NaviLogging: esp_now_register_recv_cb failed: %s", esp_err_to_name(ret));
        web_server_print(print_buffer);
        ESP_ERROR_CHECK(ret);
    }

    /* Verify the WiFi channel currently in use. Well, not really verify, but print it out so you can see it and compare it with the sender's channel.
     * This is important because ESP-NOW operates on the same channel as the Wi-Fi interface.
     * For ESP-NOW to work correctly, the sender and receiver must be on the same channel. */
    uint8_t primary_channel;
    wifi_second_chan_t second_channel;
    if (esp_wifi_get_channel(&primary_channel, &second_channel) == ESP_OK) 
    {
        snprintf(print_buffer, sizeof(print_buffer), "NaviLogging: Current Wi-Fi Primary Channel: %d", primary_channel);
        web_server_print(print_buffer);
    } 
    else 
    {
        web_server_print("NaviLogging: Failed to get Wi-Fi channel");
    }

    /* Configure the ESP-NOW peer information. This is where we set up the peer device that we want to receive data from. */
    esp_now_peer_info_t peer_info = {0};
    memcpy(peer_info.peer_addr, sender_mac, ESP_NOW_ETH_ALEN);
    /*
     * Set the ESP-NOW peer channel.
     * A value of 0 here is a special instruction for ESP-NOW: it means that
     * ESP-NOW should use the current operational Wi-Fi channel of the interface
     * specified by 'peer_info.ifidx' (which is WIFI_IF_STA in this case).
     *
     * For this device (receiver), if it's connected to a Wi-Fi Access Point (AP),
     * it will be operating on the AP's channel (e.g., Channel 9 as observed).
     * You can verify the actual channel being used by checking the output of
     * esp_wifi_get_channel() earlier in this initialization function.
     *
     * IMPORTANT: Any other ESP32 devices (senders) that need to communicate
     * with this device via ESP-NOW MUST be configured to operate on this
     * exact same Wi-Fi channel (via esp_wifi_set_channel()).
     */
    peer_info.channel = 0;
    /* 
     * Set the interface type for the peer.
     * This should match the interface type used by the sender device.
     * For ESP-NOW, this is typically WIFI_IF_STA (Station mode).
     * If you are using a different interface (like WIFI_IF_AP for Soft-AP),
     * make sure to change this accordingly.
     */
    peer_info.ifidx = WIFI_IF_STA; // IMPORTANT: Verify this matches your active Wi-Fi interface for ESP-NOW
    /* 
     * Set the local master key (LMK) for the peer.
     * This is used for encryption. If you are not using encryption,
     * you can leave this as all zeros or set it to a known value.
     * For simplicity, we are not using encryption in this example.
     */
    memset(peer_info.lmk, 0, ESP_NOW_KEY_LEN); // No encryption, so LMK is set to all zeros
    /* 
     * Set the encryption flag.
     * If you are not using encryption, set this to false.
     * If you want to use encryption, set it to true and provide a valid LMK.
     */
    peer_info.encrypt = false;

    /* Finally, add the configured peer to the ESP-NOW peer list.
     * This allows the ESP-NOW stack to recognize this peer and send/receive data to/from it. */
    ret = esp_now_add_peer(&peer_info);
    if (ret != ESP_OK) 
    {
        snprintf(print_buffer, sizeof(print_buffer), "NaviLogging: esp_now_add_peer failed: %s", esp_err_to_name(ret));
        web_server_print(print_buffer);
        ESP_ERROR_CHECK(ret);
    }

    snprintf(print_buffer, sizeof(print_buffer), "NaviLogging initialized successfully for receiving data from any ESP-NOW device");
    web_server_print(print_buffer);
    return ESP_OK;
}

esp_err_t NaviLogging_get_last_coordinates(navi_coordinates_type *coordinates)
{
    char print_buffer[256]; // Buffer for web_server_print

    /* Check if valid pointer is provided */
    if (coordinates == NULL) 
    {
        snprintf(print_buffer, sizeof(print_buffer), "NaviLogging Error: Coordinates pointer is NULL");
        web_server_print(print_buffer);
        return ESP_ERR_INVALID_ARG;
    }

    /* Copy the last received coordinates to the provided pointer */
    *coordinates = last_coordinates;

    /* Mark the coordinates as retrieved (not new anymore) */
    new_data_available = false;
    
    snprintf(print_buffer, sizeof(print_buffer), "NaviLogging Info: Retrieved coordinates: lat=%.6f, lon=%.6f, alt=%.2f",
             coordinates->latitude, coordinates->longitude, coordinates->altitude);
    web_server_print(print_buffer);
    
    return ESP_OK;
}

bool NaviLogging_is_new_data_available(void)
{
    /* Let the user know if new data is available or the last data was already retrieved */
    if (new_data_available) 
    {
        web_server_print("NaviLogging Info: New data is available");
    } 
    else 
    {
        web_server_print("NaviLogging Info: No new data available");
    }

    return new_data_available;
}

/*******************************************************************************/
/*                     STATIC FUNCTION DEFINITIONS                             */
/*******************************************************************************/

static void esp_now_receive_callback(const esp_now_recv_info_t *recv_info, const uint8_t *data, int data_len)
{
    char print_buffer[256]; // Buffer for web_server_print

    web_server_print("NaviLogging: ESP-NOW data received");

    /* Check if the packet info exists and has a valid source address */
    if (recv_info == NULL || recv_info->src_addr == NULL) 
    {
        snprintf(print_buffer, sizeof(print_buffer), "NaviLogging Error: Invalid receive info or source address");
        web_server_print(print_buffer);
        return;
    }

    /* Now that we know packet info is valid, check out who sent us this data */
    const uint8_t *mac_addr = recv_info->src_addr;
    snprintf(print_buffer, sizeof(print_buffer), "NaviLogging Info: ESP-NOW data received from %02X:%02X:%02X:%02X:%02X:%02X, length: %d",
             mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5], data_len);
    web_server_print(print_buffer);

    /* Check if the received data length matches the expected size. If it doesn't, we might be receving corrupted data or simply not the data we expect */
    if (data_len != sizeof(navi_coordinates_type)) 
    {
        snprintf(print_buffer, sizeof(print_buffer), "NaviLogging Warning: Received data size (%d) doesn't match expected size (%d)",
                 data_len, sizeof(navi_coordinates_type));
        web_server_print(print_buffer);
        return;
    }

    // Accept data from any ESP-NOW device (no MAC filtering for broadcast mode)
    // if (memcmp(mac_addr, sender_mac, ESP_NOW_ETH_ALEN) != 0) {
    //     snprintf(print_buffer, sizeof(print_buffer), "NaviLogging Warning: Received data from unknown sender, ignoring");
    //     web_server_print(print_buffer);
    //     return;
    // }

    /* Deserialize the received data into the last_coordinates structure. This can be later retrieved by the user via NaviLogging_get_last_coordinates() */
    memcpy(&last_coordinates, data, sizeof(navi_coordinates_type));
    
    /* Indicate that new data is available. This flag can be checked by the user via NaviLogging_is_new_data_available() 
       to avoid wasting time on retrieving data that is not new */
    new_data_available = true;
}
