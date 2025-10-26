/******************************************************************************
 * @file esp_now_comm.h
 * @brief Header file for ESP-NOW wireless communication component
 *
 ******************************************************************************/

#ifndef ESP_NOW_COMM_H
#define ESP_NOW_COMM_H

/*******************************************************************************/
/*                                 INCLUDES                                    */
/*******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_now.h"

/*******************************************************************************/
/*                                  MACROS                                     */
/*******************************************************************************/

/* ESP-NOW macros based on info from v5.5.1 ESP-IDF docs: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/network/esp_now.html */
/* Maximum number of peer devices that can be registered (ESP-NOW supports up to 20) */
#define ESP_NOW_COMM_MAX_PEERS 20
/* Maximum number of encrypted peer devices (configurable, default 7, max 17) */
#define ESP_NOW_COMM_MAX_ENCRYPT_PEERS 7
/* Maximum size of ESP-NOW payload in bytes 
 * v1.0 devices: 250 bytes
 * v2.0 devices: 1470 bytes
 * Using v1.0 max for compatibility with older devices, which would truncate larger packets
 */
#define ESP_NOW_COMM_PAYLOAD_SIZE 250

/*******************************************************************************/
/*                                DATA TYPES                                   */
/*******************************************************************************/
/**
 * @brief Callback function type for ESP-NOW data reception
 * 
 * @param[in] mac_addr MAC address of the peer that sent the data
 * @param[in] data Pointer to received data buffer
 * @param[in] len Length of received data in bytes
 */
typedef void (*esp_now_recv_callback_t)(const uint8_t *mac_addr, 
                                        const uint8_t *data, 
                                        int len);

/**
 * @brief Callback function type for ESP-NOW send completion
 * 
 * @param[in] mac_addr MAC address of the destination peer
 * @param[in] status ESP_NOW_SEND_SUCCESS or ESP_NOW_SEND_FAIL
 */
typedef void (*esp_now_send_callback_t)(const uint8_t *mac_addr, 
                                        esp_now_send_status_t status);

/**
 * @brief Configuration structure for ESP-NOW communication
 */
typedef struct 
{
    /* MAC address of this device */
    uint8_t mac_addr[6];
    
    /* Callback invoked when data is received from a peer */
    esp_now_recv_callback_t on_recv;
    
    /* Callback invoked when a send operation completes */
    esp_now_send_callback_t on_send;
} esp_now_comm_config_t;

/*******************************************************************************/
/*                     GLOBAL VARIABLES DECLARATIONS                           */
/*******************************************************************************/

/*******************************************************************************/
/*                     GLOBAL FUNCTION DECLARATIONS                            */
/*******************************************************************************/

/**
 * @brief Initialize ESP-NOW communication subsystem
 *
 * @details This function initializes WiFi in STA mode and sets up the ESP-NOW
 *          protocol stack. It must be called before any other ESP-NOW
 *          communication functions. The configuration structure provides
 *          callback functions for handling received data and send completions.
 *          The device's MAC address is stored in config->mac_addr during initialization.
 *
 * @param[in,out] config Pointer to configuration structure with callbacks.
 *                        On return, mac_addr field will be populated with device MAC.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if config is NULL
 *      - Other esp_err_t codes if initialization fails
 */
esp_err_t esp_now_comm_init(esp_now_comm_config_t *config);

/**
 * @brief Add a peer device for ESP-NOW communication
 *
 * @details Registers a peer device by its MAC address. The peer must be added
 *          before any data can be sent to it. Up to ESP_NOW_COMM_MAX_PEERS
 *          peers can be registered.
 *
 * @param[in] mac_addr 6-byte MAC address of the peer device
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if mac_addr is NULL or max peers exceeded
 *      - Other esp_err_t codes if operation fails
 */
esp_err_t esp_now_comm_add_peer(const uint8_t *mac_addr);

/**
 * @brief Remove a peer device from ESP-NOW communication
 *
 * @details Unregisters a previously added peer. After removal, no data can
 *          be sent to this peer until it is added again.
 *
 * @param[in] mac_addr 6-byte MAC address of the peer to remove
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if mac_addr is NULL
 *      - Other esp_err_t codes if operation fails
 */
esp_err_t esp_now_comm_remove_peer(const uint8_t *mac_addr);

/**
 * @brief Send data to a peer device via ESP-NOW
 *
 * @details Transmits data to the specified peer. The send completion
 *          is reported asynchronously via the on_send callback.
 * 
 *          Note - Messages can be sent ONLY TO REGISTERED PEERS. If the
 *                peer is not registered, the send operation will fail.
 *
 *          Addressing modes:
 *          - Specific MAC address (unicast): Sends to one registered peer
 *          - NULL (broadcast to peers): Sends to all registered peers
 *          - {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} (broadcast): Sends to all ESP32 devices in range,
 *            regardless of peer registration
 *
 * @param[in] mac_addr MAC address of destination peer, or:
 *                      - NULL to broadcast to all registered peers
 *                      - {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} for true broadcast
 * @param[in] data Pointer to data buffer to send
 * @param[in] len Length of data in bytes (max ESP_NOW_COMM_PAYLOAD_SIZE)
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if parameters are invalid
 *      - Other esp_err_t codes if operation fails
 */
esp_err_t esp_now_comm_send(const uint8_t *mac_addr, const uint8_t *data, int len);

/**
 * @brief Get this device's MAC address
 *
 * @details Simple interface for getting the MAC address, so that the caller doesn't
 *          need to be concerned with the underlying WiFi implementation.
 *          The MAC address is crucial for identifying this device on the network and
 *          used by peers to communicate with this device.
 *
 * @param[out] mac_addr Pointer to 6-byte buffer for MAC address storage
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if mac_addr is NULL
 *      - Other esp_err_t codes if operation fails
 */
esp_err_t esp_now_comm_get_mac(uint8_t *mac_addr);

/**
 * @brief Deinitialize ESP-NOW communication subsystem
 *
 * @details Shuts down ESP-NOW and WiFi. After this call, all peers are
 *          unregistered and communication is no longer possible until
 *          esp_now_comm_init is called again.
 *
 * @return ESP_OK on success
 */
esp_err_t esp_now_comm_deinit(void);

#endif /* ESP_NOW_COMM_H */
