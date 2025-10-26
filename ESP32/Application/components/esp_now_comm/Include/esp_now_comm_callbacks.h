#include "esp_now_comm.h"

/**
 * @brief Callback for ESP-NOW send completion
 *
 * @details Called by ESP-NOW stack after each transmission attempt to report
 *          success or failure.
 *
 * @param[in] mac_addr 6-byte MAC address of the destination peer
 * @param[in] status ESP_NOW_SEND_SUCCESS if transmission succeeded,
 *                   ESP_NOW_SEND_FAIL if failed
 *
 * @return None
 */
void on_data_send_callback(const uint8_t *mac_addr, esp_now_send_status_t status);

/**
 * @brief Callback for ESP-NOW data reception
 *
 * @details Called by ESP-NOW stack asynchronously whenever a valid packet
 *          is received from a registered peer device. Can be used to parse
 *          protocol messages and update device state.
 *
 * @param[in] mac_addr 6-byte MAC address of the peer that sent the data
 * @param[in] data Pointer to the received data payload
 * @param[in] len Length of received data in bytes
 *
 * @return None
 */
void on_data_recv_callback(const uint8_t *mac_addr, const uint8_t *data, int len);