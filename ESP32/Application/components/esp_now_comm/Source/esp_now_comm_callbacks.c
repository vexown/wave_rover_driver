#include "esp_now_comm_callbacks.h"
#include "esp_log.h"

#define TAG "ESP_NOW_COMM_CALLBACK"

void on_data_send_callback(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    /* Determine if send succeeded or failed and log the result */
    const char *status_str = (status == ESP_NOW_SEND_SUCCESS) ? "SUCCESS" : "FAIL";
    ESP_LOGI(TAG, "Send to %02x:%02x:%02x:%02x:%02x:%02x: %s", 
             mac_addr[0], mac_addr[1], mac_addr[2], 
             mac_addr[3], mac_addr[4], mac_addr[5], status_str);
    
    /* Here you could implement retry logic, update statistics, etc.
     * For example: increment failure counter if status is FAIL
     */
}

void on_data_recv_callback(const uint8_t *mac_addr, const uint8_t *data, int len)
{
    /* Log the reception event with peer MAC address and data length */
    ESP_LOGI(TAG, "Received %d bytes from %02x:%02x:%02x:%02x:%02x:%02x", 
             len, mac_addr[0], mac_addr[1], mac_addr[2], 
             mac_addr[3], mac_addr[4], mac_addr[5]);
    
    /* Here you could parse the received data and take action
     * For example: deserialize protocol messages, update device state, etc.
     */
}