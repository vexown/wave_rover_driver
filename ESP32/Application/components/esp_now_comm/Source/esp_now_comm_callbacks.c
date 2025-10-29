#include "esp_now_comm_callbacks.h"
#include "esp_log.h"
#include "string.h"
#include "NaviLogging.h"
#include "Common.h"

#define TAG "ESP_NOW_COMM_CALLBACK"

void on_data_send_callback(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    /* Determine if send succeeded or failed and log the result */
    const char *status_str = (status == ESP_NOW_SEND_SUCCESS) ? "SUCCESS" : "FAIL";
    LOG_TO_RPI("Send to %02x:%02x:%02x:%02x:%02x:%02x: %s", 
             mac_addr[0], mac_addr[1], mac_addr[2], 
             mac_addr[3], mac_addr[4], mac_addr[5], status_str);
    
    /* Here you could implement retry logic, update statistics, etc.
     * For example: increment failure counter if status is FAIL
     */
}

void on_data_recv_callback(const uint8_t *mac_addr, const uint8_t *data, int len)
{
    LOG_TO_RPI("Received data from %02x:%02x:%02x:%02x:%02x:%02x, length: %d bytes", 
             mac_addr[0], mac_addr[1], mac_addr[2], 
             mac_addr[3], mac_addr[4], mac_addr[5], len);
    
    /* Route the received data to the appropriate handler based on sender MAC address */
    if (memcmp(mac_addr, navi_esp32_mac, 6) == 0)
    {
        /* Data from NaviLogging sender - pass to NaviLogging handler */
        LOG_TO_RPI("Data from NaviLogging sender, routing to NaviLogging_handle_received_coords");
        NaviLogging_handle_received_coords(mac_addr, data, len);
    }
    else
    {
        /* Data from unknown or unregistered sender */
        LOG_TO_RPI("Received data from unregistered sender: %02x:%02x:%02x:%02x:%02x:%02x", 
                 mac_addr[0], mac_addr[1], mac_addr[2], 
                 mac_addr[3], mac_addr[4], mac_addr[5]);
    }
}