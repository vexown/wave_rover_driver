/******************************************************************************
 * @file comms_uart.c
 * @brief UART communication component for ESP-IDF projects
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
#include "driver/uart.h"
#include "esp_log.h" // For logging

/* Project Includes */
#include "comms_uart.h"

/*******************************************************************************/
/*                                  MACROS                                     */
/*******************************************************************************/
/** Tag for logging (used in ESP_LOGI, ESP_LOGE, etc.) Example usage: 
 *  ESP_LOGI(TAG, "Log message which will be appended to the tag"); */
#define TAG "COMMS_UART"

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

/*******************************************************************************/
/*                             STATIC VARIABLES                                */
/*******************************************************************************/
static bool is_uart_initialized = false;

/*******************************************************************************/
/*                     GLOBAL FUNCTION DEFINITIONS                             */
/*******************************************************************************/

esp_err_t comms_uart_init(void)
{
    if (is_uart_initialized) 
    {
        ESP_LOGW(TAG, "UART already initialized.");
        return ESP_OK;
    }

    uart_config_t uart_config = 
    {
        .baud_rate = COMMS_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT, // Or UART_SCLK_APB
    };

    esp_err_t ret;

    // Install UART driver, and get the queue.
    ret = uart_driver_install(COMMS_UART_NUM, COMMS_UART_BUF_SIZE * 2, 0, 0, NULL, 0);
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    // Configure UART parameters
    ret = uart_param_config(COMMS_UART_NUM, &uart_config);
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed to configure UART parameters: %s", esp_err_to_name(ret));
        uart_driver_delete(COMMS_UART_NUM); // Clean up driver install
        return ESP_FAIL;
    }

    // Set UART pins (using configured defaults)
    ret = uart_set_pin(COMMS_UART_NUM, COMMS_UART_TX_PIN, COMMS_UART_RX_PIN, COMMS_UART_RTS_PIN, COMMS_UART_CTS_PIN);
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
        uart_driver_delete(COMMS_UART_NUM); // Clean up driver install
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "UART initialized successfully on port %d", COMMS_UART_NUM);
    is_uart_initialized = true;
    return ESP_OK;
}

esp_err_t comms_uart_deinit(void)
{
    if (!is_uart_initialized) 
    {
        ESP_LOGW(TAG, "UART not initialized, cannot deinit.");
        return ESP_OK; // Or ESP_FAIL depending on desired strictness
    }

    esp_err_t ret = uart_driver_delete(COMMS_UART_NUM);
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed to delete UART driver: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    is_uart_initialized = false;
    ESP_LOGI(TAG, "UART deinitialized successfully.");
    return ESP_OK;
}

int comms_uart_send(const uint8_t* data, size_t len)
{
    if (!is_uart_initialized) 
    {
        ESP_LOGE(TAG, "UART not initialized. Call comms_uart_init first.");
        return -1;
    }
    if (data == NULL || len == 0) 
    {
        ESP_LOGW(TAG, "Invalid arguments for sending data.");
        return -1;
    }

    const int txBytes = uart_write_bytes(COMMS_UART_NUM, data, len);
    if (txBytes < 0) 
    {
        ESP_LOGE(TAG, "UART write failed");
    } 
    else if (txBytes != len) 
    {
         ESP_LOGW(TAG, "UART write incomplete. Sent %d bytes out of %d", txBytes, len);
    }
    // Consider adding uart_wait_tx_done(COMMS_UART_NUM, portMAX_DELAY); if synchronous completion is critical
    return txBytes;
}

int comms_uart_receive(uint8_t* data, size_t len, uint32_t timeout_ms)
{
     if (!is_uart_initialized) 
     {
        ESP_LOGE(TAG, "UART not initialized. Call comms_uart_init first.");
        return -1;
    }
    if (data == NULL || len == 0) 
    {
        ESP_LOGW(TAG, "Invalid arguments for receiving data.");
        return -1;
    }

    // Convert ms timeout to FreeRTOS ticks
    TickType_t timeout_ticks = (timeout_ms == portMAX_DELAY) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);

    const int rxBytes = uart_read_bytes(COMMS_UART_NUM, data, len, timeout_ticks);
    if (rxBytes < 0) 
    {
        ESP_LOGE(TAG, "UART read failed");
        return -1; // Indicate error
    }
    // rxBytes == 0 indicates timeout with no data read, which is not necessarily an error.
    return rxBytes;
}


/*******************************************************************************/
/*                     STATIC FUNCTION DEFINITIONS                             */
/*******************************************************************************/
