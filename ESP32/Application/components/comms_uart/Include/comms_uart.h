/******************************************************************************
 * @file comms_uart.h
 * @brief Header file for the UART Communication component
 *
 ******************************************************************************/

#ifndef COMMS_UART_H
#define COMMS_UART_H

/*******************************************************************************/
/*                                 INCLUDES                                    */
/*******************************************************************************/
/* C Standard Libraries */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h> // For size_t

/* ESP-IDF Libraries */
#include "esp_err.h"
#include "driver/uart.h" // Include UART driver header

/*******************************************************************************/
/*                                  MACROS                                     */
/*******************************************************************************/

/* UART Configuration */
/* !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! WARNING !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */
/* On ESP32 UART0 is usually mapped to standard output (so things like printf)  */
/* If you use UART0 with this component, you may interfere with standard output */
/* !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */
#define COMMS_UART_NUM          UART_NUM_0
#define COMMS_UART_TX_PIN       (UART_PIN_NO_CHANGE) // Use default TX pin (GPIO1)
#define COMMS_UART_RX_PIN       (UART_PIN_NO_CHANGE) // Use default RX pin (GPIO3)
#define COMMS_UART_RTS_PIN      (UART_PIN_NO_CHANGE) // No RTS
#define COMMS_UART_CTS_PIN      (UART_PIN_NO_CHANGE) // No CTS
#define COMMS_UART_BAUD_RATE    (921600)
#define COMMS_UART_BUF_SIZE     (1024)

/*******************************************************************************/
/*                                DATA TYPES                                   */
/*******************************************************************************/

/*******************************************************************************/
/*                     GLOBAL VARIABLES DECLARATIONS                           */
/*******************************************************************************/

/*******************************************************************************/
/*                     GLOBAL FUNCTION DECLARATIONS                            */
/*******************************************************************************/
/*   for functions defined in the corresponding .c file, for use in other .c   */
/*    files just by including this header file. Extern is a default linkage    */
/*    specifier for functions, so it is not necessary to use it explicitly.    */
/*******************************************************************************/

/**
 * @brief Initialize the UART communication component.
 *
 * @details Configures and installs the UART driver for UART0 with predefined
 *          settings (baud rate, pins, buffer size).
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_FAIL on error during configuration or installation
 */
esp_err_t comms_uart_init(void);

/**
 * @brief Deinitialize the UART communication component.
 *
 * @details Deletes the UART driver.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_FAIL if driver was not installed
 */
esp_err_t comms_uart_deinit(void);

/**
 * @brief Send data over UART.
 *
 * @param[in] data Pointer to the data buffer to send.
 * @param[in] len Length of the data to send in bytes.
 *
 * @return
 *      - Number of bytes written on success
 *      - -1 on error
 */
int comms_uart_send(const uint8_t* data, size_t len);

/**
 * @brief Receive data from UART.
 *
 * @param[out] data Pointer to the buffer where received data will be stored.
 * @param[in] len Maximum number of bytes to read into the buffer.
 * @param[in] timeout_ms Timeout in milliseconds to wait for data.
 *
 * @return
 *      - Number of bytes read on success
 *      - 0 if timeout occurred before any data was received
 *      - -1 on error
 */
int comms_uart_receive(uint8_t* data, size_t len, uint32_t timeout_ms);


#endif /* COMMS_UART_H */