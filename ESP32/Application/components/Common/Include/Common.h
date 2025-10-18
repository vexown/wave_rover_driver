/**
 * @file Common.h
 * @brief Header file for common definitions and utilities
 */

#ifndef COMMON_H
#define COMMON_H

/*******************************************************************************/
/*                                INCLUDES                                     */
/*******************************************************************************/
#include <stdio.h>
#include "sdkconfig.h"
#include "comms_uart.h"

/*******************************************************************************/
/*                                 DEFINES                                     */
/*******************************************************************************/
/* Raspberry Pi is connected to the Wave Rover Driver ESP32 via UART. By default, printf uses UART0, which is both
 * connected to the USB serial port and the Raspberry Pi via pin headers. In this project the stdout was re-routed to UART1.
 * This allows us to use UART0 exclusively for communication with the Raspberry Pi. */
#define LOG_TO_RPI(format, ...) log_to_rpi(format, ##__VA_ARGS__)

/*******************************************************************************/
/*                               DATA TYPES                                    */
/*******************************************************************************/

/*******************************************************************************/
/*                            GLOBAL VARIABLES                                 */
/*******************************************************************************/

/*******************************************************************************/
/*                        GLOBAL FUNCTION DECLARATION                          */
/*******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Logs a formatted message to the Raspberry Pi via UART.
 *
 * This function formats a string using printf-style formatting and sends it
 * to the Raspberry Pi through UART communication. It uses a fixed-size buffer
 * to hold the formatted string before transmission.
 *
 * @param format A printf-style format string.
 * @param ... Variable arguments corresponding to the format string.
 */
void log_to_rpi(const char *format, ...);


#ifdef __cplusplus
}
#endif


#endif /* COMMON_H */
