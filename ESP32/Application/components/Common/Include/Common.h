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

/*******************************************************************************/
/*                                 DEFINES                                     */
/*******************************************************************************/
/* Raspberry Pi is connected to the Wave Rover Driver ESP32 via UART. By default, printf uses UART0, which is both
 * connected to the USB serial port and the Raspberry Pi via pin headers. It means that
 * printf will print to both the USB serial port and the Raspberry Pi. In our case, we don't really
 * care about the USB serial port, so we can use printf as a way of sending logs to the Pi. */
#define LOG_TO_RPI printf 
                         
/*******************************************************************************/
/*                               DATA TYPES                                    */
/*******************************************************************************/

/*******************************************************************************/
/*                            GLOBAL VARIABLES                                 */
/*******************************************************************************/

/*******************************************************************************/
/*                        GLOBAL FUNCTION DELCARATION                          */
/*******************************************************************************/


#endif /* COMMON_H */