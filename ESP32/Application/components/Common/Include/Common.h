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
#ifdef CONFIG_COMPILER_OPTIMIZATION_DEBUG // Check if debug build
    #define DEBUG_BUILD 1
    #define LOG printf // Enable prints on debug build
#else
    #define DEBUG_BUILD 0
    #define LOG(...) // Disable prints on release build - they can cause issues especially when running an RTOS
#endif

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