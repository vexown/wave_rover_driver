/******************************************************************************
 * @file NaviLogging.h
 * @brief Header file for the Navigation Logging component
 *
 ******************************************************************************/

#ifndef NAVILOGGING_H
#define NAVILOGGING_H

/*******************************************************************************/
/*                                 INCLUDES                                    */
/*******************************************************************************/
/*     Include headers required for the declarations in *this* header file     */
/*                 (e.g., types used in function prototypes).                  */
/*       Prefer forward declarations over full includes where possible         */
/*             to minimize dependencies (for structs, enums etc.).             */
/*******************************************************************************/

/* C Standard Libraries */
#include <stdint.h>
#include <stdbool.h> 

/* ESP-IDF Libraries */
#include "esp_err.h"
#include "esp_now.h"

/*******************************************************************************/
/*                                  MACROS                                     */
/*******************************************************************************/

/*******************************************************************************/
/*                                DATA TYPES                                   */
/*******************************************************************************/

/* Structure to hold navigation coordinates and related data.
 * It has to match the structure used in the ESP-NOW communication on the sender side (the Navigation App ESP32 device).
 */
typedef struct 
{
    double latitude;                        /*!< Latitude in decimal degrees */
    double longitude;                       /*!< Longitude in decimal degrees */
    double altitude;                        /*!< Altitude in meters */
    int quality_indicator;                  /*!< GPS quality indicator (0-6): 0=No fix, 1/2=2D/3D fix, 3=PPS, 4=RTK fixed, etc. */
    int number_of_satellites;               /*!< Number of satellites used in the position fix (0-12+) */
    double horizontal_dilution_of_precision; /*!< Horizontal Dilution of Precision (HDOP) - lower values indicate better accuracy */
} navi_coordinates_type;

/*******************************************************************************/
/*                     GLOBAL VARIABLES DECLARATIONS                           */
/*******************************************************************************/
/*   for variables defined in the corresponding .c file, for use in other .c   */
/*      files just by including this header file. Use extern for these.        */
/*******************************************************************************/
extern uint8_t navi_esp32_mac[ESP_NOW_ETH_ALEN];

/*******************************************************************************/
/*                     GLOBAL FUNCTION DECLARATIONS                            */
/*******************************************************************************/
/*   for functions defined in the corresponding .c file, for use in other .c   */
/*    files just by including this header file. Extern is a default linkage    */
/*    specifier for functions, so it is not necessary to use it explicitly.    */
/*******************************************************************************/

/**
 * @brief Initialize the NaviLogging component.
 *
 * @details Initializes ESP-NOW communication via esp_now_comm, registers the
 *          receive callback, adds the sender as a peer, and starts the
 *          coordinates processing task.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_FAIL if initialization fails
 *
 * @note Must be called after Wi-Fi is initialized
 */
esp_err_t NaviLogging_init(void);

/**
 * @brief Get the last received coordinates.
 *
 * @details This function returns the most recently received coordinates
 *          from ESP-NOW communication.
 *
 * @param[out] coordinates Pointer to store the coordinates data
 *
 * @return
 *      - ESP_OK if valid coordinates are available
 *      - ESP_ERR_NOT_FOUND if no coordinates have been received yet
 *      - ESP_ERR_INVALID_ARG if coordinates pointer is NULL
 */
esp_err_t NaviLogging_get_last_coordinates(navi_coordinates_type *coordinates);

/**
 * @brief Check if new coordinates data is available.
 *
 * @details This function checks if new coordinates have been received
 *          since the last call to NaviLogging_get_last_coordinates().
 *          The check is based on a flag that is set when new data is received
 *          in the ESP-NOW receive callback.
 *
 * @return
 *      - true if new data is available
 *      - false if no new data since last check
 */
bool NaviLogging_is_new_data_available(void);

/**
 * @brief ESP-NOW data receive callback for NaviLogging.
 * 
 * @param[in] mac_addr MAC address of the sender device
 * @param[in] data Pointer to received data buffer
 * @param[in] len Length of received data in bytes
 *
 * @return None
 */
void NaviLogging_handle_received_coords(const uint8_t *mac_addr, const uint8_t *data, int len);

#endif /* NAVILOGGING_H */