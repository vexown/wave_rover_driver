/******************************************************************************
 * @file web_server.h
 * @brief Header file for the Template component
 *
 ******************************************************************************/

#ifndef WEB_SERVER_H
#define WEB_SERVER_H

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

/*******************************************************************************/
/*                                  MACROS                                     */
/*******************************************************************************/

/*******************************************************************************/
/*                                DATA TYPES                                   */
/*******************************************************************************/

/*******************************************************************************/
/*                     GLOBAL VARIABLES DECLARATIONS                           */
/*******************************************************************************/
/*   for variables defined in the corresponding .c file, for use in other .c   */
/*      files just by including this header file. Use extern for these.        */
/*******************************************************************************/

/*******************************************************************************/
/*                     GLOBAL FUNCTION DECLARATIONS                            */
/*******************************************************************************/
/*   for functions defined in the corresponding .c file, for use in other .c   */
/*    files just by including this header file. Extern is a default linkage    */
/*    specifier for functions, so it is not necessary to use it explicitly.    */
/*******************************************************************************/

/* Allow including from C++ files */
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize WiFi in SoftAP mode and start the HTTP web server.
 *
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t web_server_init(void);

/**
 * @brief Print a message to the web server. Can be used for debugging or status updates.
 *
 * @param message The null-terminated string containing the status message of a length
 *                less than WEBSERVER_PRINT_MAX_LEN.
 *                If the message is longer than WEBSERVER_PRINT_MAX_LEN, it will be truncated.
 *                If the message is NULL, a default error message will be set.
 */
void web_server_print(const char *message);

/**
 * @brief Gets the current IP address assigned to the ESP32 in STA mode.
 *
 * @param ip_buffer Buffer to store the IP address string.
 * @param buffer_size Size of the ip_buffer.
 * @return esp_err_t ESP_OK if IP address was copied, ESP_FAIL otherwise (e.g., not connected).
 */
esp_err_t web_server_get_ip(char *ip_buffer, size_t buffer_size);

/**
 * @brief Gets the current WiFi connection status.
 *
 * @return bool true if connected, false otherwise.
 */
bool web_server_is_connected(void);

/**
 * @brief Broadcasts the current IMU orientation over WebSocket.
 *
 * @param roll The roll angle in degrees.
 * @param pitch The pitch angle in degrees.
 * @param yaw The yaw angle in degrees.
 */
void web_server_ws_broadcast_imu_orientation(float roll, float pitch, float yaw);

#ifdef __cplusplus
}
#endif

#endif /* WEB_SERVER_H */