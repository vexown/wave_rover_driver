/******************************************************************************
 * @file i2c_manager.h
 * @brief Header file for the I2C Manager component
 *
 ******************************************************************************/

#ifndef I2C_MANAGER_H
#define I2C_MANAGER_H

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
#include "driver/i2c_master.h"

/*******************************************************************************/
/*                                  MACROS                                     */
/*******************************************************************************/
/* Default I2C configuration parameters based on the Waveshare General Driver for Robots
   board and the sensors connected to it. */
#define I2C_MANAGER_DEFAULT_PORT I2C_NUM_0  // Default I2C port number
#define I2C_MANAGER_DEFAULT_SDA GPIO_NUM_32 // Default SDA pin
#define I2C_MANAGER_DEFAULT_SCL GPIO_NUM_33 // Default SCL pin
#define I2C_MANAGER_DEFAULT_FREQ 400000 // Default I2C frequency in Hz (400kHz)

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

/**
 * @brief Initialize the I2C manager with the specified I2C port and GPIO pins.
 *
 * This function configures and then allocates an I2C master bus.
 *
 * @param i2c_port The I2C port to use (e.g., I2C_NUM_0, I2C_NUM_1).
 * @param sda_pin The GPIO pin number for the SDA line.
 * @param scl_pin The GPIO pin number for the SCL line.
 *
 * @return
 *     - ESP_OK on success
 *     - error code on failure
 */
esp_err_t i2c_manager_init(i2c_port_t i2c_port, gpio_num_t sda_pin, gpio_num_t scl_pin);


/**
 * @brief Function for retrieving the I2C bus handle.
 * 
 * @param bus_handle Pointer to a variable where the I2C bus handle will be stored.
 * 
 * @return
 *     - ESP_OK on success
 *     - ESP_ERR_INVALID_ARG if the bus_handle pointer is NULL
 *     - ESP_ERR_INVALID_STATE if the I2C bus has not been initialized
 *     - ESP_FAIL on other errors
 */
esp_err_t i2c_manager_get_bus_handle(i2c_master_bus_handle_t *bus_handle);


#endif /* I2C_MANAGER_H */