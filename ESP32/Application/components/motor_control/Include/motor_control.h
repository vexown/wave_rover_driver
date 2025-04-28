/******************************************************************************
 * @file motor_control.h
 * @brief Header file for the Template component
 *
 ******************************************************************************/

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

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

/**
 * @brief Initialize motor control GPIOs and LEDC peripherals.
 *
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t motor_init(void);

/**
 * @brief Set the speed and direction for both motors.
 *
 * @param left_motors_pwm PWM duty cycle for the left motor (-255 to 255). Negative values indicate reverse direction.
 * @param right_motors_pwm PWM duty cycle for the right motor (-255 to 255). Negative values indicate reverse direction.
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t motor_set_speed(int left_motors_pwm, int right_motors_pwm);

/**
 * @brief Stop both motors immediately.
 *
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t motor_stop(void);

/**
 * @brief Move the rover forward.
 *
 * @param pwm PWM duty cycle (0 to 255).
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t motor_move_forward(int pwm);

/**
 * @brief Move the rover backward.
 *
 * @param pwm PWM duty cycle (0 to 255).
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t motor_move_backward(int pwm);

/**
 * @brief Turn the rover left (on the spot).
 *
 * @param pwm PWM duty cycle (0 to 255).
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t motor_turn_left(int pwm);

/**
 * @brief Turn the rover right (on the spot).
 *
 * @param pwm PWM duty cycle (0 to 255).
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t motor_turn_right(int pwm);


#endif /* MOTOR_CONTROL_H */