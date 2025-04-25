#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "esp_err.h"

/**
 * @brief Initialize motor control GPIOs and LEDC peripherals.
 *
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t motor_init(void);

/**
 * @brief Set the speed and direction for both motors.
 *
 * @param left_pwm PWM duty cycle for the left motor (-255 to 255). Negative values indicate reverse direction.
 * @param right_pwm PWM duty cycle for the right motor (-255 to 255). Negative values indicate reverse direction.
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t motor_set_speed(int left_pwm, int right_pwm);

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


#endif // MOTOR_CONTROL_H