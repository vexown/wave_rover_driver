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


#endif // MOTOR_CONTROL_H