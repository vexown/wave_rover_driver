/******************************************************************************
 * @file motor_control.c
 * @brief Template component for ESP-IDF projects
 * 
 ******************************************************************************/

/*******************************************************************************/
/*                                 INCLUDES                                    */
/*******************************************************************************/
/*    Include headers required for the definitions/implementation in *this*    */
/* source file. This typically includes this module's own header("template.h") */
/*      and any headers needed for function bodies, static variables, etc.     */
/*******************************************************************************/
/* C Standard Libraries */
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

/* ESP-IDF Includes */
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"

/* Project Includes */
#include "motor_control.h"
#include "web_server.h"

/*******************************************************************************/
/*                                  MACROS                                     */
/*******************************************************************************/
/** Tag for logging (used in ESP_LOGI, ESP_LOGE, etc.) Example usage: 
 *  ESP_LOGI(TAG, "Log message which will be appended to the tag"); */
#define TAG "MOTOR_CONTROL"

/* Motor PWM Pins */
#define LEFT_MOTOR_A_PWMA_PIN       GPIO_NUM_25 // Controls the speed of motor A (left motor) via PWM
#define RIGHT_MOTOR_B_PWMB_PIN      GPIO_NUM_26 // Controls the speed of motor B (right motor) via PWM

/* Motor Direction Pins */
/* The Waveshare General Driver for Robots uses the TB6612FNG motor driver.
 * See details and wiring diagram here: https://files.waveshare.com/upload/3/37/General_Driver_for_Robots.pdf
 *
 * These pins control the direction and state of the two motor sets (A and B) connected to the TB6612FNG driver.
 * In reality there are 4 motors, but they are paired into two sets (A and B). Meaning left motors A1 and A2
 * are wired in parallel and controlled by the same pins (AIN1, AIN2) and the same goes for right motors B1 and B2.
 * 
 * Motor A Control (Pins AIN1, AIN2) (Left Motors):
 * - AIN1=1, AIN2=0: Motor A Forward
 * - AIN1=0, AIN2=1: Motor A Reverse
 * - AIN1=1, AIN2=1: Motor A Short Brake (Actively braking)
 * - AIN1=0, AIN2=0: Motor A Stop (Coast/Freewheel)
 * 
 * Motor B Control (Pins BIN1, BIN2) (Right Motors):
 * - BIN1=1, BIN2=0: Motor B Forward
 * - BIN1=0, BIN2=1: Motor B Reverse
 * - BIN1=1, BIN2=1: Motor B Short Brake (Actively braking)
 * - BIN1=0, BIN2=0: Motor B Stop (Coast/Freewheel)
 * 
 * Note: The actual direction labeled "Forward" or "Reverse" depends on the 
 * specific wiring of the motor terminals. The key is that (1,0) and (0,1) 
 * result in opposite rotation directions.
 */
#define LEFT_MOTOR_A_AIN1_PIN       GPIO_NUM_21 
#define LEFT_MOTOR_A_AIN2_PIN       GPIO_NUM_17
#define RIGHT_MOTOR_B_BIN1_PIN      GPIO_NUM_22
#define RIGHT_MOTOR_B_BIN2_PIN      GPIO_NUM_23

/* LEDC Configuration. LEDC is this poorly named module that provides PWM functionality on ESP32. 
 * It is not strictly used for LEDs, but rather for any GPIO pin that needs PWM control.*/

/* LEDC TIMER - You configure an LEDC timer (e.g., `LEDC_TIMER_0`, `LEDC_TIMER_1`, etc.) with specific properties like frequency and resolution (duty cycle precision). 
 * Think of a timer as setting the underlying pulse rate and how finely you can control the pulse width.*/
#define LEDC_TIMER                  LEDC_TIMER_0 

/* LEDC MODE - ESP32 has two modes: high-speed and low-speed. High-speed mode is typically used for applications requiring fast PWM signals, 
 * while low-speed mode is used for slower applications like motor control. */
#define LEDC_MODE                   LEDC_LOW_SPEED_MODE 

/* LEDC CHANNEL - Each timer can have multiple channels. Each channel can be assigned to a different GPIO pin for PWM output. 
 * You configure an LEDC channel (e.g., `LEDC_CHANNEL_0`, `LEDC_CHANNEL_1`, etc.) to control a specific GPIO pin.
 * The channel is later configured in a ledc_channel_config_t structure where it is assigned to a specific GPIO pin and specific timer and some other properties are set. */
#define LEFT_MOTOR_LEDC_CHANNEL_A   LEDC_CHANNEL_5 
#define RIGHT_MOTOR_LEDC_CHANNEL_B  LEDC_CHANNEL_6 

/* LEDC DUTY RESOLUTION - This defines the resolution of the PWM signal. 
 * For example, 8-bit resolution means the duty cycle can be set from 0 to 255 (256 levels). */
#define LEDC_DUTY_RES               LEDC_TIMER_8_BIT 

/* LEDC DUTY MAX - This is the maximum value for the duty cycle. For 8-bit resolution, this is 255. 
 * This means that the maximum duty cycle is 100% (fully on). */
#define LEDC_DUTY_MAX               (255)  

/* INITIAL LEDC DUTY - This is the initial duty cycle value. It is set to 0, meaning the PWM signal starts off (0% duty cycle). */
#define INITIAL_LEDC_DUTY           (0)

/* LEDC_HPOINT - Specifies the timer count value within a PWM cycle at which the output signal transitions to HIGH.
 * The valid range is [0, (2**LEDC_DUTY_RES)-1].
 * - A value of 0 means the output goes HIGH at the beginning of the timer cycle.
 * - A non-zero value introduces a phase shift, delaying the start of the HIGH pulse.
 * This is used primarily for phase-shifting PWM signals, e.g., in motor control or synchronized lighting. */
#define LEDC_HPOINT                 (0)

/* LEDC FREQUENCY - This is the frequency of the PWM signal. The frequency is set to 100kHz, which is a common frequency for motor control applications. 
 * This means that the PWM signal will toggle at this frequency, allowing for smooth control of the motor speed. */
#define LEDC_FREQUENCY              (100000)   

/*******************************************************************************/
/*                                DATA TYPES                                   */
/*******************************************************************************/

/*******************************************************************************/
/*                     GLOBAL FUNCTION DECLARATIONS                            */
/*******************************************************************************/
/*  for function defined in some other .c files, to be used here. Use extern.  */
/*******************************************************************************/

/*******************************************************************************/
/*                     GLOBAL VARIABLES DECLARATIONS                           */
/*******************************************************************************/
/*  for variables defined in some other .c files, to be used here. Use extern. */
/*******************************************************************************/

/*******************************************************************************/
/*                     GLOBAL VARIABLES DEFINITIONS                            */
/*******************************************************************************/
/*    for variables defined in this .c file, to be used in other .c files.     */
/*******************************************************************************/

/*******************************************************************************/
/*                     STATIC FUNCTION DECLARATIONS                            */
/*******************************************************************************/

/**
 * @brief Set the speed and direction for both motors.
 *
 * @param left_motors_pwm PWM duty cycle for the left motor (-255 to 255). Negative values indicate reverse direction.
 * @param right_motors_pwm PWM duty cycle for the right motor (-255 to 255). Negative values indicate reverse direction.
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
static esp_err_t motor_set_speed(int left_motors_pwm, int right_motors_pwm);

/**
 * @brief Controls the left motor's direction and speed.
 *
 * Sets the direction pins (AIN1, AIN2) based on the sign of pwm_val
 * and sets the PWM duty cycle based on the absolute value of pwm_val.
 *
 * @param pwm_val The desired PWM value for the left motor.
 *                Positive values indicate forward direction, negative values
 *                indicate backward direction. The magnitude determines the speed
 *                (clamped between 0 and LEDC_DUTY_MAX).
 * @return esp_err_t ESP_OK on success, or an error code from the LEDC driver
 *                   if setting the duty cycle fails.
 */
static esp_err_t left_motors_control(int pwm_val);

/**
 * @brief Controls the right motor's direction and speed.
 *
 * Sets the direction pins (BIN1, BIN2) based on the sign of pwm_val
 * and sets the PWM duty cycle based on the absolute value of pwm_val.
 *
 * @param pwm_val The desired PWM value for the right motor.
 *                Positive values indicate forward direction, negative values
 *                indicate backward direction. The magnitude determines the speed
 *                (clamped between 0 and LEDC_DUTY_MAX).
 * @return esp_err_t ESP_OK on success, or an error code from the LEDC driver
 *                   if setting the duty cycle fails.
 */
static esp_err_t right_motors_control(int pwm_val);

/**
 * @brief Sets the direction pins for the left motors to reverse.
 *
 * Sets AIN1 to LOW and AIN2 to HIGH.
 *
 * @return esp_err_t ESP_OK on success, or an error code from the GPIO driver.
 */
static esp_err_t left_motors_set_reverse_dir(void);

/**
 * @brief Sets the direction pins for the left motors to forward.
 *
 * Sets AIN1 to HIGH and AIN2 to LOW.
 *
 * @return esp_err_t ESP_OK on success, or an error code from the GPIO driver.
 */
static esp_err_t left_motors_set_forward_dir(void);

/**
 * @brief Sets the direction pins for the right motors to reverse.
 *
 * Sets BIN1 to LOW and BIN2 to HIGH.
 *
 * @return esp_err_t ESP_OK on success, or an error code from the GPIO driver.
 */
static esp_err_t right_motors_set_reverse_dir(void);

/**
 * @brief Sets the direction pins for the right motors to forward.
 *
 * Sets BIN1 to HIGH and BIN2 to LOW.
 *
 * @return esp_err_t ESP_OK on success, or an error code from the GPIO driver.
 */
static esp_err_t right_motors_set_forward_dir(void);

/**
 * @brief Sets the direction pins for all motors to coast (stop/freewheel).
 *
 * Sets AIN1, AIN2, BIN1, and BIN2 to LOW.
 *
 * @return esp_err_t ESP_OK on success, or an error code from the GPIO driver.
 */
static esp_err_t all_motors_set_coast(void);

/**
 * @brief Sets the direction pins for all motors to brake (short brake).
 *
 * Sets AIN1, AIN2, BIN1, and BIN2 to HIGH.
 *
 * @return esp_err_t ESP_OK on success, or an error code from the GPIO driver.
 */
static esp_err_t all_motors_set_brake(void);

/*******************************************************************************/
/*                             STATIC VARIABLES                                */
/*******************************************************************************/
/* Mutex for accessing any of the motor control functions. */
static SemaphoreHandle_t motor_mutex = NULL;

/*******************************************************************************/
/*                     GLOBAL FUNCTION DEFINITIONS                             */
/*******************************************************************************/
esp_err_t motor_init(void) 
{
    ESP_LOGI(TAG, "Initializing motor control");

    motor_mutex = xSemaphoreCreateMutex();
    if (motor_mutex == NULL) 
    {
        ESP_LOGE(TAG, "Failed to create motor mutex");
        return ESP_FAIL;
    }

    /* #01 - Define and apply the GPIO configuration for pins which are used for motor control, specifically
     * the direction control pins (AIN1, AIN2 for left motors and BIN1, BIN2 for right motors) */
    gpio_config_t dir_control_gpio_config = 
    {
        .pin_bit_mask = (1ULL << LEFT_MOTOR_A_AIN1_PIN) | (1ULL << LEFT_MOTOR_A_AIN2_PIN) |
                        (1ULL << RIGHT_MOTOR_B_BIN1_PIN) | (1ULL << RIGHT_MOTOR_B_BIN2_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    /* Apply the GPIO configuration */
    esp_err_t err = gpio_config(&dir_control_gpio_config);
    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG, "GPIO config failed: %s", esp_err_to_name(err));
        return err;
    }

    /* #02 - Set the initial state of the direction control pins to LOW (coast/stop) */
    gpio_set_level(LEFT_MOTOR_A_AIN1_PIN, 0);
    gpio_set_level(LEFT_MOTOR_A_AIN2_PIN, 0);
    gpio_set_level(RIGHT_MOTOR_B_BIN1_PIN, 0);
    gpio_set_level(RIGHT_MOTOR_B_BIN2_PIN, 0);

    /* #03 - Define and apply the LEDC configuration for PWM control of the motors */
    ledc_timer_config_t ledc_timer = 
    {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    /* Apply the LEDC timer configuration */
    err = ledc_timer_config(&ledc_timer);
    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG, "LEDC timer config failed: %s", esp_err_to_name(err));
        return err;
    }

    /* #04 - Define and apply the LEDC channel configuration for PWM control (speed) of each motor set (A and B) */
    /* Configuration of the left motors (A) */
    ledc_channel_config_t left_motor_ledc_channel_A = 
    {
        .speed_mode     = LEDC_MODE,
        .channel        = LEFT_MOTOR_LEDC_CHANNEL_A,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEFT_MOTOR_A_PWMA_PIN,
        .duty           = INITIAL_LEDC_DUTY,
        .hpoint         = LEDC_HPOINT
    };
    /* Apply the LEDC channel configuration for left motors (A) */
    err = ledc_channel_config(&left_motor_ledc_channel_A);
    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG, "LEDC channel A config failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Configuration of the right motors (B) */
    ledc_channel_config_t right_motor_ledc_channel_B = 
    {
        .speed_mode     = LEDC_MODE,
        .channel        = RIGHT_MOTOR_LEDC_CHANNEL_B,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = RIGHT_MOTOR_B_PWMB_PIN,
        .duty           = INITIAL_LEDC_DUTY,
        .hpoint         = LEDC_HPOINT
    };
    /* Apply the LEDC channel configuration for right motors (B) */
    err = ledc_channel_config(&right_motor_ledc_channel_B);
    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG, "LEDC channel B config failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Motor control initialized successfully");
    return ESP_OK;
}

esp_err_t motor_stop(void) 
{
    if (motor_mutex == NULL) return ESP_ERR_INVALID_STATE;

    if (xSemaphoreTake(motor_mutex, portMAX_DELAY) == pdTRUE)
    {
        ESP_LOGI(TAG, "Stopping motors");
        web_server_print("Stopping motors");

        /* Set all motors to brake (active braking). You could also use coast (freewheel) if desired. */
        esp_err_t err_brake = all_motors_set_brake();
        if (err_brake != ESP_OK) 
        {
            ESP_LOGE(TAG, "Failed to set motors to brake: %s", esp_err_to_name(err_brake));
            xSemaphoreGive(motor_mutex); 
            return err_brake;
        }

        /* Set the duty cycle of both left motors (A) to 0 (stop) */
        esp_err_t duty_err_set_A = ledc_set_duty(LEDC_MODE, LEFT_MOTOR_LEDC_CHANNEL_A, 0);
        if (duty_err_set_A != ESP_OK) 
        {
            ESP_LOGE(TAG, "Failed to set left motor duty to 0: %s", esp_err_to_name(duty_err_set_A));
            xSemaphoreGive(motor_mutex); 
            return duty_err_set_A;
        }
        esp_err_t duty_err_update_A = ledc_update_duty(LEDC_MODE, LEFT_MOTOR_LEDC_CHANNEL_A);
        if (duty_err_update_A != ESP_OK) 
        {
            ESP_LOGE(TAG, "Failed to update left motor duty: %s", esp_err_to_name(duty_err_update_A));
            xSemaphoreGive(motor_mutex); 
            return duty_err_update_A;
        }
        /* Set the duty cycle of both right motors (B) to 0 (stop) */
        esp_err_t duty_err_set_B = ledc_set_duty(LEDC_MODE, RIGHT_MOTOR_LEDC_CHANNEL_B, 0);
        if (duty_err_set_B != ESP_OK) 
        {
            ESP_LOGE(TAG, "Failed to set right motor duty to 0: %s", esp_err_to_name(duty_err_set_B));
            xSemaphoreGive(motor_mutex); 
            return duty_err_set_B;
        }
        esp_err_t duty_err_update_B = ledc_update_duty(LEDC_MODE, RIGHT_MOTOR_LEDC_CHANNEL_B);
        if (duty_err_update_B != ESP_OK) 
        {
            ESP_LOGE(TAG, "Failed to update right motor duty: %s", esp_err_to_name(duty_err_update_B));
            xSemaphoreGive(motor_mutex); 
            return duty_err_update_B;
        }
    }
    else
    {
        ESP_LOGE(TAG, "Failed to take motor mutex");
        return ESP_ERR_TIMEOUT;
    }

    ESP_LOGI(TAG, "All motors stopped successfully");
    xSemaphoreGive(motor_mutex); 
    return ESP_OK;
}

esp_err_t motor_move_forward(int pwm)
{
    if (motor_mutex == NULL) return ESP_ERR_INVALID_STATE;

    if (xSemaphoreTake(motor_mutex, portMAX_DELAY) == pdTRUE)
    {
        ESP_LOGI(TAG, "Moving forward with PWM: %d", pwm);
        web_server_print("Moving forward");

        esp_err_t err = motor_set_speed(pwm, pwm); 
        xSemaphoreGive(motor_mutex);
        return err;
    }
    else
    {
        ESP_LOGE(TAG, "Failed to take motor mutex in motor_move_forward");
        return ESP_ERR_TIMEOUT;
    }
}

esp_err_t motor_move_backward(int pwm)
{
    if (motor_mutex == NULL) return ESP_ERR_INVALID_STATE;
    
    if (xSemaphoreTake(motor_mutex, portMAX_DELAY) == pdTRUE)
    {
        ESP_LOGI(TAG, "Moving backward with PWM: %d", pwm);
        web_server_print("Moving backward");

        /* Allow the flexibility of providing either positive or negative PWM values for backward movement.
        * If the user provides a positive value to this function, we convert it to negative for backward movement 
        * since we assume the user wants to move backward which requires negative PWM values in our code logic. */
        int speed = (pwm < 0) ? pwm : -pwm;

        esp_err_t err = motor_set_speed(speed, speed);
        xSemaphoreGive(motor_mutex);
        return err;
    }
    else
    {
        ESP_LOGE(TAG, "Failed to take motor mutex in motor_move_backward");
        return ESP_ERR_TIMEOUT;
    }
}

esp_err_t motor_turn_left(int pwm)
{
    if (motor_mutex == NULL) return ESP_ERR_INVALID_STATE;
    
    if (xSemaphoreTake(motor_mutex, portMAX_DELAY) == pdTRUE)
    {
        ESP_LOGI(TAG, "Turning left with PWM: %d", pwm);
        web_server_print("Turning left");

        /* Going left means the left motor should go backward and the right motor should go forward.
        * We take the absolute value of the provided PWM as the speed of the turning. We set the sign 
        * for each motor accordingly to achieve the desired turning effect. */
        int speed = abs(pwm);

        esp_err_t err = motor_set_speed(-speed, speed);
        xSemaphoreGive(motor_mutex);
        return err;
    }
    else
    {
        ESP_LOGE(TAG, "Failed to take motor mutex in motor_turn_left");
        return ESP_ERR_TIMEOUT;
    }
}

esp_err_t motor_turn_right(int pwm)
{
    if (motor_mutex == NULL) return ESP_ERR_INVALID_STATE;
    
    if (xSemaphoreTake(motor_mutex, portMAX_DELAY) == pdTRUE)
    {
        ESP_LOGI(TAG, "Turning right with PWM: %d", pwm);
        web_server_print("Turning right");

        /* Going right means the left motor should go forward and the right motor should go backward.
        * We take the absolute value of the provided PWM as the speed of the turning. We set the sign 
        * for each motor accordingly to achieve the desired turning effect. */
        int speed = abs(pwm);

        esp_err_t err = motor_set_speed(speed, -speed);
        xSemaphoreGive(motor_mutex);
        return err;
    }
    else
    {
        ESP_LOGE(TAG, "Failed to take motor mutex in motor_turn_right");
        return ESP_ERR_TIMEOUT;
    }
}

/*******************************************************************************/
/*                     STATIC FUNCTION DEFINITIONS                             */
/*******************************************************************************/


static esp_err_t motor_set_speed(int left_motors_pwm, int right_motors_pwm) 
{
    /* Clamping the PWM values to the range of -LEDC_DUTY_MAX (max reverse) to LEDC_DUTY_MAX (max forward) */
    if (left_motors_pwm > LEDC_DUTY_MAX) left_motors_pwm = LEDC_DUTY_MAX;
    if (left_motors_pwm < -LEDC_DUTY_MAX) left_motors_pwm = -LEDC_DUTY_MAX;
    if (right_motors_pwm > LEDC_DUTY_MAX) right_motors_pwm = LEDC_DUTY_MAX;
    if (right_motors_pwm < -LEDC_DUTY_MAX) right_motors_pwm = -LEDC_DUTY_MAX;

    /* Set the speed and direction for both motors according to the PWM values */
    esp_err_t err_a = left_motors_control(left_motors_pwm);
    esp_err_t err_b = right_motors_control(right_motors_pwm);

    if (err_a != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed to set left motor speed: %s", esp_err_to_name(err_a));
        return err_a;
    }
    if (err_b != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed to set right motor speed: %s", esp_err_to_name(err_b));
        return err_b;
    }

    return ESP_OK;
}

static esp_err_t left_motors_control(int pwm_val) 
{
    /* Set the direction pins based on the sign of pwm_val */
    esp_err_t gpio_err;
    if (pwm_val < 0) 
    {
        gpio_err = left_motors_set_reverse_dir();
    } 
    else 
    {
        gpio_err = left_motors_set_forward_dir();
    }
    if (gpio_err != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed to set left motor direction: %s", esp_err_to_name(gpio_err));
        return gpio_err;
    }

    /* Negative PWM values are used in the motor_set_speed as an abstraction to indicate reverse direction.
     * The actual valid PWM duty cycle values are positive integers from 0 to LEDC_DUTY_MAX. */
    uint32_t duty = abs(pwm_val);
    if (duty > LEDC_DUTY_MAX) 
    {
        duty = LEDC_DUTY_MAX;
    }

    /* Set the duty cycle for the left motors */
    esp_err_t duty_err_set = ledc_set_duty(LEDC_MODE, LEFT_MOTOR_LEDC_CHANNEL_A, duty);
    if (duty_err_set != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed to set left motor duty: %s", esp_err_to_name(duty_err_set));
        return duty_err_set;
    }

    /* Update the duty cycle to apply the change */
    esp_err_t duty_err_update = ledc_update_duty(LEDC_MODE, LEFT_MOTOR_LEDC_CHANNEL_A);
    if (duty_err_update != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed to update left motor duty: %s", esp_err_to_name(duty_err_update));
        return duty_err_update;
    }

    return ESP_OK;
}

static esp_err_t right_motors_control(int pwm_val)
{
    /* Set the direction pins based on the sign of pwm_val */
    esp_err_t gpio_err;
    if (pwm_val < 0)
    {
        gpio_err = right_motors_set_reverse_dir();
    }
    else
    {
        gpio_err = right_motors_set_forward_dir();
    }
    if (gpio_err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set right motor direction: %s", esp_err_to_name(gpio_err));
        return gpio_err;
    }

    /* Negative PWM values are used in the motor_set_speed as an abstraction to indicate reverse direction.
     * The actual valid PWM duty cycle values are positive integers from 0 to LEDC_DUTY_MAX. */
    uint32_t duty = abs(pwm_val);
    if (duty > LEDC_DUTY_MAX)
    {
        duty = LEDC_DUTY_MAX;
    }

    /* Set the duty cycle for the right motors */
    esp_err_t duty_err_set = ledc_set_duty(LEDC_MODE, RIGHT_MOTOR_LEDC_CHANNEL_B, duty);
    if (duty_err_set != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set right motor duty: %s", esp_err_to_name(duty_err_set));
        return duty_err_set;
    }

    /* Update the duty cycle to apply the change */
    esp_err_t duty_err_update = ledc_update_duty(LEDC_MODE, RIGHT_MOTOR_LEDC_CHANNEL_B);
    if (duty_err_update != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to update right motor duty: %s", esp_err_to_name(duty_err_update));
        return duty_err_update;
    }

    return ESP_OK;
}

static esp_err_t left_motors_set_reverse_dir(void)
{
    esp_err_t err;

    err = gpio_set_level(LEFT_MOTOR_A_AIN1_PIN, 0);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set AIN1 low: %s", esp_err_to_name(err));
        return err;
    }
    err = gpio_set_level(LEFT_MOTOR_A_AIN2_PIN, 1);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set AIN2 high: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

static esp_err_t left_motors_set_forward_dir(void)
{
    esp_err_t err;

    err = gpio_set_level(LEFT_MOTOR_A_AIN1_PIN, 1);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set AIN1 high: %s", esp_err_to_name(err));
        return err;
    }
    err = gpio_set_level(LEFT_MOTOR_A_AIN2_PIN, 0);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set AIN2 low: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

static esp_err_t right_motors_set_reverse_dir(void)
{
    esp_err_t err;

    err = gpio_set_level(RIGHT_MOTOR_B_BIN1_PIN, 0);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set BIN1 low: %s", esp_err_to_name(err));
        return err;
    }
    err = gpio_set_level(RIGHT_MOTOR_B_BIN2_PIN, 1);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set BIN2 high: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

static esp_err_t right_motors_set_forward_dir(void)
{
    esp_err_t err;

    err = gpio_set_level(RIGHT_MOTOR_B_BIN1_PIN, 1);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set BIN1 high: %s", esp_err_to_name(err));
        return err;
    }
    err = gpio_set_level(RIGHT_MOTOR_B_BIN2_PIN, 0);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set BIN2 low: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

static esp_err_t all_motors_set_coast(void)
{
    esp_err_t err;

    err = gpio_set_level(LEFT_MOTOR_A_AIN1_PIN, 0);
    if (err != ESP_OK) { ESP_LOGE(TAG, "Coast: Failed AIN1 low: %s", esp_err_to_name(err)); return err; }
    err = gpio_set_level(LEFT_MOTOR_A_AIN2_PIN, 0);
    if (err != ESP_OK) { ESP_LOGE(TAG, "Coast: Failed AIN2 low: %s", esp_err_to_name(err)); return err; }
    err = gpio_set_level(RIGHT_MOTOR_B_BIN1_PIN, 0);
    if (err != ESP_OK) { ESP_LOGE(TAG, "Coast: Failed BIN1 low: %s", esp_err_to_name(err)); return err; }
    err = gpio_set_level(RIGHT_MOTOR_B_BIN2_PIN, 0);
    if (err != ESP_OK) { ESP_LOGE(TAG, "Coast: Failed BIN2 low: %s", esp_err_to_name(err)); return err; }

    return ESP_OK;
}

static esp_err_t all_motors_set_brake(void)
{
    esp_err_t err;

    err = gpio_set_level(LEFT_MOTOR_A_AIN1_PIN, 1);
    if (err != ESP_OK) { ESP_LOGE(TAG, "Brake: Failed AIN1 high: %s", esp_err_to_name(err)); return err; }
    err = gpio_set_level(LEFT_MOTOR_A_AIN2_PIN, 1);
    if (err != ESP_OK) { ESP_LOGE(TAG, "Brake: Failed AIN2 high: %s", esp_err_to_name(err)); return err; }
    err = gpio_set_level(RIGHT_MOTOR_B_BIN1_PIN, 1);
    if (err != ESP_OK) { ESP_LOGE(TAG, "Brake: Failed BIN1 high: %s", esp_err_to_name(err)); return err; }
    err = gpio_set_level(RIGHT_MOTOR_B_BIN2_PIN, 1);
    if (err != ESP_OK) { ESP_LOGE(TAG, "Brake: Failed BIN2 high: %s", esp_err_to_name(err)); return err; }

    return ESP_OK;
}