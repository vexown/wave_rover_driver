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
#include <string.h>

/* ESP-IDF Includes */
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

/* Project Includes */
#include "motor_control.h"
#include "roarm_m3_motor_control.h"
#include "web_server.h"
#include "Common.h"
#include "comms_uart.h"
#include "esp_now_comm.h"

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

/* FreeRTOS Task defines */
#define MOTOR_TASK_STACK_SIZE       (4096)
#define MOTOR_TASK_PRIORITY         (tskIDLE_PRIORITY + 5)

/* Xbox Controller Configuration */
#define XBOX_DEADZONE               (5000)      // Deadzone for joystick (out of ±32767)
#define XBOX_MAX_AXIS_VALUE         (32767)     // Maximum axis value from Xbox controller
#define XBOX_TRIGGER_MAX_VALUE      (255)       // Maximum trigger value from Xbox controller (0-255)
#define UART_RX_TIMEOUT_MS          (100)       // Timeout for UART receive

/*******************************************************************************/
/*                                DATA TYPES                                   */
/*******************************************************************************/

/* Xbox controller state structure */
typedef struct {
    int lx, ly;     // Left stick
    int rx, ry;     // Right stick
    int lt, rt;     // Triggers
    int dpx, dpy;   // D-pad
    int a, b, x, y; // Face buttons
    int lb, rb;     // Bumpers
    int back, start, guide; // Other buttons
} xbox_controller_t;

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

/* RoArm watchdog timer variables */
static uint64_t last_roarm_command_time = 0;
static const uint32_t ROARM_COMMAND_TIMEOUT_MS = 500;
static bool roarm_control_active = false;

/*******************************************************************************/
/*                     STATIC FUNCTION DECLARATIONS                            */
/*******************************************************************************/

/**
 * @brief Initialize ESP-NOW for motor controller communication.
 *
 * This function sets up the ESP-NOW protocol to enable wireless communication
 * with the motor controller.
 *
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
static esp_err_t esp_now_motor_controller_init(void);

/**
 * @brief Initialize direct motor control.
 *
 * This function sets up the necessary components for direct motor control,
 * such as PWM timers and GPIO pins.
 *
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
static esp_err_t direct_motor_control_init(void);

/**
 * @brief Task to handle motor control operations.
 *
 * This task is used for handling motor control operations from
 * a Xbox 360 controller (based on UART messages coming from the
 * Raspberry Pi).
 *
 * @param pvParameters Pointer to task parameters (not used).
 */
static void motor_task(void *pvParameters);

/**
 * @brief Map Xbox axis value to PWM duty cycle with deadzone.
 *
 * @param axis_value Raw axis value from Xbox controller (-32767 to 32767).
 * @return int PWM duty cycle (-255 to 255).
 */
static int map_axis_to_pwm(int axis_value);

/**
 * @brief Map Xbox axis value to angle in radians with deadzone.
 *
 * @param axis_value Raw axis value from Xbox controller (-32767 to 32767).
 * @return float Angle in radians (-1.57 to 1.57).
 */
static float map_axis_to_angle(int axis_value);

/**
 * @brief Parse Xbox controller data from UART message.
 *
 * Expected format: S|LX:value|LY:value|RX:value|RY:value|LT:value|RT:value|DPX:value|DPY:value|A:value|B:value|X:value|Y:value|LB:value|RB:value|BACK:value|START:value|GUIDE:value|E\n
 *
 * @param data The received UART data string.
 * @param controller Pointer to xbox_controller_t struct to fill.
 * @return esp_err_t ESP_OK on success, ESP_FAIL on parse error.
 */
static esp_err_t parse_xbox_data(const char* data, xbox_controller_t* controller);

/**
 * @brief Process Xbox controller input and control motors.
 *
 * @param rx_buffer The received UART data string.
 * @param mode The motor control mode.
 */
static void process_xbox_input(const char* rx_buffer, motor_control_mode_t mode);

/**
 * @brief Send constant control command to RoArm for a single axis.
 *
 * @param axis Joint ID (1=base, 2=shoulder, 3=elbow, 4=wrist, 5=roll, 6=eoat)
 * @param cmd Command (0=stop, 1=decrease, 2=increase)
 * @param speed Speed value (0-100)
 */
static void roarm_send_constant_ctrl(uint8_t axis, uint8_t cmd, uint8_t speed);

/**
 * @brief Check RoArm command timeout and send stop if needed.
 */
static void roarm_check_timeout(void);

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

/* Motor controller ESP32 MAC address for testing communications */
static uint8_t motor_controller_mac[6] = {0xB4, 0x3A, 0x45, 0x89, 0x61, 0x54};

/*******************************************************************************/
/*                     GLOBAL FUNCTION DEFINITIONS                             */
/*******************************************************************************/
esp_err_t motor_init(const motor_control_mode_t mode) 
{
    esp_err_t err;
    ESP_LOGI(TAG, "Initializing motor control");

    motor_mutex = xSemaphoreCreateMutex();
    if (motor_mutex == NULL) 
    {
        ESP_LOGE(TAG, "Failed to create motor mutex");
        return ESP_FAIL;
    }

    if(mode == DIRECT_MOTOR_CONTROL)
    {
        web_server_print("Motor Control Mode: Direct Motor Control");
        err = direct_motor_control_init();
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Direct motor control initialization failed: %s", esp_err_to_name(err));
            return err;
        }
    }
    else if(mode == ESP_NOW_MOTOR_CONTROLLER)
    {
        web_server_print("Motor Control Mode: ESP-NOW Motor Controller");
        err = esp_now_motor_controller_init();
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "ESP-NOW motor controller initialization failed: %s", esp_err_to_name(err));
            return err;
        }
    }
    else
    {
        web_server_print("Motor Control Mode: Unknown");
        return ESP_FAIL;
    }

    /* Create the motor control task */
    BaseType_t task_created = xTaskCreate(motor_task, "motor_task", MOTOR_TASK_STACK_SIZE, (void*)mode, MOTOR_TASK_PRIORITY, NULL);
    if (task_created != pdPASS) 
    {
        ESP_LOGE(TAG, "Failed to create motor control task");
        return ESP_FAIL;
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

static esp_err_t direct_motor_control_init(void)
{
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

    return ESP_OK;
}

static esp_err_t esp_now_motor_controller_init(void)
{
    /* Add the motor controller ESP32 as an ESP-NOW peer to be able to send commands to it */
    esp_err_t err = esp_now_comm_add_peer(motor_controller_mac);
    /* Add the RoArm-M3's ESP32 as an ESP-NOW peer too */
    err = esp_now_comm_add_peer(roarm_mac);
    if (err != ESP_OK)
    {
        char peer_error_buffer[64];
        snprintf(peer_error_buffer, sizeof(peer_error_buffer), "Failed to add motor controller peer: %s", esp_err_to_name(err));
        web_server_print(peer_error_buffer);
        return err;
    }
    else
    {
        web_server_print("Motor controller peer added successfully");
    }

    return ESP_OK;
}

static void motor_task(void *pvParameters) 
{
    char rx_buffer[512];  // Buffer for UART data
    uint32_t no_data_counter = 0;  // Counter for UART timeout detection
 
    motor_control_mode_t mode = (motor_control_mode_t)pvParameters;  // Cast back to the enum type

    ESP_LOGI(TAG, "Motor control task started - waiting for Xbox controller input");
    
    while (1)
    {
        /* Read Xbox controller data from RPi via UART */
        int rx_len = comms_uart_receive((uint8_t*)rx_buffer, sizeof(rx_buffer) - 1, UART_RX_TIMEOUT_MS);
        
        if (rx_len > 0)
        {
            /* Reset timeout counter when data is received */
            no_data_counter = 0;
            
            /* Null-terminate the received data */
            rx_buffer[rx_len] = '\0';
            
            process_xbox_input(rx_buffer, mode);
        }
        else
        {
            /* No data received - increment timeout counter */
            no_data_counter++;
            
            /* Safety: Stop motors if no UART data for 500ms (50 * 10ms) */
            if (no_data_counter > 50)
            {
                motor_set_speed(0, 0);  // Stop motors
                
                /* Cap the counter to prevent overflow */
                if (no_data_counter > 1000)
                {
                    no_data_counter = 51;  // Keep it above threshold
                }
            }
        }
        
        /* Check RoArm command timeout */
        roarm_check_timeout();
        
        /* Small delay to prevent task hogging CPU */
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void process_xbox_input(const char* rx_buffer, motor_control_mode_t mode)
{
    xbox_controller_t controller;
    
    /* Parse the Xbox controller data */
    esp_err_t parsing_status = parse_xbox_data(rx_buffer, &controller);

    if(parsing_status == ESP_OK)
    {
        /* === ROVER MOTOR CONTROL (Left Stick) === */
        /* LY controls forward/backward (inverted: negative = forward)
         * LX controls left/right turning (inverted: negative = right) */
        
        /* Map joystick values to PWM */
        int forward_pwm = map_axis_to_pwm(-controller.ly);  // Invert Y axis
        int turn_pwm = map_axis_to_pwm(-controller.lx);     // Invert X axis
        
        /* Check if joystick is centered (both axes in deadzone) */
        if (forward_pwm == 0 && turn_pwm == 0)
        {
            /* Stop motors immediately when joystick is released */
            motor_set_speed(0, 0);
        }
        else
        {
            /* Calculate differential drive (tank drive mixing)
             * Left motor = forward - turn
             * Right motor = forward + turn */
            int left_motor_pwm = forward_pwm - turn_pwm;
            int right_motor_pwm = forward_pwm + turn_pwm;
            
            /* Clamp final values */
            if (left_motor_pwm > LEDC_DUTY_MAX) left_motor_pwm = LEDC_DUTY_MAX;
            if (left_motor_pwm < -LEDC_DUTY_MAX) left_motor_pwm = -LEDC_DUTY_MAX;
            if (right_motor_pwm > LEDC_DUTY_MAX) right_motor_pwm = LEDC_DUTY_MAX;
            if (right_motor_pwm < -LEDC_DUTY_MAX) right_motor_pwm = -LEDC_DUTY_MAX;
            

            if(mode == DIRECT_MOTOR_CONTROL)
            {
                /* Apply motor speeds directly */
                motor_set_speed(left_motor_pwm, right_motor_pwm);
            }
            else if(mode == ESP_NOW_MOTOR_CONTROLLER)
            {
                /* Send motor speeds via ESP-NOW to motor controller */
                char motor_cmd[32];
                snprintf(motor_cmd, sizeof(motor_cmd), "L:%d|R:%d", left_motor_pwm, right_motor_pwm);
                
                esp_err_t send_err = esp_now_comm_send(motor_controller_mac, (uint8_t*)motor_cmd, strlen(motor_cmd));
                if (send_err != ESP_OK)
                {
                    char error_buffer[64];
                    snprintf(error_buffer, sizeof(error_buffer), "Failed to send motor command: %s", esp_err_to_name(send_err));
                    web_server_print(error_buffer);
                }
            }
        }

        /* === ROARM-M3 CONTROL === */
        
        /* Button Controls - Edge detection */
        static uint8_t last_a_button = 0;
        static uint8_t last_b_button = 0;
        
        /* A Button: Move to init position */
        if (controller.a == 1 && last_a_button == 0)
        {
            ESP_LOGI(TAG, "A button: Moving RoArm to init position");
            roarm_move_init();
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
        last_a_button = controller.a;
        
        /* B Button: Emergency stop all movement */
        if (controller.b == 1 && last_b_button == 0)
        {
            ESP_LOGI(TAG, "B button: Emergency stop - stopping all RoArm axes");
            for (uint8_t axis = 1; axis <= 6; axis++)
            {
                roarm_send_constant_ctrl(axis, 0, 0);
                vTaskDelay(pdMS_TO_TICKS(5));
            }
        }
        last_b_button = controller.b;
        
        /* Track previous states for stop commands */
        static int16_t prev_rx = 0;
        static int16_t prev_ry = 0;
        static uint8_t prev_lb = 0;
        static uint8_t prev_rb = 0;
        static uint8_t prev_dpad_up = 0;
        static uint8_t prev_dpad_down = 0;
        static uint8_t prev_dpad_left = 0;
        static uint8_t prev_dpad_right = 0;
        static uint8_t prev_lt_active = 0;
        static uint8_t prev_rt_active = 0;
        
        /* === Joint 1: BASE - Right Stick X === */
        if (abs(controller.rx) > XBOX_DEADZONE)
        {
            uint8_t cmd = (controller.rx > 0) ? 2 : 1;  // 2=increase (right), 1=decrease (left)
            uint8_t speed = (uint8_t)((float)abs(controller.rx) / XBOX_MAX_AXIS_VALUE * 15);
            roarm_send_constant_ctrl(1, cmd, speed);  // BASE_JOINT
            prev_rx = controller.rx;
        }
        else if (abs(prev_rx) > XBOX_DEADZONE)
        {
            roarm_send_constant_ctrl(1, 0, 0);  // Stop base
            prev_rx = 0;
        }
        
        /* === Joint 2: SHOULDER - Right Stick Y === */
        if (abs(controller.ry) > XBOX_DEADZONE)
        {
            uint8_t cmd = (controller.ry > 0) ? 2 : 1;  // 2=increase (forward), 1=decrease (back)
            uint8_t speed = (uint8_t)((float)abs(controller.ry) / XBOX_MAX_AXIS_VALUE * 15);
            roarm_send_constant_ctrl(2, cmd, speed);  // SHOULDER_JOINT
            prev_ry = controller.ry;
        }
        else if (abs(prev_ry) > XBOX_DEADZONE)
        {
            roarm_send_constant_ctrl(2, 0, 0);  // Stop shoulder
            prev_ry = 0;
        }
        
        /* === Joint 3: ELBOW - D-Pad Up/Down === */
        if (controller.dpy > 0)  // D-Pad Up
        {
            roarm_send_constant_ctrl(3, 1, 10);  // ELBOW_JOINT decrease (up)
            prev_dpad_up = 1;
        }
        else if (prev_dpad_up == 1)
        {
            roarm_send_constant_ctrl(3, 0, 0);  // Stop elbow
            prev_dpad_up = 0;
        }
        
        if (controller.dpy < 0)  // D-Pad Down
        {
            roarm_send_constant_ctrl(3, 2, 10);  // ELBOW_JOINT increase (down)
            prev_dpad_down = 1;
        }
        else if (prev_dpad_down == 1)
        {
            roarm_send_constant_ctrl(3, 0, 0);  // Stop elbow
            prev_dpad_down = 0;
        }
        
        /* === Joint 4: WRIST (Pitch) - D-Pad Left/Right === */
        if (controller.dpx < 0)  // D-Pad Left
        {
            roarm_send_constant_ctrl(4, 1, 10);  // WRIST_JOINT decrease (up)
            prev_dpad_left = 1;
        }
        else if (prev_dpad_left == 1)
        {
            roarm_send_constant_ctrl(4, 0, 0);  // Stop wrist
            prev_dpad_left = 0;
        }
        
        if (controller.dpx > 0)  // D-Pad Right
        {
            roarm_send_constant_ctrl(4, 2, 10);  // WRIST_JOINT increase (down)
            prev_dpad_right = 1;
        }
        else if (prev_dpad_right == 1)
        {
            roarm_send_constant_ctrl(4, 0, 0);  // Stop wrist
            prev_dpad_right = 0;
        }
        
        /* === Joint 5: ROLL (Wrist rotation) - LB/RB === */
        if (controller.lb == 1)
        {
            roarm_send_constant_ctrl(5, 1, 10);  // ROLL_JOINT decrease (counter-clockwise)
            prev_lb = 1;
        }
        else if (prev_lb == 1)
        {
            roarm_send_constant_ctrl(5, 0, 0);  // Stop roll
            prev_lb = 0;
        }
        
        if (controller.rb == 1)
        {
            roarm_send_constant_ctrl(5, 2, 10);  // ROLL_JOINT increase (clockwise)
            prev_rb = 1;
        }
        else if (prev_rb == 1)
        {
            roarm_send_constant_ctrl(5, 0, 0);  // Stop roll
            prev_rb = 0;
        }
        
        /* === Joint 6: GRIPPER - LT/RT === */
        float lt_normalized = (float)controller.lt / XBOX_TRIGGER_MAX_VALUE;
        float rt_normalized = (float)controller.rt / XBOX_TRIGGER_MAX_VALUE;
        const float TRIGGER_THRESHOLD = 0.1f;
        
        /* LT: Close gripper (increase angle) */
        if (lt_normalized > TRIGGER_THRESHOLD)
        {
            uint8_t speed = (uint8_t)(lt_normalized * 5);  // Reduced speed for smoother control
            roarm_send_constant_ctrl(6, 2, speed);  // EOAT_JOINT increase (close)
            prev_lt_active = 1;
        }
        else if (prev_lt_active == 1)
        {
            roarm_send_constant_ctrl(6, 0, 0);  // Stop gripper
            prev_lt_active = 0;
        }
        
        /* RT: Open gripper (decrease angle) */
        if (rt_normalized > TRIGGER_THRESHOLD)
        {
            uint8_t speed = (uint8_t)(rt_normalized * 5);  // Reduced speed for smoother control
            roarm_send_constant_ctrl(6, 1, speed);  // EOAT_JOINT decrease (open)
            prev_rt_active = 1;
        }
        else if (prev_rt_active == 1)
        {
            roarm_send_constant_ctrl(6, 0, 0);  // Stop gripper
            prev_rt_active = 0;
        }


    }
    else
    {
        /* Only log parse failures occasionally to avoid spam */
        static uint32_t parse_fail_counter = 0;
        if (++parse_fail_counter % 100 == 0)
        {
            char debug_buffer[64];
            snprintf(debug_buffer, sizeof(debug_buffer), 
                     "Failed to parse Xbox data (100 failures)");
            web_server_print(debug_buffer);
        }
    }
}

static void roarm_send_constant_ctrl(uint8_t axis, uint8_t cmd, uint8_t speed)
{
    /* Build JSON command for constant control (T:123) */
    /* Format: {"T":123,"m":0,"axis":0,"cmd":0,"spd":3} */
    char json[128];
    snprintf(json, sizeof(json),
             "{\"T\":123,\"m\":0,\"axis\":%d,\"cmd\":%d,\"spd\":%d}",
             axis, cmd, speed);
    
    roarm_send_json_cmd(roarm_mac, json);
}

static void roarm_check_timeout(void)
{
    if (roarm_control_active)
    {
        uint64_t current_time = esp_timer_get_time() / 1000;  // Convert to milliseconds
        
        if ((current_time - last_roarm_command_time) > ROARM_COMMAND_TIMEOUT_MS)
        {
            /* Timeout exceeded - send emergency stop to all axes */
            ESP_LOGW(TAG, "RoArm command timeout - sending stop commands");
            for (uint8_t axis = 1; axis <= 6; axis++)
            {
                roarm_send_constant_ctrl(axis, 0, 0);
                vTaskDelay(pdMS_TO_TICKS(5));  // Small delay between commands
            }
            roarm_control_active = false;
        }
    }
}

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

static float map_axis_to_angle(int axis_value)
{
    /* Apply deadzone */
    if (abs(axis_value) < XBOX_DEADZONE)
    {
        return 0.0f;
    }
    
    /* Map from ±32767 to ±1.57 radians (±90 degrees) */
    float normalized = (float)axis_value / (float)XBOX_MAX_AXIS_VALUE;
    float angle = normalized * 1.57f;
    
    /* Clamp to valid range */
    if (angle > 1.57f) angle = 1.57f;
    if (angle < -1.57f) angle = -1.57f;
    
    return angle;
}

static int map_axis_to_pwm(int axis_value)
{
    /* Apply deadzone */
    if (abs(axis_value) < XBOX_DEADZONE)
    {
        return 0;
    }
    
    /* Map from ±32767 to ±LEDC_DUTY_MAX (±255)
     * Note: We use float for precision during calculation */
    float normalized = (float)axis_value / (float)XBOX_MAX_AXIS_VALUE;
    int pwm = (int)(normalized * LEDC_DUTY_MAX);
    
    /* Clamp to valid range */
    if (pwm > LEDC_DUTY_MAX) pwm = LEDC_DUTY_MAX;
    if (pwm < -LEDC_DUTY_MAX) pwm = -LEDC_DUTY_MAX;
    
    return pwm;
}

static esp_err_t parse_xbox_data(const char* data, xbox_controller_t* controller)
{
    /*
     * ============================================================================
     * Xbox 360 Controller Mapping (Experimentally Determined)
     * ============================================================================
     * This mapping was determined using the test_controller_mapping.py diagnostic tool
     * and is specific to the Xbox 360 controller with Microsoft Wireless Receiver.
     *
     * AXES (Analog inputs, range: -32767 to 32767):
     * -----------------------------------------------
     * Index 0: Left Joystick X-axis (LX)
     * Index 1: Left Joystick Y-axis (LY)
     * Index 2: Right Joystick X-axis (RX)
     * Index 3: Right Joystick Y-axis (RY)
     * Index 4: Right Trigger (RT) - 32767 fully pressed, -32767 fully released
     * Index 5: Left Trigger (LT) - 32767 fully pressed, -32767 fully released
     * Index 6: D-Pad X-axis (DPX) - -32767 left, 32767 right, 0 centered
     * Index 7: D-Pad Y-axis (DPY) - -32767 up, 32767 down, 0 centered
     *
     * BUTTONS (Digital inputs, 0=released, 1=pressed):
     * ------------------------------------------------
     * Index 0: A button
     * Index 1: B button
     * Index 2: X button
     * Index 3: Y button
     * Index 4: Left Bumper (LB)
     * Index 5: Right Bumper (RB)
     * Index 6: Back button
     * Index 7: Start button
     * Index 8: Guide button (Xbox logo) - NOTE: Not included in controller_state
     * Index 9: Left Joystick Press (L3) - NOTE: Not included in controller_state
     * Index 10: Right Joystick Press (R3) - NOTE: Not included in controller_state
     * ============================================================================
     *
     * Expected format of the incoming UART data coming from Raspberry Pi:
     * S|LX:value|LY:value|RX:value|RY:value|LT:value|RT:value|DPX:value|DPY:value|A:value|B:value|X:value|Y:value|LB:value|RB:value|BACK:value|START:value|GUIDE:value|E\n 
     */
    
    /* Find the start marker */
    const char* start = strstr(data, "S|");
    if (!start)
    {
        ESP_LOGW(TAG, "No start marker found in Xbox data");
        return ESP_FAIL;
    }
    
    /* Initialize all to 0 */
    memset(controller, 0, sizeof(xbox_controller_t));
    
    /* Parse each field */
    const char* fields[] = {"LX:", "LY:", "RX:", "RY:", "LT:", "RT:", "DPX:", "DPY:", "A:", "B:", "X:", "Y:", "LB:", "RB:", "BACK:", "START:", "GUIDE:"};
    int* values[] = {&controller->lx, &controller->ly, &controller->rx, &controller->ry, &controller->lt, &controller->rt, &controller->dpx, &controller->dpy, &controller->a, &controller->b, &controller->x, &controller->y, &controller->lb, &controller->rb, &controller->back, &controller->start, &controller->guide};
    
    /* Print the whole received data [uncomment for debug] */
    // web_server_print(data);

    for (int i = 0; i < sizeof(fields)/sizeof(fields[0]); i++)
    {
        const char* pos = strstr(start, fields[i]);
        if (pos)
        {
            *values[i] = atoi(pos + strlen(fields[i]));
        }
        else
        {
            ESP_LOGW(TAG, "%s not found in Xbox data", fields[i]);
            // Don't fail, just set to 0
        }
    }
    
    return ESP_OK;
}