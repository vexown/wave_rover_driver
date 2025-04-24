#include "motor_control.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <stdlib.h> // For abs()
#include <math.h>   // For round()

// From ugv_config.h
/* Motor PWM Pins */
#define MOTOR_A_PWMA_PIN    GPIO_NUM_25
#define MOTOR_A_AIN1_PIN    GPIO_NUM_21
#define MOTOR_A_AIN2_PIN    GPIO_NUM_17
#define MOTOR_B_PWMB_PIN    GPIO_NUM_26
#define MOTOR_B_BIN1_PIN    GPIO_NUM_22
#define MOTOR_B_BIN2_PIN    GPIO_NUM_23

/* LEDC Configuration. LEDC is this poorly named module that provides PWM functionality on ESP32. 
 * It is not strictly used for LEDs, but rather for any GPIO pin that needs PWM control. */
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE // Use low speed mode if frequency is low
#define LEDC_CHANNEL_A          LEDC_CHANNEL_5 // Corresponds to channel_A = 5 in Arduino
#define LEDC_CHANNEL_B          LEDC_CHANNEL_6 // Corresponds to channel_B = 6 in Arduino
#define LEDC_DUTY_RES           LEDC_TIMER_8_BIT // Corresponds to ANALOG_WRITE_BITS = 8
#define LEDC_DUTY_MAX           (255) // (1 << LEDC_DUTY_RES) - 1
#define LEDC_FREQUENCY          (100000) // freq = 100000 in Arduino

// Note: SET_MOTOR_DIR is false in the provided ugv_config.h
// If true, the logic in motor_set_speed needs to be inverted.
#define SET_MOTOR_DIR           false

static const char *TAG = "MOTOR_CONTROL"; // Log tag for this module

esp_err_t motor_init(void) 
{
    ESP_LOGI(TAG, "Initializing motor control");

    // --- Configure GPIOs for Direction Control ---
    gpio_config_t io_conf = 
    {
        .pin_bit_mask = (1ULL << MOTOR_A_AIN1_PIN) | (1ULL << MOTOR_A_AIN2_PIN) |
                        (1ULL << MOTOR_B_BIN1_PIN) | (1ULL << MOTOR_B_BIN2_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG, "GPIO config failed: %s", esp_err_to_name(err));
        return err;
    }

    // Set initial direction states to LOW (brake/coast depending on driver)
    gpio_set_level(MOTOR_A_AIN1_PIN, 0);
    gpio_set_level(MOTOR_A_AIN2_PIN, 0);
    gpio_set_level(MOTOR_B_BIN1_PIN, 0);
    gpio_set_level(MOTOR_B_BIN2_PIN, 0);

    // --- Configure LEDC Timer ---
    ledc_timer_config_t ledc_timer = 
    {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    err = ledc_timer_config(&ledc_timer);
    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG, "LEDC timer config failed: %s", esp_err_to_name(err));
        return err;
    }

    // --- Configure LEDC Channel A (Left Motor) ---
    ledc_channel_config_t ledc_channel_a = 
    {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_A,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = MOTOR_A_PWMA_PIN,
        .duty           = 0, // Initial duty 0
        .hpoint         = 0
    };
    err = ledc_channel_config(&ledc_channel_a);
    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG, "LEDC channel A config failed: %s", esp_err_to_name(err));
        return err;
    }

    // --- Configure LEDC Channel B (Right Motor) ---
    ledc_channel_config_t ledc_channel_b = 
    {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_B,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = MOTOR_B_PWMB_PIN,
        .duty           = 0, // Initial duty 0
        .hpoint         = 0
    };
    err = ledc_channel_config(&ledc_channel_b);
    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG, "LEDC channel B config failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Motor control initialized successfully");
    return ESP_OK;
}

// Internal function similar to Arduino leftCtrl
static esp_err_t motor_left_ctrl(int pwm_val) 
{
    uint32_t duty = abs(pwm_val);
    if (duty > LEDC_DUTY_MAX) 
    {
        duty = LEDC_DUTY_MAX;
    }

    if (SET_MOTOR_DIR) 
    { 
        if (pwm_val < 0) 
        {
            gpio_set_level(MOTOR_A_AIN1_PIN, 1);
            gpio_set_level(MOTOR_A_AIN2_PIN, 0);
        } 
        else 
        {
            gpio_set_level(MOTOR_A_AIN1_PIN, 0);
            gpio_set_level(MOTOR_A_AIN2_PIN, 1);
        }
    } 
    else 
    {
        if (pwm_val < 0) 
        {
            gpio_set_level(MOTOR_A_AIN1_PIN, 0);
            gpio_set_level(MOTOR_A_AIN2_PIN, 1);
        } 
        else 
        {
            gpio_set_level(MOTOR_A_AIN1_PIN, 1);
            gpio_set_level(MOTOR_A_AIN2_PIN, 0);
        }
    }

    esp_err_t err = ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_A, duty);
    if (err != ESP_OK) return err;

    err = ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_A);
    return err;
}

// Internal function similar to Arduino rightCtrl
static esp_err_t motor_right_ctrl(int pwm_val) 
{
    uint32_t duty = abs(pwm_val);
    if (duty > LEDC_DUTY_MAX) 
    {
        duty = LEDC_DUTY_MAX;
    }

    if (SET_MOTOR_DIR) //TODO - check if SET_MOTOR_DIR is needed or can be removed
    { 
        if (pwm_val < 0) 
        {
            gpio_set_level(MOTOR_B_BIN1_PIN, 1);
            gpio_set_level(MOTOR_B_BIN2_PIN, 0);
        } 
        else 
        {
            gpio_set_level(MOTOR_B_BIN1_PIN, 0);
            gpio_set_level(MOTOR_B_BIN2_PIN, 1);
        }
    } 
    else 
    { 
        if (pwm_val < 0) 
        {
            gpio_set_level(MOTOR_B_BIN1_PIN, 0);
            gpio_set_level(MOTOR_B_BIN2_PIN, 1);
        } 
        else 
        {
            gpio_set_level(MOTOR_B_BIN1_PIN, 1);
            gpio_set_level(MOTOR_B_BIN2_PIN, 0);
        }
    }

    esp_err_t err = ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_B, duty);
    if (err != ESP_OK) return err;

    err = ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_B);
    return err;
}


esp_err_t motor_set_speed(int left_pwm, int right_pwm) 
{
    // Clamp input PWM values to the expected range (-255 to 255)
    if (left_pwm > LEDC_DUTY_MAX) left_pwm = LEDC_DUTY_MAX;
    if (left_pwm < -LEDC_DUTY_MAX) left_pwm = -LEDC_DUTY_MAX;
    if (right_pwm > LEDC_DUTY_MAX) right_pwm = LEDC_DUTY_MAX;
    if (right_pwm < -LEDC_DUTY_MAX) right_pwm = -LEDC_DUTY_MAX;

    // Note: The original Arduino code had spd_rate_A/B applied in switchPortCtrlA/B
    // and also in setGoalSpeed. Here, we assume the input pwm values are final
    // and don't apply the rate. If rate limiting is needed, it should be applied
    // before calling this function or added here explicitly.

    // Note: The original code checks mainType == 3 to decide whether to use PID
    // or direct control (scaled by 512). This implementation assumes direct PWM control.
    // The input range is assumed to be -255 to 255, matching the LEDC resolution.

    esp_err_t err_a = motor_left_ctrl(left_pwm);
    esp_err_t err_b = motor_right_ctrl(right_pwm);

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

esp_err_t motor_stop(void) 
{
    ESP_LOGI(TAG, "Stopping motors");
    // Set direction pins to LOW (brake/coast)
    gpio_set_level(MOTOR_A_AIN1_PIN, 0);
    gpio_set_level(MOTOR_A_AIN2_PIN, 0);
    gpio_set_level(MOTOR_B_BIN1_PIN, 0);
    gpio_set_level(MOTOR_B_BIN2_PIN, 0);

    // Set duty cycle to 0
    esp_err_t err_a = ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_A, 0);
    if (err_a == ESP_OK) err_a = ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_A);

    esp_err_t err_b = ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_B, 0);
    if (err_b == ESP_OK) err_b = ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_B);

    if (err_a != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed to stop left motor: %s", esp_err_to_name(err_a));
        return err_a; // Return first error encountered
    }
    if (err_b != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed to stop right motor: %s", esp_err_to_name(err_b));
        return err_b;
    }
    
    return ESP_OK;
}