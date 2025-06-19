/******************************************************************************
 * @file IMU.c
 * @brief IMU component for the Wave Rover Driver
 * 
 ******************************************************************************/

/*******************************************************************************/
/*                                 INCLUDES                                    */
/*******************************************************************************/
/*    Include headers required for the definitions/implementation in *this*    */
/* source file. This typically includes this module's own header("template.h") */
/*      and any headers needed for function bodies, static variables, etc.     */
/*******************************************************************************/
/* ESP-IDF Includes */
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

/* Project Includes */
#include "IMU.h"
#include "i2c_manager.h"
#include "web_server.h"

/*******************************************************************************/
/*                                  MACROS                                     */
/*******************************************************************************/
/** Tag for logging (used in ESP_LOGI, ESP_LOGE, etc.) Example usage: 
 *  ESP_LOGI(TAG, "Log message which will be appended to the tag"); */
#define TAG "IMU"

/* --- QMI8658C Register Definitions & Configuration --- */

/* The 7-bit I2C slave address for the QMI8658C. 0x6A if SA0 is high/unconnected, 0x6B if SA0 is low. */
#define QMI8658_I2C_ADDR_DEFAULT    0x6B // On our board, the SA0 pin is pulled low, so we use 0x6B

/* Register addresses */
#define QMI8658_WHO_AM_I_REG        0x00 // Device identifier register 
#define QMI8658_CTRL1_REG           0x02 // Serial interface and sensor enable settings 
#define QMI8658_CTRL2_REG           0x03 // Accelerometer settings 
#define QMI8658_CTRL3_REG           0x04 // Gyroscope settings 
#define QMI8658_CTRL7_REG           0x08 // Sensor enable register 
#define QMI8658_RESET_REG           0x60 // Soft reset register 

/* Register addresses for sensor data (accelerometer and gyroscope) */
#define QMI8658_REG_AX_L   0x35
#define QMI8658_REG_AX_H   0x36
#define QMI8658_REG_AY_L   0x37
#define QMI8658_REG_AY_H   0x38
#define QMI8658_REG_AZ_L   0x39
#define QMI8658_REG_AZ_H   0x3A
#define QMI8658_REG_GX_L   0x3B
#define QMI8658_REG_GX_H   0x3C
#define QMI8658_REG_GY_L   0x3D
#define QMI8658_REG_GY_H   0x3E
#define QMI8658_REG_GZ_L   0x3F
#define QMI8658_REG_GZ_H   0x40

/* Expected Device ID */
#define QMI8658_DEVICE_ID           0x05 // Value in WHO_AM_I register 

/* CTRL2: Accelerometer: Full-scale = ±8g, ODR = 940 Hz
 * aFS<2:0> = 010 for ±8g 
 * aODR<3:0> = 0011 for 940 Hz in 6DOF mode */
#define QMI8658_ACCEL_CONFIG        0b00100011

/* CTRL3: Gyroscope: Full-scale = ±2048 dps, ODR = 940 Hz
 * gFS<2:0> = 111 for ±2048 dps 
 * gODR<3:0> = 0011 for 940 Hz */ 
#define QMI8658_GYRO_CONFIG         0b01110011

/* CTRL1: Enable auto-increment for I2C address for burst reads
 * ADDR_AI = 1  */
#define QMI8658_CTRL1_CONFIG        0b01000000

/* CTRL7: Enable Accelerometer and Gyroscope
 * aEN = 1 (Enable Accelerometer) 
 * gEN = 1 (Enable Gyroscope) */
#define QMI8658_SENSOR_ENABLE       0b00000011

/* FreeRTOS task configuration */
#define TASK_READ_QMI8658_DATA_PRIORITY (tskIDLE_PRIORITY + 1)
#define TASK_READ_QMI8658_DATA_STACK_SIZE 2048 // Stack size in bytes
#define TASK_READ_QMI8658_DATA_PERIOD_TICKS pdMS_TO_TICKS(100) // Task period in ticks (100 ms)

/*******************************************************************************/
/*                                DATA TYPES                                   */
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
/*                     GLOBAL FUNCTION DECLARATIONS                            */
/*******************************************************************************/
/*  for function defined in some other .c files, to be used here. Use extern.  */
/*******************************************************************************/

/*******************************************************************************/
/*                             STATIC VARIABLES                                */
/*******************************************************************************/
/* I2C device handle for the sensor */
static i2c_master_dev_handle_t qmi8658_dev_handle = NULL;

/*******************************************************************************/
/*                     STATIC FUNCTION DECLARATIONS                            */
/*******************************************************************************/

/**
 * @brief Initialize the IMU component.
 * This function initializes the QMI8658C accelerometer and gyroscope, configures
 * the I2C bus, and sets up the necessary registers for operation.
 * 
 * @return ESP_OK on success, or an error code on failure.
 */
static esp_err_t qmi8658_init(void);

/**
 * @brief Initialize the I2C bus for the QMI8658C sensor.
 * This function acquires the I2C bus handle, configures the device address,
 * and sets the I2C speed. After that, it adds the QMI8658C device to the I2C bus.
 * 
 * @return ESP_OK on success, or an error code on failure.
 */
static esp_err_t qmi8658_i2c_init(void);

/**
 * @brief Reset the QMI8658C sensor.
 * This function performs a soft reset of the QMI8658C by writing a specific value
 * to the reset register. It also waits for the device to boot up.
 * 
 * @return ESP_OK on success, or an error code on failure.
 */
static esp_err_t qmi8658_reset(void);

/**
 * @brief Verify the device ID of the QMI8658C sensor.
 * This function reads the WHO_AM_I register to confirm that the device is a QMI8658C.
 * If the device ID does not match the expected value, it returns an error.
 * 
 * @return ESP_OK if the device ID matches, or an error code on failure.
 */
static esp_err_t qmi8658_verify_device_id(void);

/**
 * @brief Set the configuration for the QMI8658C in IMU mode.
 * This function configures the accelerometer and gyroscope settings, including
 * full-scale range and output data rate, as well as enabling auto-increment for
 * multi-byte reads.
 * 
 * @return ESP_OK on success, or an error code on failure.
 */
static esp_err_t qmi8658_set_config_for_IMU_mode(void);

/**
 * @brief Enable the accelerometer and gyroscope on the QMI8658C.
 * This function writes to the CTRL7 register to enable both sensors and
 * includes a short delay to allow the sensors to stabilize.
 * 
 * @return ESP_OK on success, or an error code on failure.
 */
static esp_err_t qmi8658_enable_accel_and_gyro(void);

/**
 * @brief Write a single byte to a specific register on the QMI8658C.
 *
 * @param reg_addr The register address to write to.
 * @param data The byte of data to write.
 * @return ESP_OK on success, ESP_FAIL on failure.
 */
static esp_err_t qmi8658_write_reg(uint8_t reg_addr, uint8_t data);

/**
 * @brief Read a single byte from a specific register on the QMI8658C.
 *
 * @param reg_addr The register address to read from.
 * @param data Pointer to store the read data.
 * @param data_size The size of the data to read (should be 1 for single byte).
 * @return ESP_OK on success, ESP_FAIL on failure.
 * 
 * @note For the burst reads to work correctly, in CTRL1 register, the ADDR_AI bit must be set to 1.
 *       This allows the device to auto-increment the register address for multi-byte reads.
 *       The data_size parameter should be set to the number of bytes you want to read.
 */
static esp_err_t qmi8658_read_reg(uint8_t reg_addr, uint8_t *data, size_t data_size);

/**
 * @brief Task to read data from the QMI8658C accelerometer and gyroscope.
 * This task will periodically read sensor data and process it.
 *
 * @param pvParameters Pointer to task parameters (not used).
 * @return void
 */
static void task_read_qmi8658_data(void* pvParameters);


/*******************************************************************************/
/*                     GLOBAL FUNCTION DEFINITIONS                             */
/*******************************************************************************/

esp_err_t imu_init(void)
{
    esp_err_t init_status = ESP_OK;

    init_status = qmi8658_init();
    if (init_status != ESP_OK)
    {
        return init_status;
    }

    /* Initialize AK09918 magnetometer */
    /* TODO */
    
    return ESP_OK;
}

/*******************************************************************************/
/*                     STATIC FUNCTION DEFINITIONS                             */
/*******************************************************************************/

static esp_err_t qmi8658_init(void) 
{
    esp_err_t ret = ESP_OK;
    char log_buffer[256]; // Buffer for formatted log messages
    
    /* Initialize I2C bus for QMI8658C */
    ret = qmi8658_i2c_init();
    if (ret != ESP_OK) return ret;

    /* Reset the QMI8658C device */
    ret = qmi8658_reset();
    if (ret != ESP_OK) return ret;

    /* Verify the device ID */
    ret = qmi8658_verify_device_id();
    if (ret != ESP_OK) return ret;

    /* Set the configuration for the QMI8658C in IMU mode */
    ret = qmi8658_set_config_for_IMU_mode();
    if (ret != ESP_OK) return ret;

    /* Enable the accelerometer and gyroscope */
    ret = qmi8658_enable_accel_and_gyro();
    if (ret != ESP_OK) return ret;

    snprintf(log_buffer, sizeof(log_buffer), "IMU: QMI8658C configured successfully as an IMU.");
    web_server_print(log_buffer);

    /* Create the task to read QMI8658C data */
    BaseType_t status_task = xTaskCreate(task_read_qmi8658_data, "accel_gyro_data_read", TASK_READ_QMI8658_DATA_STACK_SIZE, NULL, TASK_READ_QMI8658_DATA_PRIORITY, NULL);
    if(status_task != pdPASS)
    {
        snprintf(log_buffer, sizeof(log_buffer), "IMU: Failed to create task for reading QMI8658C data");
        web_server_print(log_buffer);
        i2c_master_bus_rm_device(qmi8658_dev_handle);
        return ESP_FAIL;
    }

    return ret; // Should return ESP_OK if everything is successful, otherwise it would have returned earlier with an error code
}

static esp_err_t qmi8658_i2c_init(void)
{
    esp_err_t ret = ESP_OK;
    char log_buffer[256]; // Buffer for formatted log messages

    /* Get I2C bus handle */
    i2c_master_bus_handle_t i2c_manager_bus_handle = NULL;
    ret = i2c_manager_get_bus_handle(&i2c_manager_bus_handle);
    if (ret != ESP_OK)
    {
        snprintf(log_buffer, sizeof(log_buffer), "IMU: Failed to get I2C bus handle: %s", esp_err_to_name(ret));
        web_server_print(log_buffer);
        return ret;
    }

    /* Initialize QMI8658 accelerometer and gyroscope */
    /* Define the configuration for the QMI8658C device */
    i2c_device_config_t dev_cfg = 
    {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,      // 7-bit address length
        .device_address = QMI8658_I2C_ADDR_DEFAULT, // Default I2C address for QMI8658C
        .scl_speed_hz = 400000,                     // I2C Fast Mode supported up to 400 kHz 
        .scl_wait_us = 0,                           // Use default wait time
        .flags.disable_ack_check = false,           // Enable ACK check 
    };

    /* Add the QMI8658C device to the I2C bus */
    ret = i2c_master_bus_add_device(i2c_manager_bus_handle, &dev_cfg, &qmi8658_dev_handle);
    if (ret != ESP_OK) 
    {
        snprintf(log_buffer, sizeof(log_buffer), "IMU: Failed to add QMI8658C device to I2C bus");
        web_server_print(log_buffer);
        return ret;
    }

    return ret; // Should return ESP_OK if everything is successful, otherwise it would have returned earlier with an error code
}

static esp_err_t qmi8658_reset(void)
{
    esp_err_t ret = ESP_OK;
    char log_buffer[256]; // Buffer for formatted log messages

    /* Soft reset the device to ensure a clean startup */
    ret = qmi8658_write_reg(QMI8658_RESET_REG, 0xB0); // Writing 0xB0 triggers a reset
    if (ret != ESP_OK)
    {
        snprintf(log_buffer, sizeof(log_buffer), "IMU: Failed to reset QMI8658C: %s", esp_err_to_name(ret));
        web_server_print(log_buffer);
        i2c_master_bus_rm_device(qmi8658_dev_handle);
        return ret;
    }

    /* Wait for the device to boot up. The datasheet specifies 150ms system turn-on time. */
    vTaskDelay(pdMS_TO_TICKS(150));

    return ret; // Should return ESP_OK if everything is successful, otherwise it would have returned earlier with an error code
}

static esp_err_t qmi8658_verify_device_id(void) 
{
    esp_err_t ret = ESP_OK;
    char log_buffer[256]; // Buffer for formatted log messages

    /* Verify the device ID */
    uint8_t chip_id = 0;
    ret = qmi8658_read_reg(QMI8658_WHO_AM_I_REG, &chip_id, 1);
    if (ret != ESP_OK) 
    {
        snprintf(log_buffer, sizeof(log_buffer), "IMU: Failed to read WHO_AM_I register");
        web_server_print(log_buffer);
        i2c_master_bus_rm_device(qmi8658_dev_handle);
        return ret;
    }
    if (chip_id != QMI8658_DEVICE_ID) 
    {
        snprintf(log_buffer, sizeof(log_buffer), "IMU: Device ID mismatch! Expected 0x%02X, got 0x%02X", QMI8658_DEVICE_ID, chip_id);
        web_server_print(log_buffer);
        i2c_master_bus_rm_device(qmi8658_dev_handle);
        return ESP_FAIL;
    }

    return ret; // Should return ESP_OK if everything is successful, otherwise it would have returned earlier with an error code
}

static esp_err_t qmi8658_set_config_for_IMU_mode(void)
{
    esp_err_t ret = ESP_OK;
    char log_buffer[256]; // Buffer for formatted log messages

    /* Configure sensor settings */
    /* Configure Accelerometer settings (CTRL2) */
    if (qmi8658_write_reg(QMI8658_CTRL2_REG, QMI8658_ACCEL_CONFIG) != ESP_OK) 
    {
         snprintf(log_buffer, sizeof(log_buffer), "IMU: Failed to configure accelerometer (CTRL2)");
         web_server_print(log_buffer);
         return ESP_FAIL;
    }
    /* Configure Gyroscope settings (CTRL3) */
    if (qmi8658_write_reg(QMI8658_CTRL3_REG, QMI8658_GYRO_CONFIG) != ESP_OK) 
    {
        snprintf(log_buffer, sizeof(log_buffer), "IMU: Failed to configure gyroscope (CTRL3)");
        web_server_print(log_buffer);
        return ESP_FAIL;
    }
    /* Configure address auto-increment for multi-byte reads (CTRL1) */
    if (qmi8658_write_reg(QMI8658_CTRL1_REG, QMI8658_CTRL1_CONFIG) != ESP_OK) 
    {
        snprintf(log_buffer, sizeof(log_buffer), "IMU: Failed to configure auto-increment (CTRL1)");
        web_server_print(log_buffer);
        return ESP_FAIL;
    }

    return ret; // Should return ESP_OK if everything is successful, otherwise it would have returned earlier with an error code
}

static esp_err_t qmi8658_enable_accel_and_gyro(void)
{
    esp_err_t ret = ESP_OK;
    char log_buffer[256]; // Buffer for formatted log messages

    /* Enable the accelerometer and gyroscope (CTRL7) */
    if (qmi8658_write_reg(QMI8658_CTRL7_REG, QMI8658_SENSOR_ENABLE) != ESP_OK) 
    {
        snprintf(log_buffer, sizeof(log_buffer), "IMU: Failed to enable sensors (CTRL7)");
        web_server_print(log_buffer);
        return ESP_FAIL;
    }

    /* A short delay to allow the sensors to stabilize after being enabled */
    vTaskDelay(pdMS_TO_TICKS(20));

    return ret; // Should return ESP_OK if everything is successful, otherwise it would have returned earlier with an error code
}

static esp_err_t qmi8658_write_reg(uint8_t reg_addr, uint8_t data) 
{
    uint8_t write_buf[2] = {reg_addr, data};
    
    return i2c_master_transmit(qmi8658_dev_handle, write_buf, sizeof(write_buf), -1);
}


static esp_err_t qmi8658_read_reg(uint8_t reg_addr, uint8_t *data, size_t data_size)
{
    return i2c_master_transmit_receive(qmi8658_dev_handle, &reg_addr, 1, data, data_size, -1);
}


static void task_read_qmi8658_data(void* pvParameters) 
{
    /********************* Task Initialization ***************************/ 
    // TODO: Add variables to store calibration offsets, calculated at startup
    // float gyro_x_bias = 0.0, gyro_y_bias = 0.0, gyro_z_bias = 0.0;

    /* Sensitivity values from datasheet for ±8g and ±2048dps */
    const float ACCEL_SENSITIVITY = 4096.0f; // LSB/g for ±8g
    const float GYRO_SENSITIVITY = 16.0f;    // LSB/dps for ±2048dps

    TickType_t xLastWakeTime = xTaskGetTickCount();

    /********************* Task Loop ***************************/
    while (1)
    {
        char log_buffer[128]; // Buffer for formatted log messages
        uint8_t sensor_data_buffer[12];
        int16_t accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;

        /* Perform a burst read of accelerometer and gyroscope data 
           This covers the following QMI8658C registers (0x35 to 0x40):
               - accel: AX_L, AX_H, AY_L, AY_H, AZ_L, AZ_H,
               - gyro: GX_L, GX_H, GY_L, GY_H, GZ_L, GZ_H 
        */
        esp_err_t read_status = qmi8658_read_reg(QMI8658_REG_AX_L, sensor_data_buffer, sizeof(sensor_data_buffer));

        if (read_status == ESP_OK)
        {
            /* Extract accelerometer and gyroscope data and combine the high and low bytes into 16-bit integers */
            accel_x = (int16_t)((sensor_data_buffer[1] << 8) | sensor_data_buffer[0]);
            accel_y = (int16_t)((sensor_data_buffer[3] << 8) | sensor_data_buffer[2]);
            accel_z = (int16_t)((sensor_data_buffer[5] << 8) | sensor_data_buffer[4]);
            gyro_x  = (int16_t)((sensor_data_buffer[7] << 8) | sensor_data_buffer[6]);
            gyro_y  = (int16_t)((sensor_data_buffer[9] << 8) | sensor_data_buffer[8]);
            gyro_z  = (int16_t)((sensor_data_buffer[11] << 8) | sensor_data_buffer[10]);

            /* Convert to Physical Units (g and dps) */
            float ax = (float)accel_x / ACCEL_SENSITIVITY;
            float ay = (float)accel_y / ACCEL_SENSITIVITY;
            float az = (float)accel_z / ACCEL_SENSITIVITY;
            float gx = (float)gyro_x / GYRO_SENSITIVITY;
            float gy = (float)gyro_y / GYRO_SENSITIVITY;
            float gz = (float)gyro_z / GYRO_SENSITIVITY;

            /* Apply Calibration (Subtract Bias) */
            // gx -= gyro_x_bias;
            // gy -= gyro_y_bias;
            // gz -= gyro_z_bias;
            // TODO: Apply accelerometer calibration

            /* Filter the data (e.g., Low-Pass Filter) */
            // TODO: Implement a filter for smoother output

            /* --- Step 4: Use the processed data --- */
            //snprintf(log_buffer, sizeof(log_buffer), "IMU Data - Accel: [%.2f, %.2f, %.2f] g, Gyro: [%.2f, %.2f, %.2f] dps", ax, ay, az, gx, gy, gz);
            //web_server_print(log_buffer);
            
        }
        else
        {
            snprintf(log_buffer, sizeof(log_buffer), "IMU: Failed to read sensor data");
            web_server_print(log_buffer);
        }

        vTaskDelayUntil(&xLastWakeTime, TASK_READ_QMI8658_DATA_PERIOD_TICKS); // Task should execute periodically, precisely (hence the use of vTaskDelayUntil)
    }
}
