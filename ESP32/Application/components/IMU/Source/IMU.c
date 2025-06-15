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
 * @brief Write a single byte to a specific register on the QMI8658C.
 *
 * @param reg_addr The register address to write to.
 * @param data The byte of data to write.
 * @return ESP_OK on success, ESP_FAIL on failure.
 */
static esp_err_t qmi8658_write_reg(uint8_t reg_addr, uint8_t data) 
{
    uint8_t write_buf[2] = {reg_addr, data};
    
    return i2c_master_transmit(qmi8658_dev_handle, write_buf, sizeof(write_buf), -1);
}

/**
 * @brief Read a single byte from a specific register on the QMI8658C.
 *
 * @param reg_addr The register address to read from.
 * @param data Pointer to store the read data.
 * @return ESP_OK on success, ESP_FAIL on failure.
 */
static esp_err_t qmi8658_read_reg(uint8_t reg_addr, uint8_t *data) 
{
    return i2c_master_transmit_receive(qmi8658_dev_handle, &reg_addr, 1, data, 1, -1);
}


/*******************************************************************************/
/*                     GLOBAL FUNCTION DEFINITIONS                             */
/*******************************************************************************/

esp_err_t imu_init(void)
{
    esp_err_t ret = ESP_OK;

    /* Get I2C bus handle */
    i2c_master_bus_handle_t i2c_manager_bus_handle = NULL;
    ret = i2c_manager_get_bus_handle(&i2c_manager_bus_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get I2C bus handle: %s", esp_err_to_name(ret));
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
        ESP_LOGE(TAG, "Failed to add QMI8658C device to I2C bus");
        return ret;
    }
    else
    {
        ESP_LOGI(TAG, "QMI8658C device added to I2C bus successfully");
    }

    /* Soft reset the device to ensure a clean startup */
    ESP_LOGI(TAG, "Resetting device...");
    ret = qmi8658_write_reg(QMI8658_RESET_REG, 0xB0); // Writing 0xB0 triggers a reset 
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed to reset QMI8658C");
        i2c_master_bus_rm_device(qmi8658_dev_handle);
        return ret;
    }
    /* Wait for the device to boot up. The datasheet specifies 150ms system turn-on time. */
    vTaskDelay(pdMS_TO_TICKS(150));

    /* Verify the device ID */
    uint8_t chip_id = 0;
    ret = qmi8658_read_reg(QMI8658_WHO_AM_I_REG, &chip_id);
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I register");
        i2c_master_bus_rm_device(qmi8658_dev_handle);
        return ret;
    }
    if (chip_id != QMI8658_DEVICE_ID) 
    {
        ESP_LOGE(TAG, "Device ID mismatch! Expected 0x%02X, got 0x%02X", QMI8658_DEVICE_ID, chip_id);
        i2c_master_bus_rm_device(qmi8658_dev_handle);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Device ID verified successfully: 0x%02X", chip_id);

    /* Configure sensor settings */
    ESP_LOGI(TAG, "Configuring sensor registers for IMU operation...");

    /* Configure Accelerometer settings (CTRL2) */
    if (qmi8658_write_reg(QMI8658_CTRL2_REG, QMI8658_ACCEL_CONFIG) != ESP_OK) 
    {
         ESP_LOGE(TAG, "Failed to configure accelerometer (CTRL2)");
         return ESP_FAIL;
    }

    /* Configure Gyroscope settings (CTRL3) */
    if (qmi8658_write_reg(QMI8658_CTRL3_REG, QMI8658_GYRO_CONFIG) != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed to configure gyroscope (CTRL3)");
        return ESP_FAIL;
    }

    /* Configure address auto-increment for multi-byte reads (CTRL1) */
    if (qmi8658_write_reg(QMI8658_CTRL1_REG, QMI8658_CTRL1_CONFIG) != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed to configure auto-increment (CTRL1)");
        return ESP_FAIL;
    }

    /* Enable the accelerometer and gyroscope (CTRL7) */
    if (qmi8658_write_reg(QMI8658_CTRL7_REG, QMI8658_SENSOR_ENABLE) != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed to enable sensors (CTRL7)");
        return ESP_FAIL;
    }

    /* A short delay to allow the sensors to stabilize after being enabled */
    vTaskDelay(pdMS_TO_TICKS(20));
    ESP_LOGI(TAG, "QMI8658C configured successfully as an IMU.");

    /* Initialize AK09918 magnetometer */
    /* TODO */
    
    return ESP_OK;
}

/*******************************************************************************/
/*                     STATIC FUNCTION DEFINITIONS                             */
/*******************************************************************************/
