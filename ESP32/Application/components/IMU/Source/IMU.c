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

    /* Verify the device ID */
    uint8_t chip_id = 0;
    ret = qmi8658_read_reg(QMI8658_WHO_AM_I_REG, &chip_id);
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
    /* Enable the accelerometer and gyroscope (CTRL7) */
    if (qmi8658_write_reg(QMI8658_CTRL7_REG, QMI8658_SENSOR_ENABLE) != ESP_OK) 
    {
        snprintf(log_buffer, sizeof(log_buffer), "IMU: Failed to enable sensors (CTRL7)");
        web_server_print(log_buffer);
        return ESP_FAIL;
    }

    /* A short delay to allow the sensors to stabilize after being enabled */
    vTaskDelay(pdMS_TO_TICKS(20));

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
    
    /* Initialize AK09918 magnetometer */
    /* TODO */
    
    return ESP_OK;
}

/*******************************************************************************/
/*                     STATIC FUNCTION DEFINITIONS                             */
/*******************************************************************************/

static void task_read_qmi8658_data(void* pvParameters) 
{
    /********************* Task Initialization ***************************/ 
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        // TODO: Read sensor data from QMI8658C registers 0x35-0x40 and process it

        vTaskDelayUntil(&xLastWakeTime, TASK_READ_QMI8658_DATA_PERIOD_TICKS); // Task should execute periodically, precisely (hence the use of vTaskDelayUntil)
    }

}
