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
#include <stdio.h>
#include <math.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"

/* Project Includes */
#include "IMU.h"
#include "i2c_manager.h"
#include "web_server.h"
#include "Fusion.h"

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
#define QMI8658_CTRL5_REG           0x06 // Low pass filter settings
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

/* CTRL5: Enable and configure Low-Pass Filters for Accelerometer and Gyroscope
 * ODR means Output Data Rate - it's the number of times per second the sensor
 * provides a new measurement (Hz). Based on CTRL2 and CTRL3 settings, the ODR is 940 Hz.
 * Based on that, the Low-Pass Filter (LPF) bandwidth (BW) is set as a percentage of the ODR.
 * BW is essentially the cutoff frequency of the filter. This means that if for example
 * we use a MODE with a BW of 13.37% of ODR, the cutoff frequency will be 13.37% of 940 Hz (125.7 Hz)
 *
 * Stage 1 (LPF): The physical signal goes through the filter first. This is where noise and unwanted high frequencies are removed.
 * Stage 2 (ODR): The now-clean signal gets sampled at a specific rate (the ODR) to be turned into the digital numbers your program reads.
 * 
 * gLPF_EN<4> = 1 (Enable Gyro LPF)
 * gLPF_MODE<6:5> = 11 (BW = 13.37% of ODR)
 * aLPF_EN<0> = 1 (Enable Accel LPF)
 * aLPF_MODE<2:1> = 11 (BW = 13.37% of ODR)
 */
#define QMI8658_FILTER_CONFIG       0b01110111

/* CTRL1: Enable auto-increment for I2C address for burst reads
 * ADDR_AI = 1  */
#define QMI8658_CTRL1_CONFIG        0b01000000

/* CTRL7: Enable Accelerometer and Gyroscope
 * aEN = 1 (Enable Accelerometer) 
 * gEN = 1 (Enable Gyroscope) */
#define QMI8658_SENSOR_ENABLE       0b00000011

/* --- AK09918 Register Definitions & Configuration --- */

/* The 7-bit I2C slave address for the AK09918 */
#define AK09918_I2C_ADDR_DEFAULT    0x0C

/* Register addresses for AK09918 */
#define AK09918_WIA1_REG            0x00 // Company ID register (0x48) (WIA == Who I Am)
#define AK09918_WIA2_REG            0x01 // Device ID register (0x0C) 
#define AK09918_ST1_REG             0x10 // Status 1 register
#define AK09918_HXL_REG             0x11 // X-axis magnetic data low byte
#define AK09918_HXH_REG             0x12 // X-axis magnetic data high byte
#define AK09918_HYL_REG             0x13 // Y-axis magnetic data low byte
#define AK09918_HYH_REG             0x14 // Y-axis magnetic data high byte
#define AK09918_HZL_REG             0x15 // Z-axis magnetic data low byte
#define AK09918_HZH_REG             0x16 // Z-axis magnetic data high byte
#define AK09918_TMPS_REG            0x17 // Dummy register
#define AK09918_ST2_REG             0x18 // Status 2 register
#define AK09918_CNTL2_REG           0x31 // Control 2 register (mode setting)
#define AK09918_CNTL3_REG           0x32 // Control 3 register (soft reset)

/* Expected Device IDs */
#define AK09918_COMPANY_ID          0x48 // Company ID (AKM)
#define AK09918_DEVICE_ID           0x0C // Device ID for AK09918

/* Control register values */
#define AK09918_SOFT_RESET          0x01 // Soft reset bit in CNTL3
#define AK09918_MODE_POWER_DOWN     0x00 // Power-down mode
#define AK09918_MODE_SINGLE         0x01 // Single measurement mode - Sensor is measured for one time and data is output. Transits to Power-down mode automatically after measurement ended.
#define AK09918_MODE_CONT_1         0x02 // Continuous mode 1 - Sensor is measured periodically at 10Hz
#define AK09918_MODE_CONT_2         0x04 // Continuous mode 2 - Sensor is measured periodically at 20Hz
#define AK09918_MODE_CONT_3         0x06 // Continuous mode 3 - Sensor is measured periodically at 50Hz
#define AK09918_MODE_CONT_4         0x08 // Continuous mode 4 - Sensor is measured periodically at 100Hz
#define AK09918_MODE_SELF_TEST      0x10 // Self-test mode - Sensor is self-tested and the result is output. Transits to Power-down mode automatically.

/* Status register bits */
#define AK09918_ST1_DRDY_BIT        0x01 // Data ready bit
#define AK09918_ST1_DOR_BIT         0x02 // Data overrun bit
#define AK09918_ST2_HOFL_BIT        0x08 // Magnetic sensor overflow bit

/* FreeRTOS task configuration */
#define TASK_READ_QMI8658_DATA_PRIORITY (tskIDLE_PRIORITY + 1)
#define TASK_READ_QMI8658_DATA_STACK_SIZE 4096 // Stack size in bytes
#define TASK_READ_QMI8658_DATA_PERIOD_TICKS pdMS_TO_TICKS(50) // Task period in ticks (50 ms)
#define TASK_READ_AK09918_DATA_PRIORITY (tskIDLE_PRIORITY + 1)
#define TASK_READ_AK09918_DATA_STACK_SIZE 4096 // Stack size in bytes
#define TASK_READ_AK09918_DATA_PERIOD_TICKS pdMS_TO_TICKS(50) // Task period in ticks (50 ms)

/* Fusion AHRS algorithm settings */
/* Sample period matches the task period (50ms = 0.05s) */
#define FUSION_SAMPLE_PERIOD 0.05f
/* Beta Gain values for Madgwick IMU sensor fusion algorithm */
/* This parameter controls the trade-off between trusting gyroscope data
 * (for smooth, short-term accuracy) and accelerometer/magnetometer corrections
 * (for long-term drift prevention).
 * 
 *      Increase beta: Faster correction of gyro drift, better for dynamic systems
 *                     with reliable acc/mag sensors, but may amplify sensor noise or disturbances.
 *      Decrease beta: Smoother, more stable output relying more on gyro integration, ideal for
 *                     low-noise gyros but risks gradual orientation drift over time.
 *
 * Tune by starting with ~0.04 (per Madgwick's paper), test in your setup, and
 * adjust based on observed drift vs. noise in orientation estimates.
 * 
 * If orientation drifts over time, INCREASE beta. If orientation is too jittery during movement, DECREASE beta. */

/* Default values from Madgwick's paper: https://courses.cs.washington.edu/courses/cse466/14au/labs/l4/madgwick_internal_report.pdf
#define BETA_IMU_DEFAULT 0.033f    // Default for IMU (gyro + accelerometer)
#define BETA_MARG_DEFAULT 0.041f   // Default for MARG (gyro + accel + mag) */
#define FUSION_GAIN 0.5f
/* Gyroscope range in degrees per second (matches our ±2048 dps setting) */
#define FUSION_GYRO_RANGE 2048.0f
/* Acceleration rejection threshold - ignores accelerometer during high acceleration */
#define FUSION_ACCEL_REJECTION 10.0f
/* Magnetic rejection threshold - ignores magnetometer during magnetic interference */
#define FUSION_MAGNETIC_REJECTION 10.0f
/* Recovery trigger period - 5 seconds worth of samples at 20Hz */
#define FUSION_RECOVERY_TRIGGER_PERIOD (5 * (1000 / 50))

/*******************************************************************************/
/*                                DATA TYPES                                   */
/*******************************************************************************/

/* Data structure to hold IMU sensor data */
typedef struct
{
    float ax; // Accelerometer X-axis value in g
    float ay; // Accelerometer Y-axis value in g
    float az; // Accelerometer Z-axis value in g
    float gx; // Gyroscope X-axis value in dps
    float gy; // Gyroscope Y-axis value in dps
    float gz; // Gyroscope Z-axis value in dps
} imu_sensor_data_t;

/* Data structure to hold fused IMU orientation data */
typedef struct
{
    float roll;  // Roll angle in degrees
    float pitch; // Pitch angle in degrees
    float yaw;   // Yaw angle in degrees
} imu_orientation_t;


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
/* I2C device handles for the sensors */
static i2c_master_dev_handle_t qmi8658_dev_handle = NULL;
static i2c_master_dev_handle_t ak09918_dev_handle = NULL;

/* Sensitivity values from datasheet for ±8g and ±2048dps */
static const float ACCEL_SENSITIVITY = 4096.0f; // LSB/g for ±8g
static const float GYRO_SENSITIVITY = 16.0f;    // LSB/dps for ±2048dps

/* Sensitivity for magnetic field conversion */
static const float MAG_SENSITIVITY = 0.15f; // μT/LSB (typical)

/* Fusion AHRS instances for sensor fusion */
static FusionOffset fusion_offset;
static FusionAhrs fusion_ahrs;

/* Shared magnetometer data for sensor fusion */
static float magnetometer_x = 0.0f;
static float magnetometer_y = 0.0f;
static float magnetometer_z = 0.0f;
static bool magnetometer_data_available = false;

/*******************************************************************************/
/*                     STATIC FUNCTION DECLARATIONS                            */
/*******************************************************************************/

/**
 * @brief Initialize the Fusion AHRS algorithm.
 * This function initializes the Fusion offset correction and AHRS algorithms
 * with appropriate settings for our IMU configuration.
 * 
 * @return ESP_OK on success, or an error code on failure.
 */
static esp_err_t fusion_init(void);

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
 * @brief Get sensor data from the QMI8658C accelerometer and gyroscope.
 * This function reads the sensor data registers and converts the raw values
 * into physical units (g for accelerometer, dps for gyroscope).
 * 
 * @param sensor_data Pointer to an imu_sensor_data_t structure to store the sensor data.
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if the sensor_data pointer is NULL, or
 *         specific error code from qmi8658_read_reg function.
 */
static esp_err_t qmi8658_get_sensor_data(imu_sensor_data_t *sensor_data);

/**
 * @brief Calibrate the gyroscope data of the QMI8658C.
 * This function applies calibration offsets to the gyroscope data.
 * Calibration data is obtained during first few seconds of operation. Robot should be stationary during this time.
 *
 * @param sensor_data Pointer to an imu_sensor_data_t structure containing the gyroscope data to be calibrated.
 * @return ESP_OK on success, or an ESP_ERR_INVALID_ARG if the sensor_data pointer is NULL.
 * 
 * @note This function needs to be called on every read gyro data, it's not a one-time calibration.
 */
static esp_err_t qmi8658_calibrate_gyro_data(imu_sensor_data_t *sensor_data);

/**
 * @brief Task to read data from the QMI8658C accelerometer and gyroscope.
 * This task will periodically read sensor data and process it.
 *
 * @param pvParameters Pointer to task parameters (not used).
 * @return void
 */
static void task_read_qmi8658_data(void* pvParameters);

/**
 * @brief Initialize the AK09918 magnetometer.
 * This function initializes the I2C communication, configures the AK09918
 * device, enables it and then creates a task to read data from the magnetometer.
 *
 * @return ESP_OK on success, or an error code on failure.
 */
static esp_err_t ak09918_init(void);

/**
 * @brief Initialize the I2C bus for the AK09918 magnetometer.
 * This function acquires the I2C bus handle, configures the device address,
 * and sets the I2C speed. After that, it adds the AK09918 device to the I2C bus.
 *
 * @return ESP_OK on success, or an error code on failure.
 */
static esp_err_t ak09918_i2c_init(void);

/**
 * @brief Reset the AK09918 magnetometer.
 * This function performs a reset of the AK09918 device.
 *
 * @return ESP_OK on success, or an error code on failure.
 */
static esp_err_t ak09918_reset(void);

/**
 * @brief Verify the device ID of the AK09918 magnetometer.
 * This function reads the WIA1 and WIA2 registers and checks two things:
 *      1. The WIA1 register should contain the value 0x48 (the company ID).
 *      2. The WIA2 register should contain the value 0x09 (the device ID).
 * If either of these checks fails, it returns an error.
 *
 * @return ESP_OK if the device ID matches, or an error code on failure.
 */
static esp_err_t ak09918_verify_device_id(void);

/**
 * @brief Set the mode of the AK09918 magnetometer.
 * This function writes a specific mode to the CNTL2 register to set the
 * desired operating mode of the magnetometer (e.g., power-down, single measurement,
 * continuous measurement).
 *
 * @param mode The mode to set (e.g., AK09918_MODE_CONT_1 for continuous mode 1).
 * @return ESP_OK on success, or an error code on failure.
 */
static esp_err_t ak09918_set_mode(uint8_t mode);

/**
 * @brief Write a value to a register of the AK09918 magnetometer.
 * This function writes a single byte of data to a specified register address.
 *
 * @param reg_addr The register address to write to.
 * @param data The data to write to the register.
 * @return ESP_OK on success, or an error code on failure.
 */
static esp_err_t ak09918_write_reg(uint8_t reg_addr, uint8_t data);

/**
 * @brief Read data from a register of the AK09918 magnetometer.
 * This function reads a specified number of bytes from a given register address.
 *
 * @param reg_addr The register address to read from.
 * @param data Pointer to store the read data.
 * @param data_size The number of bytes to read.
 * @return ESP_OK on success, or an error code on failure.
 */
static esp_err_t ak09918_read_reg(uint8_t reg_addr, uint8_t *data, size_t data_size);

/**
 * @brief Get the magnetic field data from the AK09918 magnetometer.
 * This function checks if the data is available (DRDY) and if there has 
 * been data overrun (DOR). Overrun meaning we haven't read the previous
 * data in time and a new value replaced it (we are reading too slow, or 
 * data is produced too fast - check your task period and the magnetometer mode).
 * If all is well, then it reads the magnetic field data from the AK09918 
 * registers and converts it to microteslas (uT).
 *
 * @param mx Pointer to store the X-axis magnetic field data.
 * @param my Pointer to store the Y-axis magnetic field data.
 * @param mz Pointer to store the Z-axis magnetic field data.
 * @return ESP_OK on success, or an error code on failure.
 */
static esp_err_t ak09918_get_magnetic_data(float *mx, float *my, float *mz);

/**
 * @brief Task to read data from the AK09918 magnetometer.
 * This task will periodically read magnetic field data and process it.
 *
 * @param pvParameters Pointer to task parameters (not used).
 * @return void
 */
static void task_read_ak09918_data(void* pvParameters);

/**
 * @brief Update the shared magnetometer data from AK09918.
 * This function reads magnetometer data and updates the shared variables
 * used by the sensor fusion algorithm.
 *
 * @return ESP_OK on success, or an error code on failure.
 */
static esp_err_t update_magnetometer_data(void);

/*******************************************************************************/
/*                     GLOBAL FUNCTION DEFINITIONS                             */
/*******************************************************************************/

esp_err_t imu_init(void)
{
    esp_err_t init_status = ESP_OK;

    /* Initialize Fusion AHRS algorithm */
    init_status = fusion_init();
    if (init_status != ESP_OK)
    {
        return init_status;
    }

    init_status = qmi8658_init();
    if (init_status != ESP_OK)
    {
        return init_status;
    }

    init_status = ak09918_init();
    if (init_status != ESP_OK)
    {
        return init_status;
    }

    return ESP_OK;
}

/*******************************************************************************/
/*                     STATIC FUNCTION DEFINITIONS                             */
/*******************************************************************************/

static esp_err_t fusion_init(void)
{
    char log_buffer[256]; // Buffer for formatted log messages
    
    /* Initialize Fusion offset algorithm */
    /* Sample rate is 20Hz (1000ms / 50ms task period) */
    const unsigned int sample_rate = 1000 / 50; // 20 Hz
    FusionOffsetInitialise(&fusion_offset, sample_rate);
    
    /* Initialize Fusion AHRS algorithm */
    FusionAhrsInitialise(&fusion_ahrs);
    
    /* Configure AHRS settings */
    const FusionAhrsSettings settings = {
        .convention = FusionConventionNwu,                    // North-West-Up coordinate system
        .gain = FUSION_GAIN,                                  // Algorithm gain (0.5f)
        .gyroscopeRange = FUSION_GYRO_RANGE,                  // Gyroscope range in degrees/s (2048.0f)
        .accelerationRejection = FUSION_ACCEL_REJECTION,      // Acceleration rejection threshold (10.0f)
        .magneticRejection = FUSION_MAGNETIC_REJECTION,       // Magnetic rejection threshold (10.0f)
        .recoveryTriggerPeriod = FUSION_RECOVERY_TRIGGER_PERIOD, // Recovery trigger period (5 seconds)
    };
    FusionAhrsSetSettings(&fusion_ahrs, &settings);
    
    snprintf(log_buffer, sizeof(log_buffer), "IMU: Fusion AHRS initialized successfully with gain=%.2f", FUSION_GAIN);
    web_server_print(log_buffer);
    
    return ESP_OK;
}

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
        .flags = 
        {
            .disable_ack_check = false              // Enable ACK check 
        }
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
    /* Configure Low-Pass Filters for Accelerometer and Gyroscope (CTRL5) */
    if (qmi8658_write_reg(QMI8658_CTRL5_REG, QMI8658_FILTER_CONFIG) != ESP_OK) 
    {
        snprintf(log_buffer, sizeof(log_buffer), "IMU: Failed to configure filters (CTRL5)");
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


static esp_err_t qmi8658_get_sensor_data(imu_sensor_data_t *sensor_data)
{
    /* Input validation */
    if(sensor_data == NULL) 
    {
        web_server_print("IMU: Invalid sensor data pointer");
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t sensor_data_buffer[12] = {0};
    int16_t accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;

    /* Perform a burst read of accelerometer and gyroscope data 
        This covers the following QMI8658C registers (0x35 to 0x40):
            - accel: AX_L, AX_H, AY_L, AY_H, AZ_L, AZ_H,
            - gyro: GX_L, GX_H, GY_L, GY_H, GZ_L, GZ_H */
    esp_err_t read_status = qmi8658_read_reg(QMI8658_REG_AX_L, sensor_data_buffer, sizeof(sensor_data_buffer));

    if (read_status != ESP_OK) 
    {
        web_server_print("IMU: Failed to read sensor data from QMI8658C");
        return read_status;
    }
    else
    {
        /* Extract accelerometer and gyroscope data and combine the high and low bytes into 16-bit integers */
        accel_x = (int16_t)((sensor_data_buffer[1] << 8) | sensor_data_buffer[0]);
        accel_y = (int16_t)((sensor_data_buffer[3] << 8) | sensor_data_buffer[2]);
        accel_z = (int16_t)((sensor_data_buffer[5] << 8) | sensor_data_buffer[4]);
        gyro_x  = (int16_t)((sensor_data_buffer[7] << 8) | sensor_data_buffer[6]);
        gyro_y  = (int16_t)((sensor_data_buffer[9] << 8) | sensor_data_buffer[8]);
        gyro_z  = (int16_t)((sensor_data_buffer[11] << 8) | sensor_data_buffer[10]);

        /* Convert to Physical Units (g and dps) and store in sensor_data structure */
        sensor_data->ax = (float)accel_x / ACCEL_SENSITIVITY;
        sensor_data->ay = (float)accel_y / ACCEL_SENSITIVITY;
        sensor_data->az = (float)accel_z / ACCEL_SENSITIVITY;
        sensor_data->gx = (float)gyro_x / GYRO_SENSITIVITY;
        sensor_data->gy = (float)gyro_y / GYRO_SENSITIVITY;
        sensor_data->gz = (float)gyro_z / GYRO_SENSITIVITY;
    }

    return ESP_OK; // Return ESP_OK to indicate success, in case of failure the function returns as soon as it encounters an error
}

static esp_err_t qmi8658_calibrate_gyro_data(imu_sensor_data_t *sensor_data)
{
    /* Input validation */
    if(sensor_data == NULL) 
    {
        web_server_print("IMU: Invalid sensor data pointer for calibration");
        return ESP_ERR_INVALID_ARG;
    }

    char log_buffer[128]; // Buffer for formatted log messages
    /* Calibration variables */
    const int CALIBRATION_SAMPLE_COUNT = 40; // This is 4 seconds at 100 ms per sample (TASK_READ_QMI8658_DATA_PERIOD_TICKS)
    static int calibration_count = 0;
    static float gyro_x_bias = 0.0f, gyro_y_bias = 0.0f, gyro_z_bias = 0.0f;
    static bool calibration_values_available = false;

    /* Apply gyroscope calibration if the bias values are available, otherwise accumulate them */
    if (calibration_values_available)
    {
        /* Apply Calibration (Substract Bias) */
        sensor_data->gx -= gyro_x_bias;
        sensor_data->gy -= gyro_y_bias;
        sensor_data->gz -= gyro_z_bias;
    }
    else
    {
        if (calibration_count < CALIBRATION_SAMPLE_COUNT)
        {
            /* Accumulate the gyroscope biases */
            gyro_x_bias += sensor_data->gx;
            gyro_y_bias += sensor_data->gy;
            gyro_z_bias += sensor_data->gz;
            calibration_count++;

            if (calibration_count == 1) 
            {
                web_server_print("IMU: Starting gyroscope calibration... Please keep the device still.");
            }
        }
        else // All samples collected
        {
            /* Calculate the average biases based on the collected samples */
            gyro_x_bias /= CALIBRATION_SAMPLE_COUNT;
            gyro_y_bias /= CALIBRATION_SAMPLE_COUNT;
            gyro_z_bias /= CALIBRATION_SAMPLE_COUNT;

            calibration_values_available = true; 
            snprintf(log_buffer, sizeof(log_buffer), "IMU: Gyro calibrated. Bias: [%.2f, %.2f, %.2f]", gyro_x_bias, gyro_y_bias, gyro_z_bias);
            web_server_print(log_buffer);
        }
    }

    return ESP_OK; // Return ESP_OK to indicate success, in case of failure the function returns as soon as it encounters an error
}

static void task_read_qmi8658_data(void* pvParameters) 
{
    /********************* Task Initialization ***************************/ 
    char log_buffer[128]; // Buffer for formatted log messages

    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    /********************* Task Loop ***************************/
    while (1)
    {
        static imu_sensor_data_t sensor_data;
        imu_orientation_t orientation;

        /* Acquire sensor data from QMI8658C */
        esp_err_t sensor_status = qmi8658_get_sensor_data(&sensor_data);

        if (sensor_status == ESP_OK)
        {
            esp_err_t cal_status = qmi8658_calibrate_gyro_data(&sensor_data);
            if (cal_status != ESP_OK)
            {
                web_server_print("IMU: Failed to calibrate gyroscope data");
            }

            /* Create Fusion vectors from sensor data */
            /* Note: Fusion expects gyroscope data in degrees/s (not radians) */
            FusionVector gyroscope = {sensor_data.gx, sensor_data.gy, sensor_data.gz};
            FusionVector accelerometer = {sensor_data.ax, sensor_data.ay, sensor_data.az};
            FusionVector magnetometer = {magnetometer_x, magnetometer_y, magnetometer_z};

            /* Apply Fusion offset correction to gyroscope (handles bias/drift) */
            gyroscope = FusionOffsetUpdate(&fusion_offset, gyroscope);

            /* Perform sensor fusion using appropriate method */
            if (magnetometer_data_available)
            {
                /* Full 9-DOF sensor fusion with magnetometer */
                FusionAhrsUpdate(&fusion_ahrs, gyroscope, accelerometer, magnetometer, FUSION_SAMPLE_PERIOD);
            }
            else
            {
                /* 6-DOF sensor fusion without magnetometer */
                FusionAhrsUpdateNoMagnetometer(&fusion_ahrs, gyroscope, accelerometer, FUSION_SAMPLE_PERIOD);
            }

            /* Get the fused orientation as Euler angles */
            FusionQuaternion quaternion = FusionAhrsGetQuaternion(&fusion_ahrs);
            FusionEuler euler = FusionQuaternionToEuler(quaternion);
            
            /* Convert from Fusion's coordinate system to our orientation structure */
            /* Note: May need to adjust these mappings based on your coordinate system requirements */
            orientation.roll = euler.angle.roll;
            orientation.pitch = euler.angle.pitch;
            orientation.yaw = euler.angle.yaw;

            /* Broadcast the fused orientation data over WebSocket */
            web_server_ws_broadcast_imu_orientation(orientation.roll, orientation.pitch, orientation.yaw);
        }
        else
        {
            snprintf(log_buffer, sizeof(log_buffer), "IMU: Failed to read sensor data");
            web_server_print(log_buffer);
        }

        vTaskDelayUntil(&xLastWakeTime, TASK_READ_QMI8658_DATA_PERIOD_TICKS); // Task should execute periodically, precisely (hence the use of vTaskDelayUntil)
    }
}

static esp_err_t ak09918_init(void)
{
    esp_err_t status = ESP_OK;
    char log_buffer[256]; // Buffer for formatted log messages

    /* Initialize I2C bus for AK09918 */
    status = ak09918_i2c_init();
    if (status != ESP_OK) return status;

    /* Reset the AK09918 device */
    status = ak09918_reset();
    if (status != ESP_OK) return status;

    /* Verify the device ID */
    status = ak09918_verify_device_id();
    if (status != ESP_OK) return status;

    /* Set to continuous measurement mode 1 (10Hz) for regular operation */
    const uint8_t mode = AK09918_MODE_CONT_1;
    status = ak09918_set_mode(mode);
    if (status != ESP_OK) return status;

    snprintf(log_buffer, sizeof(log_buffer), "IMU: AK09918 magnetometer initialized successfully in continuous mode %d", mode);
    web_server_print(log_buffer);

    /* Create the task to read AK09918 magnetometer data */
    BaseType_t status_task = xTaskCreate(task_read_ak09918_data, "magnetometer_data_read", TASK_READ_AK09918_DATA_STACK_SIZE, NULL, TASK_READ_AK09918_DATA_PRIORITY, NULL);
    if(status_task != pdPASS)
    {
        snprintf(log_buffer, sizeof(log_buffer), "IMU: Failed to create task for reading AK09918 magnetometer data");
        web_server_print(log_buffer);
        i2c_master_bus_rm_device(ak09918_dev_handle);
        return ESP_FAIL;
    }

    return ESP_OK;
}

static esp_err_t ak09918_i2c_init(void)
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

    /* Define the I2C configuration for the AK09918 device */
    i2c_device_config_t dev_cfg = 
    {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,      // 7-bit address length
        .device_address = AK09918_I2C_ADDR_DEFAULT, // Default I2C address for AK09918
        .scl_speed_hz = 400000,                     // AK09918 supports I2C Fast Mode up to 400 kHz
        .scl_wait_us = 0,                           // Use default wait time
        .flags = 
        {
            .disable_ack_check = false              // Enable ACK check
        }
    };

    /* Add the AK09918 device to the I2C bus */
    ret = i2c_master_bus_add_device(i2c_manager_bus_handle, &dev_cfg, &ak09918_dev_handle);
    if (ret != ESP_OK) 
    {
        snprintf(log_buffer, sizeof(log_buffer), "IMU: Failed to add AK09918 device to I2C bus");
        web_server_print(log_buffer);
        return ret;
    }

    return ret; // Should return ESP_OK if everything is successful, otherwise it would have returned earlier with an error code
}

static esp_err_t ak09918_reset(void)
{
    esp_err_t ret = ESP_OK;
    char log_buffer[256];

    /* Perform soft reset */
    ret = ak09918_write_reg(AK09918_CNTL3_REG, AK09918_SOFT_RESET);
    if (ret != ESP_OK)
    {
        snprintf(log_buffer, sizeof(log_buffer), "IMU: Failed to reset AK09918: %s", esp_err_to_name(ret));
        web_server_print(log_buffer);
        i2c_master_bus_rm_device(ak09918_dev_handle);
        return ret;
    }

    /* Wait for reset to complete (datasheet specifies 100μs minimum) */
    vTaskDelay(pdMS_TO_TICKS(10)); // 10ms to be safe

    return ret;
}

static esp_err_t ak09918_verify_device_id(void)
{
    esp_err_t ret = ESP_OK;
    char log_buffer[256];
    uint8_t company_id = 0;
    uint8_t device_id = 0;

    /* Read company ID (WIA1 register) */
    ret = ak09918_read_reg(AK09918_WIA1_REG, &company_id, 1);
    if (ret != ESP_OK)
    {
        snprintf(log_buffer, sizeof(log_buffer), "IMU: Failed to read AK09918 company ID");
        web_server_print(log_buffer);
        i2c_master_bus_rm_device(ak09918_dev_handle);
        return ret;
    }

    /* Read device ID (WIA2 register) */
    ret = ak09918_read_reg(AK09918_WIA2_REG, &device_id, 1);
    if (ret != ESP_OK)
    {
        snprintf(log_buffer, sizeof(log_buffer), "IMU: Failed to read AK09918 device ID");
        web_server_print(log_buffer);
        i2c_master_bus_rm_device(ak09918_dev_handle);
        return ret;
    }

    /* Verify company ID */
    if (company_id != AK09918_COMPANY_ID)
    {
        snprintf(log_buffer, sizeof(log_buffer), "IMU: AK09918 company ID mismatch! Expected 0x%02X, got 0x%02X", 
                 AK09918_COMPANY_ID, company_id);
        web_server_print(log_buffer);
        i2c_master_bus_rm_device(ak09918_dev_handle);
        return ESP_FAIL;
    }

    /* Verify device ID */
    if (device_id != AK09918_DEVICE_ID)
    {
        snprintf(log_buffer, sizeof(log_buffer), "IMU: AK09918 device ID mismatch! Expected 0x%02X, got 0x%02X", 
                 AK09918_DEVICE_ID, device_id);
        web_server_print(log_buffer);
        i2c_master_bus_rm_device(ak09918_dev_handle);
        return ESP_FAIL;
    }

    snprintf(log_buffer, sizeof(log_buffer), "IMU: AK09918 device verification successful (Company: 0x%02X, Device: 0x%02X)", 
             company_id, device_id);
    web_server_print(log_buffer);

    return ESP_OK;
}

static esp_err_t ak09918_set_mode(uint8_t mode)
{
    esp_err_t ret = ESP_OK;
    char log_buffer[256];

    /* Set to power-down mode first (required before changing to any other mode) */
    ret = ak09918_write_reg(AK09918_CNTL2_REG, AK09918_MODE_POWER_DOWN);
    if (ret != ESP_OK)
    {
        snprintf(log_buffer, sizeof(log_buffer), "IMU: Failed to set AK09918 to power-down mode");
        web_server_print(log_buffer);
        return ret;
    }

    /* Wait required time before setting new mode (datasheet specifies 100μs minimum) */
    vTaskDelay(pdMS_TO_TICKS(1)); // 1ms to be safe

    /* Set the desired mode */
    ret = ak09918_write_reg(AK09918_CNTL2_REG, mode);
    if (ret != ESP_OK)
    {
        snprintf(log_buffer, sizeof(log_buffer), "IMU: Failed to set AK09918 mode to 0x%02X", mode);
        web_server_print(log_buffer);
        return ret;
    }

    /* Small delay to allow mode to take effect */
    vTaskDelay(pdMS_TO_TICKS(10));

    return ESP_OK;
}

static esp_err_t ak09918_write_reg(uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    
    return i2c_master_transmit(ak09918_dev_handle, write_buf, sizeof(write_buf), -1);
}

static esp_err_t ak09918_read_reg(uint8_t reg_addr, uint8_t *data, size_t data_size)
{
    return i2c_master_transmit_receive(ak09918_dev_handle, &reg_addr, 1, data, data_size, -1);
}

static esp_err_t ak09918_get_magnetic_data(float *mx, float *my, float *mz)
{
    /* Input validation */
    if(mx == NULL || my == NULL || mz == NULL) 
    {
        web_server_print("IMU: Invalid magnetometer data pointers");
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t status_reg = 0;
    uint8_t mag_data_buffer[7] = {0}; // ST1 + 6 bytes of data
    int16_t mag_x, mag_y, mag_z;

    /* Check if data is ready by reading ST1 register */
    esp_err_t read_status = ak09918_read_reg(AK09918_ST1_REG, &status_reg, 1);
    if (read_status != ESP_OK) 
    {
        web_server_print("IMU: Failed to read AK09918 status register");
        return read_status;
    }

    /* Check DRDY bit - if not ready, return without error but set values to 0 */
    if (!(status_reg & AK09918_ST1_DRDY_BIT))
    {
        *mx = 0.0f;
        *my = 0.0f; 
        *mz = 0.0f;
        return ESP_OK; // Not an error, just no new data available
    }

    /* Check for data overrun */
    if (status_reg & AK09918_ST1_DOR_BIT)
    {
        web_server_print("IMU: AK09918 data overrun detected");
    }

    /* Perform burst read of magnetometer data (HXL to HZH) */
    /* Measurement data is stored in two’s complement and Little Endian format. Measurement range of each axis is -32752 to 32752 in 16-bit output */
    read_status = ak09918_read_reg(AK09918_HXL_REG, mag_data_buffer, 6);
    if (read_status != ESP_OK) 
    {
        web_server_print("IMU: Failed to read AK09918 magnetometer data");
        return read_status;
    }

    /* Read ST2 register to complete the data reading sequence (required by datasheet) */
    /* When ST2 register is read, AK09918 judges that data reading is finished. Stored measurement data is
       protected during data reading and data is not updated. By reading ST2 register, this protection is
       released. It is required to read ST2 register after data reading. */
    uint8_t status2_reg = 0;
    read_status = ak09918_read_reg(AK09918_ST2_REG, &status2_reg, 1);
    if (read_status != ESP_OK) 
    {
        web_server_print("IMU: Failed to read AK09918 ST2 register");
        return read_status;
    }

    /* Check for magnetic sensor overflow */
    /* The magnetic sensor may overflow even though measurement data register is not saturated. In this case, measurement data is not
       correct and HOFL bit turns to “1”. AK09918 has the limitation for measurement range that the sum of absolute values of each axis should be
       smaller than 4912 μT (|X|+|Y|+|Z| < 4912 μT). When the magnetic field exceeded this limitation, data stored at measurement data are not correct.
       This is called Magnetic Sensor Overflow. */
    if (status2_reg & AK09918_ST2_HOFL_BIT)
    {
        web_server_print("IMU: AK09918 magnetic sensor overflow detected");
        *mx = 0.0f;
        *my = 0.0f;
        *mz = 0.0f;
        return ESP_OK; // Don't treat overflow as fatal error, just return zero values
    }

    /* Extract magnetometer data and combine the high and low bytes into 16-bit integers */
    /* Data is in Little Endian format: low byte first, then high byte */
    mag_x = (int16_t)((mag_data_buffer[1] << 8) | mag_data_buffer[0]);
    mag_y = (int16_t)((mag_data_buffer[3] << 8) | mag_data_buffer[2]);
    mag_z = (int16_t)((mag_data_buffer[5] << 8) | mag_data_buffer[4]);

    /* Convert to Physical Units (μT) and store in output variables */
    *mx = (float)mag_x * MAG_SENSITIVITY;
    *my = (float)mag_y * MAG_SENSITIVITY;
    *mz = (float)mag_z * MAG_SENSITIVITY;

    return ESP_OK;
}

static esp_err_t update_magnetometer_data(void)
{
    float mx, my, mz;
    
    /* Acquire magnetometer data from AK09918 */
    esp_err_t sensor_status = ak09918_get_magnetic_data(&mx, &my, &mz);
    
    if (sensor_status == ESP_OK)
    {
        /* Update the shared magnetometer data */
        magnetometer_x = mx;
        magnetometer_y = my;
        magnetometer_z = mz;
        magnetometer_data_available = true;
    }
    else
    {
        /* Mark magnetometer data as unavailable on error */
        magnetometer_data_available = false;
    }
    
    return sensor_status;
}

static void task_read_ak09918_data(void* pvParameters) 
{
    /********************* Task Initialization ***************************/ 
    char log_buffer[256]; // Buffer for formatted log messages
    TickType_t xLastWakeTime = xTaskGetTickCount();

    /********************* Task Loop ***************************/
    while (1)
    {
        /* Update shared magnetometer data for sensor fusion */
        esp_err_t sensor_status = update_magnetometer_data();

        if (sensor_status != ESP_OK)
        {
            snprintf(log_buffer, sizeof(log_buffer), "IMU: Failed to read magnetometer data: %s", esp_err_to_name(sensor_status));
            web_server_print(log_buffer);
        }

        vTaskDelayUntil(&xLastWakeTime, TASK_READ_AK09918_DATA_PERIOD_TICKS); // Task should execute periodically, precisely
    }
}