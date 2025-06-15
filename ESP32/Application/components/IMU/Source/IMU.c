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
#define QMI8658_I2C_ADDR_DEFAULT    0x6A // assuming SA0 is high (default)

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

/*******************************************************************************/
/*                             STATIC VARIABLES                                */
/*******************************************************************************/

/*******************************************************************************/
/*                     GLOBAL FUNCTION DEFINITIONS                             */
/*******************************************************************************/

esp_err_t imu_init(void)
{
    /* Get I2C bus handle */
    i2c_master_bus_handle_t i2c_manager_bus_handle = NULL;
    esp_err_t status = i2c_manager_get_bus_handle(&i2c_manager_bus_handle);
    if (status != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get I2C bus handle: %s", esp_err_to_name(status));
        return status;
    }

    /* Initialize QMI8658 accelerometer and gyroscope */



    /* Initialize AK09918 magnetometer */
    
    return ESP_OK;
}

/*******************************************************************************/
/*                     STATIC FUNCTION DEFINITIONS                             */
/*******************************************************************************/
