/******************************************************************************
 * @file IMU.h
 * @brief Header file for the IMU component.
 *
 * @details The onboard 9-axis IMU on the Waveshare General Driver for Robots
 *          board (https://www.waveshare.com/general-driver-for-robots.htm) consists of:
 *              - AK09918 magnetometer: https://www.akm.com/content/dam/documents/products/electronic-compass/ak09918c/ak09918c-en-datasheet.pdf
 *              - QMI8658 accelerometer/gyroscope: https://qstcorp.com/upload/pdf/202202/QMI8658C%20datasheet%20rev%200.9.pdf
 *          The ESP32 can communicate with both sensors via I2C:
 *             - AK09918: I2C address 0x0C
 *             - QMI8658: I2C address 0x6A (or 0x6B if SA0 pin is pulled down)
 *
 * 9-Axis IMU (Inertial Measurement Unit) Summary:
 * A 9-axis IMU is a sensor module that provides data from three types of
 * 3-axis sensors:
 *
 * 1. 3-axis Accelerometer (QMI8658):
 *    - Measures linear acceleration along X, Y, Z axes (e.g., movement, shocks).
 *    - Provides an absolute reference for PITCH and ROLL angles by sensing
 *      the Earth's gravitational force (which always points 'down').
 *
 * 2. 3-axis Gyroscope (QMI8658):
 *    - Measures angular velocity (rotational speed) around X, Y, Z axes.
 *    - Tracks changes in orientation (rotations in pitch, roll, and yaw) (like in an airplane).
 *    - Important for dynamic movements, but prone to 'drift' over time due
 *      to accumulated errors, meaning its absolute orientation estimate can
 *      become inaccurate without external correction.
 *
 * 3. 3-axis Magnetometer (AK09918):
 *    - Measures the strength and direction of the surrounding magnetic field.
 *    - Its primary role is to detect the Earth's magnetic field, acting like
 *      a digital compass.
 *    - Provides an ABSOLUTE reference for the HEADING (YAW) angle.
 *    - Crucially, it's used in sensor fusion algorithms to correct the drift
 *      of the gyroscope, especially in the yaw axis, providing a stable and
 *      accurate orientation over long periods.
 *
 * Overall Purpose:
 * The combination of these three sensor types allows the IMU to achieve a
 * robust and accurate understanding of an object's ABSOLUTE ORIENTATION in
 * 3D space (pitch, roll, and yaw). It combines relative motion data with
 * absolute references (gravity and Earth's magnetic field) to compensate for
 * sensor drift. This is vital for applications requiring precise 3D
 * positioning and attitude control, such as in robotics, drones, and VR systems.
 * 
 ******************************************************************************/

#ifndef IMU_H
#define IMU_H

/*******************************************************************************/
/*                                 INCLUDES                                    */
/*******************************************************************************/
/*     Include headers required for the declarations in *this* header file     */
/*                 (e.g., types used in function prototypes).                  */
/*       Prefer forward declarations over full includes where possible         */
/*             to minimize dependencies (for structs, enums etc.).             */
/*******************************************************************************/

/* C Standard Libraries */
#include <stdint.h>
#include <stdbool.h> 

/* ESP-IDF Libraries */
#include "esp_err.h"

/*******************************************************************************/
/*                                  MACROS                                     */
/*******************************************************************************/

/*******************************************************************************/
/*                                DATA TYPES                                   */
/*******************************************************************************/

/*******************************************************************************/
/*                     GLOBAL VARIABLES DECLARATIONS                           */
/*******************************************************************************/
/*   for variables defined in the corresponding .c file, for use in other .c   */
/*      files just by including this header file. Use extern for these.        */
/*******************************************************************************/

/*******************************************************************************/
/*                     GLOBAL FUNCTION DECLARATIONS                            */
/*******************************************************************************/
/*   for functions defined in the corresponding .c file, for use in other .c   */
/*    files just by including this header file. Extern is a default linkage    */
/*    specifier for functions, so it is not necessary to use it explicitly.    */
/*******************************************************************************/

esp_err_t imu_init(void);



#endif /* IMU_H */