/******************************************************************************
 * @file roarm_m3_motor_control.h
 * @brief Header file for the RoArm-M3 Motor Control component
 *
 ******************************************************************************/

#ifndef ROARM_M3_MOTOR_CONTROL_H
#define ROARM_M3_MOTOR_CONTROL_H

/*******************************************************************************/
/*                                 INCLUDES                                    */
/*******************************************************************************/
/*     Include headers required for the declarations in *this* header file     */
/*                 (e.g., types used in function prototypes).                  */
/*       Prefer forward declarations over full includes where possible         */
/*             to minimize dependencies (for structs, enums etc.).             */
/*******************************************************************************/

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_now.h"
#include "nvs_flash.h"

/*******************************************************************************/
/*                                  MACROS                                     */
/*******************************************************************************/

#define MAX_MESSAGE_LEN 200

/*******************************************************************************/
/*                                DATA TYPES                                   */
/*******************************************************************************/

/*
 * ESP-NOW Message Structure for RoArm-M3 Control
 * 
 * This structure matches the Arduino project's struct_message defined in esp_now_ctrl.h
 * and is used to send control commands wirelessly via ESP-NOW protocol.
 * 
 * Fields:
 * 
 * devCode (uint8_t):
 *   - Device identifier for multi-arm control
 *   - Typically set to 0 for single device setups
 *   - Used in group control scenarios to address specific robots
 * 
 * base, shoulder, elbow, wrist, roll, hand (float):
 *   - Joint angles in radians for direct servo control
 *   - Only used when cmd = 0 (Direct Joint Control mode)
 *   - Ignored when cmd = 1 or cmd = 2 (JSON command modes)
 *   - Values typically range based on mechanical limits of each joint
 * 
 * cmd (uint8_t):
 *   - Command processing mode selector (see detailed explanation below)
 *   - 0 = Direct joint control (uses float fields above)
 *   - 1 = JSON immediate (AVOID - blocks ESP-NOW ISR)
 *   - 2 = JSON deferred (RECOMMENDED - safe, non-blocking)
 *   - 3 = Debug/print message only
 * 
 * message[MAX_MESSAGE_LEN] (char array):
 *   - JSON command string when cmd = 1 or cmd = 2
 *   - Must be null-terminated C string
 *   - Example: "{\"T\":100}" for CMD_MOVE_INIT
 *   - Example: "{\"T\":104,\"x\":235.0,\"y\":0.0,\"z\":234.0,\"t\":0.0,\"r\":0.0,\"g\":3.14,\"spd\":0.25}"
 *   - Parsed by ArduinoJson library on receiver side
 *   - Ignored when cmd = 0
 * 
 * Usage Pattern:
 *   For movement commands: Set cmd = 2, populate message field with JSON
 *   For real-time mirroring: Set cmd = 0, populate joint angle fields
 */
typedef struct {
    uint8_t devCode;
    float base;
    float shoulder;
    float elbow;
    float wrist;
    float roll;
    float hand;
    uint8_t cmd;
    char message[MAX_MESSAGE_LEN];
} roarm_message_t;

/*******************************************************************************/
/*                     GLOBAL VARIABLES DECLARATIONS                           */
/*******************************************************************************/
/*   for variables defined in the corresponding .c file, for use in other .c   */
/*      files just by including this header file. Use extern for these.        */
/*******************************************************************************/
extern uint8_t roarm_mac[ESP_NOW_ETH_ALEN];

/*******************************************************************************/
/*                     GLOBAL FUNCTION DECLARATIONS                            */
/*******************************************************************************/
/*   for functions defined in the corresponding .c file, for use in other .c   */
/*    files just by including this header file. Extern is a default linkage    */
/*    specifier for functions, so it is not necessary to use it explicitly.    */
/*******************************************************************************/

/**
 * @brief Send joint angle command (cmd=0: direct servo control)
 *
 * @param mac Target MAC address
 * @param base Base joint angle in radians
 * @param shoulder Shoulder joint angle in radians
 * @param elbow Elbow joint angle in radians
 * @param wrist Wrist joint angle in radians
 * @param roll Roll joint angle in radians
 * @param hand Hand joint angle in radians
 * @return esp_err_t ESP_OK on success, or an error code on failure
 */
esp_err_t roarm_send_joint_angles(const uint8_t *mac, float base, float shoulder, 
                                   float elbow, float wrist, float roll, float hand);

/**
 * @brief Send JSON command (cmd=1: JSON command processing)
 *
 * @param mac Target MAC address
 * @param json_cmd JSON command string
 * @return esp_err_t ESP_OK on success, or an error code on failure
 */
esp_err_t roarm_send_json_cmd(const uint8_t *mac, const char *json_cmd);

/**
 * @brief Move to initial position
 */
void roarm_move_init(void);

/**
 * @brief Control single joint
 *
 * @param joint_id Joint ID (e.g., 1 for BASE_JOINT)
 * @param angle_rad Angle in radians
 * @param speed Movement speed
 * @param accel Acceleration
 */
void roarm_control_joint(uint8_t joint_id, float angle_rad, uint16_t speed, uint8_t accel);

/**
 * @brief Control XYZ position with orientation
 *
 * @param x X coordinate
 * @param y Y coordinate
 * @param z Z coordinate
 * @param theta Theta angle
 * @param roll Roll angle
 * @param gripper Gripper angle
 * @param speed Movement speed
 */
void roarm_move_xyz(float x, float y, float z, float theta, float roll, float gripper, float speed);

/**
 * @brief Control gripper
 *
 * @param angle_rad Gripper angle in radians
 */
void roarm_control_gripper(float angle_rad);

/**
 * @brief Control LED light
 *
 * @param brightness Brightness level (0-255)
 */
void roarm_control_light(uint8_t brightness);

/**
 * @brief Demo function: Demonstrate various movements
 *
 */
void roarm_demo(void);

#endif /* ROARM_M3_MOTOR_CONTROL_H */