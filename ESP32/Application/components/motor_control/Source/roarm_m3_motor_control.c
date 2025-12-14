/******************************************************************************
 * @file roarm_m3_motor_control.c
 * @brief RoArm-M3 Motor Control component for ESP-IDF projects
 * 
 ******************************************************************************/

/*******************************************************************************/
/*                                 INCLUDES                                    */
/*******************************************************************************/
/*    Include headers required for the definitions/implementation in *this*    */
/* source file. This typically includes this module's own header("template.h") */
/*      and any headers needed for function bodies, static variables, etc.     */
/*******************************************************************************/

#include "roarm_m3_motor_control.h"

/*******************************************************************************/
/*                                  MACROS                                     */
/*******************************************************************************/

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

/* Target RoArm-M3 MAC address */
uint8_t roarm_mac[ESP_NOW_ETH_ALEN] = {0xF0, 0x24, 0xF9, 0x10, 0xF5, 0x9C};

/*******************************************************************************/
/*                     STATIC FUNCTION DECLARATIONS                            */
/*******************************************************************************/

/*******************************************************************************/
/*                             STATIC VARIABLES                                */
/*******************************************************************************/

static const char *TAG = "ROARM_CTRL";

/*******************************************************************************/
/*                     GLOBAL FUNCTION DEFINITIONS                             */
/*******************************************************************************/

esp_err_t roarm_send_joint_angles(const uint8_t *mac, float base, float shoulder, 
                                   float elbow, float wrist, float roll, float hand)
{
    roarm_message_t msg = {
        .devCode = 0,
        .base = base,
        .shoulder = shoulder,
        .elbow = elbow,
        .wrist = wrist,
        .roll = roll,
        .hand = hand,
        .cmd = 0,  /* Direct servo control */
        .message = {0}
    };
    
    return esp_now_send((uint8_t *)mac, (uint8_t *)&msg, sizeof(roarm_message_t));
}

esp_err_t roarm_send_json_cmd(const uint8_t *mac, const char *json_cmd)
{
    roarm_message_t msg = {
        .devCode = 0,   // ||
        .base = 0,      // ||
        .shoulder = 0,  // ||
        .elbow = 0,     // | ===> UNUSED FIELDS WHEN .cmd = 1 or .cmd = 2
        .wrist = 0,     // ||
        .roll = 0,      // ||
        .hand = 0,      // ||
        .cmd = 2,       // JSON deferred processing (RECOMMENDED - safe, non-blocking)
    };
    
    strncpy(msg.message, json_cmd, MAX_MESSAGE_LEN - 1);
    msg.message[MAX_MESSAGE_LEN - 1] = '\0';
    
    vTaskDelay(pdMS_TO_TICKS(20));
    return esp_now_send((uint8_t *)mac, (uint8_t *)&msg, sizeof(roarm_message_t));
}

void roarm_move_init(void)
{
    const char *json = "{\"T\":100}";
    roarm_send_json_cmd(roarm_mac, json);
}

void roarm_control_joint(uint8_t joint_id, float angle_rad, uint16_t speed, uint8_t accel)
{
    char json[128];
    snprintf(json, sizeof(json), 
             "{\"T\":101,\"joint\":%d,\"rad\":%.2f,\"spd\":%d,\"acc\":%d}",
             joint_id, angle_rad, speed, accel);
    roarm_send_json_cmd(roarm_mac, json);
}

void roarm_move_xyz(float x, float y, float z, float theta, float roll, float gripper, float speed)
{
    char json[200];
    snprintf(json, sizeof(json),
             "{\"T\":104,\"x\":%.2f,\"y\":%.2f,\"z\":%.2f,\"t\":%.2f,\"r\":%.2f,\"g\":%.2f,\"spd\":%.2f}",
             x, y, z, theta, roll, gripper, speed);
    roarm_send_json_cmd(roarm_mac, json);
}

void roarm_control_gripper(float angle_rad)
{
    char json[128];
    snprintf(json, sizeof(json),
             "{\"T\":106,\"cmd\":%.2f,\"spd\":0,\"acc\":0}",
             angle_rad);
    roarm_send_json_cmd(roarm_mac, json);
}

void roarm_control_light(uint8_t brightness)
{
    char json[64];
    snprintf(json, sizeof(json), "{\"T\":114,\"led\":%d}", brightness);
    roarm_send_json_cmd(roarm_mac, json);
}

void roarm_demo(void)
{
    vTaskDelay(pdMS_TO_TICKS(2000));  /* Wait for initialization */
    
    ESP_LOGI(TAG, "Starting RoArm-M3 demo sequence");
    
    /* 1. Move to initial position */
    ESP_LOGI(TAG, "Moving to init position");
    roarm_move_init();
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    /* 2. Move to specific XYZ coordinates */
    ESP_LOGI(TAG, "Moving to target position");
    roarm_move_xyz(235.0, 0.0, 234.0, 0.0, 0.0, 3.14, 0.25);
    vTaskDelay(pdMS_TO_TICKS(4000));
    
    /* 3. Control individual joint (BASE_JOINT) */
    ESP_LOGI(TAG, "Rotating base joint");
    roarm_control_joint(1, 0.785, 10, 10);  /* 45 degrees */
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    /* 4. Close gripper */
    ESP_LOGI(TAG, "Closing gripper");
    roarm_control_gripper(3.14);  /* Closed position */
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    /* 5. Open gripper */
    ESP_LOGI(TAG, "Opening gripper");
    roarm_control_gripper(1.57);  /* Open position */
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    /* 6. Turn on LED */
    ESP_LOGI(TAG, "Turning on LED");
    roarm_control_light(255);
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    /* 7. Turn off LED */
    ESP_LOGI(TAG, "Turning off LED");
    roarm_control_light(0);
    
    ESP_LOGI(TAG, "Demo sequence complete");
}

/*******************************************************************************/
/*                     STATIC FUNCTION DEFINITIONS                             */
/*******************************************************************************/