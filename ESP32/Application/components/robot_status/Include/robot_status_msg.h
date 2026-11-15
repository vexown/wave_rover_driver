/******************************************************************************
 * @file robot_status_msg.h
 * @brief Heartbeat message that every controller of the robot sends to the
 *        wave_rover_driver board over ESP-NOW, once per second.
 *
 * This file is shared: each repo that sends or receives the heartbeat keeps
 * its own copy. The copies must be identical. Any change to the layout
 * needs a new ROBOT_STATUS_PROTO_VERSION.
 ******************************************************************************/
#ifndef ROBOT_STATUS_MSG_H
#define ROBOT_STATUS_MSG_H

#include <stdint.h>

#define ROBOT_STATUS_MAGIC_0        'R'     /* First two bytes of every heartbeat: tell it apart from any other packet */
#define ROBOT_STATUS_MAGIC_1        'S'
#define ROBOT_STATUS_PROTO_VERSION  1
#define ROBOT_STATUS_PERIOD_MS      1000    /* How often a node sends its heartbeat */
#define ROBOT_STATUS_FW_VERSION_LEN 12

/* Who a heartbeat is from (node_id) */
typedef enum
{
    ROBOT_NODE_DRIVER = 0,  /* wave_rover_driver board itself (never sent, used on the driver only) */
    ROBOT_NODE_ARM,         /* RoArm-M3 ESP32 */
    ROBOT_NODE_MOTOR,       /* Motor controller ESP32 */
    ROBOT_NODE_GPS,         /* GPS / navigation ESP32 */
    ROBOT_NODE_PI,          /* Raspberry Pi */
    ROBOT_NODE_COUNT
} robot_node_t;

/* Meaning of the flags, per node */
#define ROBOT_ARM_FLAG_INIT_OK              (1u << 0)   /* Init sequence (incl. shoulder calibration) completed */
#define ROBOT_ARM_FLAG_CALIBRATION_FAILED   (1u << 1)   /* Shoulder driven servo calibration failed */
#define ROBOT_ARM_FLAG_JOGGING              (1u << 2)   /* At least one axis is moving under constant control */
#define ROBOT_ARM_FLAG_TIMEOUT_STOP         (1u << 3)   /* An axis was stopped by the command timeout since boot */

#define ROBOT_MOTOR_FLAG_MOVING             (1u << 0)   /* Motors are being driven */

typedef struct __attribute__((packed))
{
    uint8_t  magic[2];                                  /* ROBOT_STATUS_MAGIC_0, ROBOT_STATUS_MAGIC_1 */
    uint8_t  proto_version;                             /* ROBOT_STATUS_PROTO_VERSION */
    uint8_t  node_id;                                   /* robot_node_t */
    uint32_t uptime_s;                                  /* Seconds since boot (a drop means the node rebooted) */
    uint16_t flags;                                     /* ROBOT_<NODE>_FLAG_* bits */
    uint16_t last_error;                                /* Node-specific code of the most recent problem, 0 = none */
    char     fw_version[ROBOT_STATUS_FW_VERSION_LEN];   /* Zero-terminated, e.g. "1.0.7" */
} robot_status_msg_t;                                   /* 24 bytes */

/* Sender and receiver must agree on the size: catches a layout change at build time */
_Static_assert(sizeof(robot_status_msg_t) == 24, "robot_status_msg_t must be 24 bytes");

#endif /* ROBOT_STATUS_MSG_H */
