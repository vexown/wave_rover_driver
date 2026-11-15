/******************************************************************************
 * @file robot_status.h
 * @brief Keeps track of which controllers of the robot are alive and shows
 *        them on the web page (STATUS panel).
 *
 * Nodes report in one of three ways:
 *  - ESP-NOW nodes send a heartbeat (robot_status_msg.h) once per second.
 *  - The Raspberry Pi sends a status line over UART once per second
 *    (robot_status_handle_pi_line).
 *  - Nodes that already send other data regularly are marked as seen when
 *    that data arrives (robot_status_mark_seen).
 * A node is online when it was heard from within the last 3 s.
 ******************************************************************************/
#ifndef ROBOT_STATUS_H
#define ROBOT_STATUS_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "robot_status_msg.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Flags in the Raspberry Pi's status line. They must match STATUS_FLAG_CONTROLLER_ACTIVE and
 * STATUS_SERVICES in the Pi's UART bridge (wave_rover_pi repo). The Pi is not an ESP-NOW node,
 * so these are not in robot_status_msg.h, which is shared with the ESP-NOW nodes. */
#define ROBOT_PI_FLAG_CONTROLLER_ACTIVE (1U << 0)   // The Pi is sending Xbox controller frames
#define ROBOT_PI_FLAG_XBOXDRV           (1U << 1)   // Xbox controller driver service is running
#define ROBOT_PI_FLAG_SERIAL_NODES      (1U << 2)   // ROS serial nodes service is running
#define ROBOT_PI_FLAG_IMU_PARSER        (1U << 3)   // ROS IMU parser service is running
#define ROBOT_PI_FLAG_GPS_PARSER        (1U << 4)   // ROS GPS parser service is running

/**
 * @brief Start the task that sends the status of all nodes to the web page once per second.
 *
 * Call after web_server_init(). robot_status_mark_seen() and
 * robot_status_handle_heartbeat() may be called before this.
 *
 * @param driver_fw_version Firmware version of this board, shown in its own row.
 *                          The string must stay valid (pass a string literal).
 * @return ESP_OK, or ESP_FAIL if the task could not be created.
 */
esp_err_t robot_status_init(const char *driver_fw_version);

/**
 * @brief Record that a node which sends no heartbeat is alive.
 *
 * Call it where the node's regular data arrives.
 *
 * @param node Which node was heard from.
 */
void robot_status_mark_seen(robot_node_t node);

/**
 * @brief Handle a received ESP-NOW packet if it is a heartbeat.
 *
 * A packet is a heartbeat when it has the heartbeat's exact length and starts
 * with the two magic bytes.
 *
 * @param data Received payload.
 * @param len  Payload length in bytes.
 * @return true if the packet was a heartbeat (and was consumed),
 *         false if it is something else and should be routed as usual.
 */
bool robot_status_handle_heartbeat(const uint8_t *data, int len);

/**
 * @brief Look for the Raspberry Pi's status line in text received over UART and use it.
 *
 * The line is "$PI,<uptime_s>,<flags>*<checksum>\n": uptime in seconds and the sum of
 * ROBOT_PI_FLAG_ values, both decimal, then the XOR of all characters between '$' and '*'
 * as two hex digits. A line that is cut off or has a wrong checksum is ignored.
 *
 * @param text Zero-terminated received text. It may hold other data before and after the line.
 * @return true if a valid status line was found, false otherwise.
 */
bool robot_status_handle_pi_line(const char *text);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_STATUS_H */
