/******************************************************************************
 * @file robot_status.h
 * @brief Keeps track of which controllers of the robot are alive and shows
 *        them on the web page (STATUS panel).
 *
 * Nodes report in one of two ways:
 *  - ESP-NOW nodes send a heartbeat (robot_status_msg.h) once per second.
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

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_STATUS_H */
