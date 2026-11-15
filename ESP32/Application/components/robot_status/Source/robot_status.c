/******************************************************************************
 * @file robot_status.c
 * @brief Tracks which controllers of the robot are alive and pushes their
 *        status to the web page once per second. See robot_status.h.
 ******************************************************************************/

/*******************************************************************************/
/*                                INCLUDES                                     */
/*******************************************************************************/
#include "robot_status.h"

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "web_server.h"

/*******************************************************************************/
/*                                  MACROS                                     */
/*******************************************************************************/
#define TAG "ROBOT_STATUS"

#define ROBOT_STATUS_OFFLINE_AFTER_MS   (3000)  // A node is offline when not heard from for this long (3 missed heartbeats)
#define ROBOT_STATUS_TASK_STACK_SIZE    (4096)
#define ROBOT_STATUS_TASK_PRIORITY      (tskIDLE_PRIORITY + 2)
#define ROBOT_STATUS_JSON_MAX_LEN       (1024)
#define ROBOT_STATUS_INFO_MAX_LEN       (96)

/*******************************************************************************/
/*                                DATA TYPES                                   */
/*******************************************************************************/
typedef struct
{
    const char *name;               // Shown on the web page
    const char *seen_info;          // Shown for nodes without a heartbeat: what marks them as seen
    bool seen;                      // Heard from at least once since boot
    int64_t last_seen_us;           // esp_timer time when last heard from
    bool has_heartbeat;             // heartbeat below is valid
    robot_status_msg_t heartbeat;   // Last heartbeat received
} robot_node_entry_t;

/* Name of one flag bit, for showing flags as text */
typedef struct
{
    uint16_t bit;
    const char *name;
} flag_name_t;

/*******************************************************************************/
/*                             STATIC VARIABLES                                */
/*******************************************************************************/
static robot_node_entry_t nodes[ROBOT_NODE_COUNT] =
{
    [ROBOT_NODE_DRIVER] = { .name = "Driver (this board)" },
    [ROBOT_NODE_ARM]    = { .name = "RoArm-M3" },
    [ROBOT_NODE_MOTOR]  = { .name = "Motor controller" },
    [ROBOT_NODE_GPS]    = { .name = "GPS / navigation", .seen_info = "coordinates received over ESP-NOW" },
    [ROBOT_NODE_PI]     = { .name = "Raspberry Pi",     .seen_info = "Xbox frames received over UART" },
};

static const flag_name_t arm_flag_names[] =
{
    { ROBOT_ARM_FLAG_INIT_OK,            "init OK" },
    { ROBOT_ARM_FLAG_CALIBRATION_FAILED, "CALIBRATION FAILED" },
    { ROBOT_ARM_FLAG_JOGGING,            "jogging" },
    { ROBOT_ARM_FLAG_TIMEOUT_STOP,       "timeout stop occurred" },
    { 0, NULL }
};

static const flag_name_t motor_flag_names[] =
{
    { ROBOT_MOTOR_FLAG_MOVING, "moving" },
    { 0, NULL }
};

/* Protects nodes[]: written from the WiFi task (ESP-NOW receive) and motor_task, read by the status task */
static portMUX_TYPE nodes_lock = portMUX_INITIALIZER_UNLOCKED;

static const char *driver_fw = "?";

/*******************************************************************************/
/*                     STATIC FUNCTION DECLARATIONS                            */
/*******************************************************************************/
static void robot_status_task(void *pvParameters);
static int build_status_json(char *json, size_t json_size);
static void build_heartbeat_info(const robot_status_msg_t *heartbeat, char *info, size_t info_size);
static void copy_json_safe(char *dst, size_t dst_size, const char *src, size_t src_max_len);

/*******************************************************************************/
/*                     GLOBAL FUNCTION DEFINITIONS                             */
/*******************************************************************************/
esp_err_t robot_status_init(const char *driver_fw_version)
{
    if (driver_fw_version != NULL)
    {
        driver_fw = driver_fw_version;
    }

    BaseType_t task_created = xTaskCreate(robot_status_task, "robot_status", ROBOT_STATUS_TASK_STACK_SIZE,
                                          NULL, ROBOT_STATUS_TASK_PRIORITY, NULL);
    if (task_created != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create robot status task");
        return ESP_FAIL;
    }

    return ESP_OK;
}

void robot_status_mark_seen(robot_node_t node)
{
    if (node >= ROBOT_NODE_COUNT)
    {
        return;
    }

    portENTER_CRITICAL(&nodes_lock);
    nodes[node].seen = true;
    nodes[node].last_seen_us = esp_timer_get_time();
    portEXIT_CRITICAL(&nodes_lock);
}

bool robot_status_handle_heartbeat(const uint8_t *data, int len)
{
    /* A heartbeat has both the exact length and the two magic bytes. Anything else is left to
     * the normal routing: other packets have no header, so their first two bytes can match by chance. */
    robot_status_msg_t heartbeat;
    if (data == NULL || len != (int)sizeof(heartbeat) ||
        data[0] != ROBOT_STATUS_MAGIC_0 || data[1] != ROBOT_STATUS_MAGIC_1)
    {
        return false;
    }

    /* It is a heartbeat, so it is consumed here even if it can't be used */
    memcpy(&heartbeat, data, sizeof(heartbeat));

    if (heartbeat.proto_version != ROBOT_STATUS_PROTO_VERSION)
    {
        ESP_LOGW(TAG, "Heartbeat from node %d has protocol version %d (expected %d), ignored",
                 heartbeat.node_id, heartbeat.proto_version, ROBOT_STATUS_PROTO_VERSION);
        return true;
    }
    if (heartbeat.node_id == ROBOT_NODE_DRIVER || heartbeat.node_id >= ROBOT_NODE_COUNT)
    {
        ESP_LOGW(TAG, "Heartbeat with unknown node id %d ignored", heartbeat.node_id);
        return true;
    }

    portENTER_CRITICAL(&nodes_lock);
    nodes[heartbeat.node_id].seen = true;
    nodes[heartbeat.node_id].last_seen_us = esp_timer_get_time();
    nodes[heartbeat.node_id].has_heartbeat = true;
    nodes[heartbeat.node_id].heartbeat = heartbeat;
    portEXIT_CRITICAL(&nodes_lock);

    return true;
}

/*******************************************************************************/
/*                     STATIC FUNCTION DEFINITIONS                             */
/*******************************************************************************/
static void robot_status_task(void *pvParameters)
{
    static char json[ROBOT_STATUS_JSON_MAX_LEN];

    while (1)
    {
        /* This board is alive by definition: it is the one reporting */
        robot_status_mark_seen(ROBOT_NODE_DRIVER);

        if (build_status_json(json, sizeof(json)) > 0)
        {
            web_server_ws_broadcast_text(json);
        }

        vTaskDelay(pdMS_TO_TICKS(ROBOT_STATUS_PERIOD_MS));
    }
}

/**
 * @brief Build the status of all nodes as JSON for the web page.
 *
 * Format: {"type":"status","nodes":[{"name":"..","state":"online|offline|never",
 *          "age":seconds since last heard,"fw":"..","uptime":seconds,"info":".."}, ...]}
 * "fw" is "" and "uptime" is -1 for nodes that send no heartbeat.
 *
 * @return Length of the JSON, or 0 if it did not fit in the buffer.
 */
static int build_status_json(char *json, size_t json_size)
{
    /* Work on a copy so the lock is held only for the copy */
    robot_node_entry_t snapshot[ROBOT_NODE_COUNT];
    portENTER_CRITICAL(&nodes_lock);
    memcpy(snapshot, nodes, sizeof(snapshot));
    portEXIT_CRITICAL(&nodes_lock);

    int64_t now_us = esp_timer_get_time();
    int used = snprintf(json, json_size, "{\"type\":\"status\",\"nodes\":[");

    for (int i = 0; i < ROBOT_NODE_COUNT; i++)
    {
        const robot_node_entry_t *node = &snapshot[i];
        int age_s = (int)((now_us - node->last_seen_us) / 1000000);
        const char *state = "never";
        if (node->seen)
        {
            state = ((now_us - node->last_seen_us) / 1000 <= ROBOT_STATUS_OFFLINE_AFTER_MS) ? "online" : "offline";
        }

        char fw[ROBOT_STATUS_FW_VERSION_LEN + 1] = "";
        char info[ROBOT_STATUS_INFO_MAX_LEN] = "";
        long uptime_s = -1;

        if (i == ROBOT_NODE_DRIVER)
        {
            copy_json_safe(fw, sizeof(fw), driver_fw, ROBOT_STATUS_FW_VERSION_LEN);
            uptime_s = (long)(now_us / 1000000);

            /* WiFi signal strength. The call fails when not connected to an access point.
             * snprintf results discarded here and below: these short texts always fit in info. */
            wifi_ap_record_t ap_info;
            if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK)
            {
                (void)snprintf(info, sizeof(info), "WiFi signal %d dBm", ap_info.rssi);
            }
            else
            {
                (void)snprintf(info, sizeof(info), "WiFi not connected");
            }
        }
        else if (node->has_heartbeat)
        {
            copy_json_safe(fw, sizeof(fw), node->heartbeat.fw_version, ROBOT_STATUS_FW_VERSION_LEN);
            uptime_s = (long)node->heartbeat.uptime_s;
            build_heartbeat_info(&node->heartbeat, info, sizeof(info));
        }
        else if (node->seen && node->seen_info != NULL)
        {
            (void)snprintf(info, sizeof(info), "%s", node->seen_info);
        }

        if (used < (int)json_size)
        {
            used += snprintf(json + used, json_size - used,
                             "%s{\"name\":\"%s\",\"state\":\"%s\",\"age\":%d,\"fw\":\"%s\",\"uptime\":%ld,\"info\":\"%s\"}",
                             (i == 0) ? "" : ",", node->name, state, node->seen ? age_s : -1, fw, uptime_s, info);
        }
    }

    if (used < (int)json_size)
    {
        used += snprintf(json + used, json_size - used, "]}");
    }

    if (used >= (int)json_size)
    {
        ESP_LOGE(TAG, "Status JSON does not fit in %d bytes", (int)json_size);
        return 0;
    }
    return used;
}

/**
 * @brief Turn a heartbeat's flags and last error into text for the web page.
 */
static void build_heartbeat_info(const robot_status_msg_t *heartbeat, char *info, size_t info_size)
{
    const flag_name_t *names = NULL;
    if (heartbeat->node_id == ROBOT_NODE_ARM)   names = arm_flag_names;
    if (heartbeat->node_id == ROBOT_NODE_MOTOR) names = motor_flag_names;

    int used = 0;
    info[0] = '\0';

    for (int i = 0; names != NULL && names[i].name != NULL; i++)
    {
        if ((heartbeat->flags & names[i].bit) && used < (int)info_size)
        {
            used += snprintf(info + used, info_size - used, "%s%s", (used == 0) ? "" : ", ", names[i].name);
        }
    }

    if (heartbeat->last_error != 0 && used < (int)info_size)
    {
        /* Result not needed: a truncated text is acceptable here, info is always terminated */
        (void)snprintf(info + used, info_size - used, "%slast error %u", (used == 0) ? "" : ", ", heartbeat->last_error);
    }
}

/**
 * @brief Copy text that goes inside a JSON string, keeping only characters that are safe there.
 *
 * The firmware version of a heartbeat comes from another device, so it is not trusted to be
 * terminated or printable.
 */
static void copy_json_safe(char *dst, size_t dst_size, const char *src, size_t src_max_len)
{
    size_t out = 0;
    for (size_t i = 0; i < src_max_len && src[i] != '\0' && out + 1 < dst_size; i++)
    {
        char c = src[i];
        if (c >= ' ' && c <= '~' && c != '"' && c != '\\')
        {
            dst[out++] = c;
        }
    }
    dst[out] = '\0';
}
