/******************************************************************************
 * @file NaviLogging.c
 * @brief Component for receiving navigation data via ESP-NOW.
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
#include "freertos/semphr.h"
#include "esp_log.h"
#include "string.h"
#include "web_server.h"
#include "Common.h"

/* Project Includes */
#include "NaviLogging.h"
#include "esp_now_comm.h"
/*******************************************************************************/
/*                                  MACROS                                     */
/*******************************************************************************/
/** Tag for logging (used in ESP_LOGI, ESP_LOGE, etc.) Example usage: 
 *  ESP_LOGI(TAG, "Log message which will be appended to the tag"); */
#define TAG "NAVILOGGING"

#define NAVILOGGING_RECEIVE_TIMEOUT_MS 10000 // Timeout for new data (10 seconds)

/* Task configuration for the NaviLogging component */
#define NAVILOGGING_TASK_STACK_SIZE 4096
#define NAVILOGGING_TASK_PRIORITY (tskIDLE_PRIORITY + 2)
#define NAVILOGGING_TASK_PERIOD_TICKS pdMS_TO_TICKS(1000)

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

/* MAC address of the ESP-NOW sender device (Navigation App ESP32) */
uint8_t navi_esp32_mac[ESP_NOW_ETH_ALEN] = {0xC8, 0xF0, 0x9E, 0xA2, 0x47, 0x1C};

/*******************************************************************************/
/*                     STATIC FUNCTION DECLARATIONS                            */
/*******************************************************************************/

/**
 * @brief Task for processing received navigation coordinates.
 * 
 * Check for new data and process it.
 * Currently it just retrieves the last received coordinates and logs them.
 * 
 * @param xTaskParameter Pointer to task parameters (not used in this case).
 */
static void navi_coordinates_processing_task(void *xTaskParameter);

/*******************************************************************************/
/*                             STATIC VARIABLES                                */
/*******************************************************************************/
/* Variable for storing received coordinates */
static navi_coordinates_type last_coordinates;

/* Flag to signal if new data is available */
static bool new_data_available = false;

/* Mutex for protecting access to last_coordinates and new_data_available */
static SemaphoreHandle_t coords_access_mutex = NULL;

/*******************************************************************************/
/*                     GLOBAL FUNCTION DEFINITIONS                             */
/*******************************************************************************/

esp_err_t NaviLogging_init(void)
{
    /* Create mutex for critical sections */
    coords_access_mutex = xSemaphoreCreateMutex();
    if (coords_access_mutex == NULL)
    {
        LOG_TO_RPI("NaviLogging: Failed to create mutex for coordinates access");
        return ESP_FAIL;
    }

    /* Add the sender as an ESP-NOW peer */
    esp_err_t ret = esp_now_comm_add_peer(navi_esp32_mac);
    if (ret != ESP_OK)
    {
        LOG_TO_RPI("NaviLogging: Failed to add ESP-NOW peer: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Start the coordinates processing task */
    BaseType_t status_task = xTaskCreate(
        navi_coordinates_processing_task,
        "navi_coordinates_processing_task",
        NAVILOGGING_TASK_STACK_SIZE,
        NULL,
        NAVILOGGING_TASK_PRIORITY,
        NULL
    );
    if (status_task != pdPASS)
    {
        LOG_TO_RPI("NaviLogging: Failed to create navi_coordinates_processing_task");
        return ESP_FAIL;
    }

    LOG_TO_RPI("NaviLogging: Initialized successfully. Waiting for data...");
    return ESP_OK;
}

esp_err_t NaviLogging_get_last_coordinates(navi_coordinates_type *coordinates)
{
    /* Validate input pointer */
    if (coordinates == NULL)
    {
        LOG_TO_RPI("NaviLogging Error: NULL pointer provided to NaviLogging_get_last_coordinates");
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(coords_access_mutex, portMAX_DELAY) == pdTRUE)
    {
        /* Copy the last received coordinates to the provided pointer */
        *coordinates = last_coordinates;

        /* Mark the coordinates as retrieved (not new anymore) */
        new_data_available = false;
        
        /* Give back the mutex to allow other tasks to access the coordinates */
        xSemaphoreGive(coords_access_mutex);
        
        return ESP_OK;
    }
    else
    {
        LOG_TO_RPI("NaviLogging Error: Failed to take mutex in NaviLogging_get_last_coordinates");
        return ESP_FAIL; // Or handle error appropriately
    }
}

bool NaviLogging_is_new_data_available(void)
{
    bool status;
    if (xSemaphoreTake(coords_access_mutex, portMAX_DELAY) == pdTRUE)
    {
        status = new_data_available;
        xSemaphoreGive(coords_access_mutex);
    }
    else
    {
        // Log error or handle mutex take failure
        ESP_LOGE(TAG, "Failed to take mutex in NaviLogging_is_new_data_available");
        status = false; // Default to false in case of error
    }
    return status;
}

void NaviLogging_handle_received_coords(const uint8_t *mac_addr, const uint8_t *data, int len)
{
    /* Validate input parameters */
    if (mac_addr == NULL || data == NULL)
    {
        LOG_TO_RPI("NaviLogging Warning: Received NULL mac_addr or data pointer");
        return;
    }

    /* Check if the received data length matches the expected size. If it doesn't, we might be receving corrupted data or simply not the data we expect */
    if (len != sizeof(navi_coordinates_type))
    {
        LOG_TO_RPI("NaviLogging Warning: Received data length (%d) does not match expected size (%zu)",
                   len, sizeof(navi_coordinates_type));
        return;
    }

    /* Attempt to update coordinates with non-blocking mutex */
    if (xSemaphoreTake(coords_access_mutex, (TickType_t)0) == pdTRUE)
    {
        /* Deserialize the received data into the last_coordinates structure. This can be later retrieved by the user via NaviLogging_get_last_coordinates() */
        memcpy(&last_coordinates, data, sizeof(navi_coordinates_type));
        
        /* Indicate that new data is available. This flag can be checked by the user via NaviLogging_is_new_data_available() 
           to avoid wasting time on retrieving data that is not new */
        new_data_available = true;

        xSemaphoreGive(coords_access_mutex);
    }
    else
    {
        /* Failed to take mutex, data might be lost or delayed. Log this. */
        LOG_TO_RPI("NaviLogging Error: Failed to take mutex in esp_now_receive_callback, data might be lost or delayed");
    }
}

/*******************************************************************************/
/*                     STATIC FUNCTION DEFINITIONS                             */
/*******************************************************************************/

static void navi_coordinates_processing_task(void *xTaskParameter)
{
    /******************** Task Initialization ********************/


    /************************* Task Loop *************************/
    while (1) 
    {
        /* Check if new data is available */
        if (NaviLogging_is_new_data_available()) 
        {
            /* Log the last coordinates */
            /* We could get the coordinates directly from the last_coordinates variable, but just for the 
               sake of consistency, we will use the NaviLogging_get_last_coordinates() function */
            navi_coordinates_type coordinates;
            esp_err_t err = NaviLogging_get_last_coordinates(&coordinates);
            if (err == ESP_OK) 
            {
                LOG_TO_RPI("NaviLogging Task: New coordinates received - lat:%.7f,lon:%.7f,alt:%.2f,fix:%d,hdop:%.1f,sats:%d\n",
                       coordinates.latitude, coordinates.longitude, coordinates.altitude, 
                       coordinates.quality_indicator, coordinates.horizontal_dilution_of_precision, coordinates.number_of_satellites);
            } 
            else 
            {
                LOG_TO_RPI("NaviLogging Task: Failed to get new coordinates - this should never happen (only error condition is if the pointer is NULL)");
            }
        } 

        /* Delay for a while before checking again */
        vTaskDelay(NAVILOGGING_TASK_PERIOD_TICKS);
    }
}
