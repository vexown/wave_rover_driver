/******************************************************************************
 * @file i2c_manager.c
 * @brief Template component for ESP-IDF projects
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

/* Project Includes */
#include "i2c_manager.h"
#include "esp_log.h"

/*******************************************************************************/
/*                                  MACROS                                     */
/*******************************************************************************/
/** Tag for logging (used in ESP_LOGI, ESP_LOGE, etc.) Example usage: 
 *  ESP_LOGI(TAG, "Log message which will be appended to the tag"); */
#define TAG "I2C_MANAGER"

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
static i2c_master_bus_handle_t i2c_manager_bus_handle = NULL; // Global handle for the I2C master bus

static bool i2c_manager_initialized = false; // Flag to check if the I2C manager has been initialized

/*******************************************************************************/
/*                     GLOBAL FUNCTION DEFINITIONS                             */
/*******************************************************************************/

esp_err_t i2c_manager_init(i2c_port_t i2c_port, gpio_num_t sda_pin, gpio_num_t scl_pin)
{
    /* Check if the I2C manager is already initialized */
    if ((i2c_manager_initialized) || (i2c_manager_bus_handle != NULL))
    {
        ESP_LOGW(TAG, "I2C manager is already initialized. To reinitialize, please call i2c_manager_deinit() first.");
        return ESP_ERR_INVALID_STATE; // Return error if already initialized
    }

    ESP_LOGI(TAG, "Initializing I2C master");

    /* Define the I2C bus configuration */
    /* Make sure to set the correct SDA and SCL pins that are used in your setup */
    i2c_master_bus_config_t i2c_mst_config =
    {
        .clk_source = I2C_CLK_SRC_DEFAULT,      // Use default clock source
        .i2c_port = i2c_port,                   // Select the I2C port (we have two ports available, 0 and 1)
        .scl_io_num = scl_pin,         
        .sda_io_num = sda_pin,      
        .glitch_ignore_cnt = 7,                 // Glitch filter count (7 is a common value)
        .flags.enable_internal_pullup = true,   // Enable internal pull-up resistors (recommended for I2C lines)
    };

    /* Use the defined I2C bus configuration to allocate a new I2C master bus */
    /* The bus handle will be used for all I2C operations for any device connected to this bus */
    esp_err_t i2c_alloc_status = i2c_new_master_bus(&i2c_mst_config, &i2c_manager_bus_handle);
    if (i2c_alloc_status != ESP_OK) 
    {
        ESP_LOGE(TAG, "Allocating new I2C master bus failed: %s", esp_err_to_name(i2c_alloc_status));
    }
    else
    {
        ESP_LOGI(TAG, "I2C master bus initialized successfully on port %d", i2c_port);
        i2c_manager_initialized = true; 
    }

    return i2c_alloc_status; // Return the status of the I2C bus allocation
}

esp_err_t i2c_manager_deinit(void)
{
    /* Check if the I2C manager is initialized */
    if (!i2c_manager_initialized || i2c_manager_bus_handle == NULL)
    {
        ESP_LOGW(TAG, "I2C manager is not initialized. Nothing to deinitialize.");
        return ESP_ERR_INVALID_STATE; // Return error if not initialized
    }

    /* Deinitialize the I2C master bus and delete the bus handle */
    esp_err_t delete_status = i2c_del_master_bus(i2c_manager_bus_handle);
    if(delete_status == ESP_OK)
    {
        ESP_LOGI(TAG, "I2C master bus deleted successfully.");

        /* Reset the global bus handle and initialization flag */
        i2c_manager_bus_handle = NULL;
        i2c_manager_initialized = false;
    }
    else
    {
        ESP_LOGE(TAG, "Failed to delete I2C master bus: %s", esp_err_to_name(delete_status));
    }

    return delete_status; 
}

esp_err_t i2c_manager_get_bus_handle(i2c_master_bus_handle_t *bus_handle)
{
    esp_err_t status = ESP_FAIL;

    /* Input validation */
    if (i2c_manager_bus_handle == NULL) 
    {
        ESP_LOGE(TAG, "I2C bus handle is not initialized. Please call i2c_manager_init() first.");
        status = ESP_ERR_INVALID_STATE;
    }
    else if (bus_handle == NULL) 
    {
        ESP_LOGE(TAG, "Provided bus handle pointer is NULL.");
        status = ESP_ERR_INVALID_ARG;
    }
    else
    {
        /* Assign the global bus handle to the provided pointer */
        *bus_handle = i2c_manager_bus_handle;
        status = ESP_OK;
    }

    return status;
}


/*******************************************************************************/
/*                     STATIC FUNCTION DEFINITIONS                             */
/*******************************************************************************/
