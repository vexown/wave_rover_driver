#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include "esp_err.h"

/**
 * @brief Initialize WiFi in SoftAP mode and start the HTTP web server.
 *
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t web_server_init(void);

#endif // WEB_SERVER_H