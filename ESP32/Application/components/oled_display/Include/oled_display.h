#ifndef OLED_DISPLAY_H
#define OLED_DISPLAY_H

#include "esp_err.h"

/**
 * @brief Initialize the I2C bus and the SSD1306 OLED display.
 *
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t oled_init(void);

/**
 * @brief Clear the OLED display buffer.
 *
 * Call oled_refresh() afterwards to update the physical display.
 */
void oled_clear_buffer(void);

/**
 * @brief Write a string to a specific line on the OLED display buffer.
 *
 * Assumes a 4-line display (128x32) with 8-pixel high characters.
 *
 * @param line Line number (0-3).
 * @param text The null-terminated string to write.
 */
void oled_write_string(uint8_t line, const char *text);

/**
 * @brief Refresh the physical OLED display with the content of the buffer.
 *
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t oled_refresh(void);

#endif // OLED_DISPLAY_H