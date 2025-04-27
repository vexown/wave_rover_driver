/******************************************************************************
 * @file oled_display.h
 * @brief Header file for the Template component
 *
 ******************************************************************************/

#ifndef OLED_DISPLAY_H
#define OLED_DISPLAY_H

/*******************************************************************************/
/*                                 INCLUDES                                    */
/*******************************************************************************/
/*     Include headers required for the declarations in *this* header file     */
/*                 (e.g., types used in function prototypes).                  */
/*       Prefer forward declarations over full includes where possible         */
/*             to minimize dependencies (for structs, enums etc.).             */
/*******************************************************************************/

/* C Standard Libraries */
#include <stdint.h>
#include <stdbool.h> 

/* ESP-IDF Libraries */
#include "esp_err.h"

/*******************************************************************************/
/*                                  MACROS                                     */
/*******************************************************************************/

/*******************************************************************************/
/*                                DATA TYPES                                   */
/*******************************************************************************/

/*******************************************************************************/
/*                     GLOBAL VARIABLES DECLARATIONS                           */
/*******************************************************************************/
/*   for variables defined in the corresponding .c file, for use in other .c   */
/*      files just by including this header file. Use extern for these.        */
/*******************************************************************************/

/*******************************************************************************/
/*                     GLOBAL FUNCTION DECLARATIONS                            */
/*******************************************************************************/
/*   for functions defined in the corresponding .c file, for use in other .c   */
/*    files just by including this header file. Extern is a default linkage    */
/*    specifier for functions, so it is not necessary to use it explicitly.    */
/*******************************************************************************/

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
 * Assumes a 4-line display (128x32) with 8-pixel high characters (we use 8x8 font - font8x8_basic).
 * 
 * The SSD1306 128x32 display is divided into 4 pages (0-3), each 8 pixels high, which in the particular case
 * of 8x8 font means 4 lines of text. We could potentialy use a bigger font (e.g. 16x16) or a smaller one (e.g. 5x7)
 * but this is a good compromise between readability and space. Right now the number of lines corresponds to the number of pages
 * but it wouldn't necessarily be the case for other fonts.
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

#endif /* OLED_DISPLAY_H */