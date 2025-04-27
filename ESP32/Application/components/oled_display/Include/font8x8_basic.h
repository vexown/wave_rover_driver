/******************************************************************************
 * @file font8x8_basic.h
 * @brief Header file for the Template component
 *
 ******************************************************************************/

#ifndef FONT8X8_BASIC_H
#define FONT8X8_BASIC_H

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
/* Font size for the characters. The font used is 8x8 pixels, which is common
 * for many small displays. This should match the font data used in the project.
 * which is defined in the .c file corresponding to this header file. */
#define FONT_WIDTH 8
#define FONT_HEIGHT 8

/*******************************************************************************/
/*                                DATA TYPES                                   */
/*******************************************************************************/

/*******************************************************************************/
/*                     GLOBAL VARIABLES DECLARATIONS                           */
/*******************************************************************************/
/*   for variables defined in the corresponding .c file, for use in other .c   */
/*      files just by including this header file. Use extern for these.        */
/*******************************************************************************/

/**
 * @brief Basic 8x8 pixel font data for the Standard ASCII character set. (0x00 to 0x7F)
 *
 * This array stores the bitmap data for a fixed-width 8x8 pixel font.
 * - The first dimension indexes the ASCII character code (0-127). Only standard ASCII characters
 *   are supported.
 * - The second dimension represents the 8 bytes defining the character bitmap.
 *
 * Pixel Mapping Explanation (Common for displays like SSD1306):
 * Each character is represented by an 8x8 grid of pixels. The array stores this
 * grid column by column.
 * - `font8x8_basic[character_code]` provides an array of 8 bytes for the specified character.
 * - Each `uint8_t` value within this array (e.g., `font8x8_basic[charCode][col]`)
 *   represents a single *vertical column* of 8 pixels, where `col` 0 is the leftmost
 *   column and `col` 7 is the rightmost column.
 * - Within each byte (column data):
 *   - Bit 0 (Least Significant Bit - LSB) corresponds to the *top* pixel in that column.
 *   - Bit 7 (Most Significant Bit - MSB) corresponds to the *bottom* pixel in that column.
 * - A bit value of '1' typically means the pixel should be turned ON (lit).
 * - A bit value of '0' typically means the pixel should be turned OFF (dark).
 *
 * Example: Character 'A' (ASCII 65) using hypothetical values for illustration
 *
 * Visual Representation (1 = ON, 0 = OFF):
 *       Column-> 0 1 2 3 4 5 6 7
 * Row 0 (LSB)    0 0 1 1 1 0 0 0   <- Col 0 bit 0, Col 1 bit 0, ...
 * Row 1          0 1 1 1 1 1 0 0   <- Col 0 bit 1, Col 1 bit 1, ...
 * Row 2          1 1 0 0 0 1 1 0   <- ...
 * Row 3          1 1 0 0 0 1 1 0
 * Row 4          1 1 1 1 1 1 1 0
 * Row 5          1 1 0 0 0 1 1 0
 * Row 6          1 1 0 0 0 1 1 0
 * Row 7 (MSB)    0 0 0 0 0 0 0 0   <- Col 0 bit 7, Col 1 bit 7, ...
 *
 * Data in the array `font8x8_basic[65]` (or `font8x8_basic['A']`) based on the above visual:
 * Each byte represents a *vertical column*, LSB is the top pixel.
 * Column 0: 0b01111100 (0x7C)  (Bits read bottom-up: 0,1,1,1,1,1,0,0)
 * Column 1: 0b01111110 (0x7E)  (Bits read bottom-up: 0,1,1,1,1,1,1,0)
 * Column 2: 0b00010011 (0x13)  (Bits read bottom-up: 0,0,0,1,0,0,1,1)
 * Column 3: 0b00010011 (0x13)  (Bits read bottom-up: 0,0,0,1,0,0,1,1)
 * Column 4: 0b00010011 (0x13)  (Bits read bottom-up: 0,0,0,1,0,0,1,1)
 * Column 5: 0b01111110 (0x7E)  (Bits read bottom-up: 0,1,1,1,1,1,1,0)
 * Column 6: 0b01111100 (0x7C)  (Bits read bottom-up: 0,1,1,1,1,1,0,0)
 * Column 7: 0b00000000 (0x00)  (Bits read bottom-up: 0,0,0,0,0,0,0,0)
 *
 * So, `font8x8_basic['A']` would contain { 0x7C, 0x7E, 0x13, 0x13, 0x13, 0x7E, 0x7C, 0x00 }
 * (Note: The actual font data defined in the corresponding .c file determines the specific appearance).
 */
extern const uint8_t font8x8_basic[128][8];

/*******************************************************************************/
/*                     GLOBAL FUNCTION DECLARATIONS                            */
/*******************************************************************************/
/*   for functions defined in the corresponding .c file, for use in other .c   */
/*    files just by including this header file. Extern is a default linkage    */
/*    specifier for functions, so it is not necessary to use it explicitly.    */
/*******************************************************************************/

#endif /* FONT8X8_BASIC_H */