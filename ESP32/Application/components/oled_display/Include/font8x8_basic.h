#ifndef _FONT8X8_BASIC_H_
#define _FONT8X8_BASIC_H_

#include <stdint.h>

// Extern declaration for the font data array
// Each character is 8 bytes (8 rows of 8 pixels)
extern const uint8_t font8x8_basic[256][8];

#endif // _FONT8X8_BASIC_H_