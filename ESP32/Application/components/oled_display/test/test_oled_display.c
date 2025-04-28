#include <string.h>
#include "unity.h"
#include "oled_display.h" // Include the header for the component being tested
#include "font8x8_basic.h" // Include font data used by the component
#include "esp_log.h"       // For potential logging checks or context

/* [required] Add the following conditional compilation block around the oled_buffer
    definition in oled_display.c to make it accessible during tests:
#ifdef UNIT_TEST
// For unit testing, we expose the buffer to allow verification of its contents.
uint8_t oled_buffer[OLED_WIDTH * (OLED_HEIGHT / 8)];
#else
// For normal operation, keep the buffer static.
static uint8_t oled_buffer[OLED_WIDTH * (OLED_HEIGHT / 8)];
#endif
*/

// --- Copied definitions from oled_display.c ---
// These are needed because they are static or defined in the .c file
// and not easily accessible otherwise.

#define OLED_WIDTH          128
#define OLED_HEIGHT         32
#define SSD1306_PAGE_HEIGHT 8
#define SSD1306_128x32_NUM_OF_PAGES (OLED_HEIGHT / SSD1306_PAGE_HEIGHT) // 4 pages
#define SSD1306_128x32_NUM_OF_LINES (OLED_HEIGHT / FONT_HEIGHT) // 4 lines for 8x8 font
#define SSD1306_PAGE_START_INDEX(page_number) ((page_number) * OLED_WIDTH)

// Make oled_buffer accessible in tests
#ifdef UNIT_TEST
extern uint8_t oled_buffer[OLED_WIDTH * (OLED_HEIGHT / 8)];
#endif
// --- End of copied definitions ---


// Helper function to check if a specific line (page) in the buffer is cleared
static bool is_line_cleared(uint8_t line)
{
     if (line >= SSD1306_128x32_NUM_OF_LINES) {
          return false; // Invalid line
     }
     int page = line; // For 8x8 font, line number equals page number
     uint16_t start_idx = SSD1306_PAGE_START_INDEX(page);
     for (int i = 0; i < OLED_WIDTH; i++) {
          if (oled_buffer[start_idx + i] != 0) {
                return false;
          }
     }
     return true;
}

// Helper function to get expected byte value for a character column
static uint8_t get_expected_char_byte(char c, int font_col) {
     if ((uint8_t)c > 127) {
          c = '?'; // Handle unsupported characters as the original function does
     }
     if (font_col < 0 || font_col >= FONT_WIDTH) {
          return 0; // Out of bounds column
     }
     return font8x8_basic[(uint8_t)c][font_col];
}


TEST_CASE("oled_clear_buffer_clears_entire_buffer", "[oled_display]")
{
     // Arrange: Fill buffer with some data
     memset(oled_buffer, 0xFF, sizeof(oled_buffer));
     TEST_ASSERT_EQUAL_UINT8(0xFF, oled_buffer[0]); // Pre-check
     TEST_ASSERT_EQUAL_UINT8(0xFF, oled_buffer[sizeof(oled_buffer) - 1]); // Pre-check

     // Act: Clear the buffer
     oled_clear_buffer();

     // Assert: Check if all bytes are zero
     for (size_t i = 0; i < sizeof(oled_buffer); i++) {
          TEST_ASSERT_EQUAL_UINT8(0x00, oled_buffer[i]);
     }
}

TEST_CASE("oled_write_string_valid_line0", "[oled_display]")
{
     // Arrange
     const char *test_str = "Hello";
     uint8_t line = 0;
     int page = line; // For 8x8 font
     uint16_t start_idx = SSD1306_PAGE_START_INDEX(page);

     // Act
     oled_write_string(line, test_str);

     // Assert
     // Check characters written
     for (int char_idx = 0; test_str[char_idx] != '\0'; char_idx++) {
          for (int font_col = 0; font_col < FONT_WIDTH; font_col++) {
                int buffer_col = char_idx * FONT_WIDTH + font_col;
                TEST_ASSERT_EQUAL_UINT8(get_expected_char_byte(test_str[char_idx], font_col), oled_buffer[start_idx + buffer_col]);
          }
     }
     // Check rest of the line is cleared
     int written_width = strlen(test_str) * FONT_WIDTH;
     for (int buffer_col = written_width; buffer_col < OLED_WIDTH; buffer_col++) {
            TEST_ASSERT_EQUAL_UINT8(0x00, oled_buffer[start_idx + buffer_col]);
     }
     // Check other lines are untouched (should be cleared by setUp)
     TEST_ASSERT_TRUE(is_line_cleared(1));
     TEST_ASSERT_TRUE(is_line_cleared(2));
     TEST_ASSERT_TRUE(is_line_cleared(3));
}

TEST_CASE("oled_write_string_valid_last_line", "[oled_display]")
{
     // Arrange
     const char *test_str = "Line 3";
     uint8_t line = SSD1306_128x32_NUM_OF_LINES - 1; // Last valid line (line 3)
     int page = line;
     uint16_t start_idx = SSD1306_PAGE_START_INDEX(page);

     // Store the initial state of other lines before the write operation
     uint8_t initial_line0_state[OLED_WIDTH];
     uint8_t initial_line1_state[OLED_WIDTH];
     uint8_t initial_line2_state[OLED_WIDTH];
     memcpy(initial_line0_state, &oled_buffer[SSD1306_PAGE_START_INDEX(0)], OLED_WIDTH);
     memcpy(initial_line1_state, &oled_buffer[SSD1306_PAGE_START_INDEX(1)], OLED_WIDTH);
     memcpy(initial_line2_state, &oled_buffer[SSD1306_PAGE_START_INDEX(2)], OLED_WIDTH);


     // Act
     oled_write_string(line, test_str);

     // Assert
     // Check characters written to the target line (line 3)
     for (int char_idx = 0; test_str[char_idx] != '\0'; char_idx++) {
          for (int font_col = 0; font_col < FONT_WIDTH; font_col++) {
                int buffer_col = char_idx * FONT_WIDTH + font_col;
                TEST_ASSERT_EQUAL_UINT8(get_expected_char_byte(test_str[char_idx], font_col), oled_buffer[start_idx + buffer_col]);
          }
     }
      // Check rest of the target line is cleared
     int written_width = strlen(test_str) * FONT_WIDTH;
     for (int buffer_col = written_width; buffer_col < OLED_WIDTH; buffer_col++) {
            TEST_ASSERT_EQUAL_UINT8(0x00, oled_buffer[start_idx + buffer_col]);
     }
     // Check other lines are untouched by comparing with their initial state
     TEST_ASSERT_EQUAL_UINT8_ARRAY(initial_line0_state, &oled_buffer[SSD1306_PAGE_START_INDEX(0)], OLED_WIDTH);
     TEST_ASSERT_EQUAL_UINT8_ARRAY(initial_line1_state, &oled_buffer[SSD1306_PAGE_START_INDEX(1)], OLED_WIDTH);
     TEST_ASSERT_EQUAL_UINT8_ARRAY(initial_line2_state, &oled_buffer[SSD1306_PAGE_START_INDEX(2)], OLED_WIDTH);
}

TEST_CASE("oled_write_string_overwrite_line", "[oled_display]")
{
     // Arrange
     const char *str1 = "Overwrite Me";
     const char *str2 = "Done";
     uint8_t line = 1;
     int page = line;
     uint16_t start_idx = SSD1306_PAGE_START_INDEX(page);

     // Act
     oled_write_string(line, str1); // Write first string
     oled_write_string(line, str2); // Write second string to the same line

     // Assert
     // Check second string is present
     for (int char_idx = 0; str2[char_idx] != '\0'; char_idx++) {
          for (int font_col = 0; font_col < FONT_WIDTH; font_col++) {
                int buffer_col = char_idx * FONT_WIDTH + font_col;
                TEST_ASSERT_EQUAL_UINT8(get_expected_char_byte(str2[char_idx], font_col), oled_buffer[start_idx + buffer_col]);
          }
     }
     // Check rest of the line is cleared (overwritten part of str1 and beyond)
     int written_width = strlen(str2) * FONT_WIDTH;
     for (int buffer_col = written_width; buffer_col < OLED_WIDTH; buffer_col++) {
            TEST_ASSERT_EQUAL_UINT8(0x00, oled_buffer[start_idx + buffer_col]);
     }
}

TEST_CASE("oled_write_string_invalid_line", "[oled_display]")
{
     // Arrange
     const char *test_str = "Should not appear";
     uint8_t invalid_line = SSD1306_128x32_NUM_OF_LINES; // e.g., line 4
     uint8_t initial_buffer_state[sizeof(oled_buffer)];

     // Store the initial state of the entire buffer
     memcpy(initial_buffer_state, oled_buffer, sizeof(oled_buffer));

     // Act
     oled_write_string(invalid_line, test_str);

     // Assert: Buffer should remain completely unchanged
     TEST_ASSERT_EQUAL_UINT8_ARRAY(initial_buffer_state, oled_buffer, sizeof(oled_buffer));
}

TEST_CASE("oled_write_string_long_string_truncation", "[oled_display]")
{
     // Arrange: Create a string exactly one character too long
     char long_str[OLED_WIDTH / FONT_WIDTH + 2]; // Max chars + null terminator + 1 extra
     memset(long_str, 'A', sizeof(long_str) - 1);
     long_str[sizeof(long_str) - 1] = '\0';
     long_str[OLED_WIDTH / FONT_WIDTH] = 'B'; // The character that should be truncated

     uint8_t line = 0;
     int page = line;
     uint16_t start_idx = SSD1306_PAGE_START_INDEX(page);
     int max_chars = OLED_WIDTH / FONT_WIDTH;

     // Act
     oled_write_string(line, long_str);

     // Assert
     // Check that only the first 'max_chars' characters were written
     for (int char_idx = 0; char_idx < max_chars; char_idx++) {
          for (int font_col = 0; font_col < FONT_WIDTH; font_col++) {
                int buffer_col = char_idx * FONT_WIDTH + font_col;
                TEST_ASSERT_EQUAL_UINT8(get_expected_char_byte('A', font_col), oled_buffer[start_idx + buffer_col]);
          }
     }
     // The rest of the buffer for this line should be 0 (it's cleared first)
     // Note: The original code doesn't explicitly clear *after* truncation,
     // but the initial clear of the line handles this.
     // We already checked the written part, no need to check the cleared part again.
}

TEST_CASE("oled_write_string_empty_string_clears_line", "[oled_display]")
{
     // Arrange
     uint8_t line = 2;
     int page = line;
     uint16_t start_idx = SSD1306_PAGE_START_INDEX(page);
     // Pre-fill the line
     memset(&oled_buffer[start_idx], 0xAA, OLED_WIDTH);
     TEST_ASSERT_EQUAL_UINT8(0xAA, oled_buffer[start_idx]); // Pre-check

     // Act
     oled_write_string(line, "");

     // Assert: The line should now be cleared
     TEST_ASSERT_TRUE(is_line_cleared(line));
}

TEST_CASE("oled_write_string_unsupported_character", "[oled_display]")
{
     // Arrange
     // Create a string with a character outside standard ASCII (e.g., > 127)
     char test_str[] = {'H', (char)150, 'i', '\0'}; // 150 is outside 0-127
     uint8_t line = 0;
     int page = line;
     uint16_t start_idx = SSD1306_PAGE_START_INDEX(page);

     // Act
     oled_write_string(line, test_str);

     // Assert
     // Check 'H'
     for (int font_col = 0; font_col < FONT_WIDTH; font_col++) {
          int buffer_col = 0 * FONT_WIDTH + font_col;
          TEST_ASSERT_EQUAL_UINT8(get_expected_char_byte('H', font_col), oled_buffer[start_idx + buffer_col]);
     }
     // Check that character 150 was replaced with '?'
     for (int font_col = 0; font_col < FONT_WIDTH; font_col++) {
          int buffer_col = 1 * FONT_WIDTH + font_col;
          TEST_ASSERT_EQUAL_UINT8(get_expected_char_byte('?', font_col), oled_buffer[start_idx + buffer_col]);
     }
      // Check 'i'
     for (int font_col = 0; font_col < FONT_WIDTH; font_col++) {
          int buffer_col = 2 * FONT_WIDTH + font_col;
          TEST_ASSERT_EQUAL_UINT8(get_expected_char_byte('i', font_col), oled_buffer[start_idx + buffer_col]);
     }
}

// Note: Testing oled_init and oled_refresh requires mocking ESP-IDF hardware functions (I2C, LCD panel),
// which is beyond the scope of basic Unity tests and often requires more advanced techniques or frameworks (like CMock).
// The tests above focus on the buffer manipulation logic which is testable in isolation.
