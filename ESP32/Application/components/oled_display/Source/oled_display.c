/******************************************************************************
 * @file oled_display.c
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
/* C Standard Libraries */
#include <string.h>
#include <assert.h>

/* ESP-IDF Includes */
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"
#include "esp_check.h"

/* Project Includes */
#include "oled_display.h"
#include "font8x8_basic.h" 
#include "i2c_manager.h"

/*******************************************************************************/
/*                                  MACROS                                     */
/*******************************************************************************/
/** Tag for logging (used in ESP_LOGI, ESP_LOGE, etc.) Example usage: 
 *  ESP_LOGI(TAG, "Log message which will be appended to the tag"); */
#define TAG "OLED_DISPLAY"

/* I2C Configuration of the SSD1306 OLED display with the ESP32 */
/* For SDA and SCL GPIO number confirmation see the schematics of the Waveshare 
 * General Driver for Robots which is the board we are using
 * https://files.waveshare.com/upload/3/37/General_Driver_for_Robots.pdf 
 **/
#define OLED_I2C_PORT       I2C_MANAGER_DEFAULT_PORT
#define OLED_I2C_SDA_PIN    I2C_MANAGER_DEFAULT_SDA
#define OLED_I2C_SCL_PIN    I2C_MANAGER_DEFAULT_SCL 

/* I2C address of the SSD1306 OLED display. The address is typically 0x3C for most
 * SSD1306 displays. This can be confirmed in the datasheet of the display or by
 * using an I2C scanner tool. */
#define OLED_I2C_ADDR       0x3C

/* Size of the OLED display in pixels. The width and height should match the
 * actual dimensions of the display being used. In this case, we are using
 * a 128x32 pixel SSD1306-based OLED display (some kind of Waveshare module). */
#define OLED_WIDTH          128
#define OLED_HEIGHT         32

/* The SSD1306 display is organized in pages, each page is 8 pixels high.
 * The number of pages for a 32 pixel high display is 4 (32 / 8 = 4). */
#define SSD1306_PAGE_HEIGHT 8

/* Number of pages for a 128x32 SSD1306 display (4 pages) */
#define SSD1306_128x32_NUM_OF_PAGES (OLED_HEIGHT / SSD1306_PAGE_HEIGHT)

/* Number of lines for a 128x32 SSD1306 display based on the font height */
/* Note: For 8x8 font, the number of lines is equal to the number of pages 
 * since the height of each page is 8 pixels and the font height is also 8 pixels.
 * However, if a different font is used, this may not be the case. */
#define SSD1306_128x32_NUM_OF_LINES (OLED_HEIGHT / FONT_HEIGHT)

/* Macro to calculate the starting byte index in the oled_buffer for a given page number */
/* Note - pages are counted from 0 e.g Page 0, Page 1, Page 2 and so on.
 * This assumes the OLED buffer is a linear uint8 array where each page is stored sequentially.
 * For example, for a 128x32 display, the first page (page 0) starts at index 0,
 * the second page (page 1) starts at index 128, and so on. */
#define SSD1306_PAGE_START_INDEX(page_number) ((page_number) * OLED_WIDTH) // 0 for page 0, 128 for page 1, etc.

/* Pixel clock frequency for the I2C communication with the OLED display.
 * This is typically set to 400KHz for fast I2C mode. */
#define OLED_PIXEL_CLOCK_HZ (400 * 1000)

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
/**
 * @brief Draws a single character onto the internal OLED buffer at the specified coordinates.
 *
 * This function takes a character and its top-left starting coordinates (x, y)
 * and renders it into the `oled_buffer` using the `font8x8_basic` font data.
 * It handles the necessary transposition and mirroring to display correctly
 * on the SSD1306 when the panel is configured with mirroring (true, true).
 * Coordinates outside the buffer boundaries are clipped.
 *
 * @param x The horizontal starting position (column) for the character's top-left corner.
 * @param y The vertical starting position (row) for the character's top-left corner.
 * @param char_to_draw The character to draw.
 */
static void oled_draw_char(int x, int y, char char_to_draw);

/**
 * @brief Cleans up resources allocated during OLED initialization, especially after a failure.
 *
 * This function is typically called internally after an initialization attempt
 * to release any partially allocated resources if the initialization failed.
 * It logs the final status.
 *
 * @param log_ret The return code from the initialization process that triggered the cleanup.
 *                This value will be logged.
 * @return esp_err_t The `log_ret` value passed to the function, allowing the caller
 *                   to easily return the original error code.
 */
static esp_err_t oled_init_cleanup(esp_err_t log_ret);

/*******************************************************************************/
/*                             STATIC VARIABLES                                */
/*******************************************************************************/
/* LCD Panel handles. In our case we are using I2C for the SSD1306 OLED,
 * which is not an LCD panel, but the API is designed to be generic for various types of displays.
 * Both LCD and OLED technologies can control individual pixels, so the very general idea is similar */
static esp_lcd_panel_io_handle_t io_handle = NULL;
static esp_lcd_panel_handle_t panel_handle = NULL;

/* Buffer for the OLED display. The size is determined by the width and height of the display.
 * The buffer is used to store pixel data before sending it to the display. */
#ifdef UNIT_TEST
/* For unit testing, we expose the buffer to allow verification of its contents.
 * This is done by NOT defining the buffer as static in this file. */
uint8_t oled_buffer[OLED_WIDTH * (OLED_HEIGHT / 8)];
#else
/* For normal operation, we keep the buffer static to limit its scope to this file. */
static uint8_t oled_buffer[OLED_WIDTH * (OLED_HEIGHT / 8)];
#endif

/*******************************************************************************/
/*                     GLOBAL FUNCTION DEFINITIONS                             */
/*******************************************************************************/

esp_err_t oled_init(void)
{
    esp_err_t ret = ESP_OK;
    i2c_master_bus_handle_t i2c_bus_handle = NULL;

    /* #01 - Initialize I2C bus (if not already done) for communication between ESP32 and OLED display */
    ESP_LOGI(TAG, "Initializing I2C master bus");
    ret = i2c_manager_init(OLED_I2C_PORT, OLED_I2C_SDA_PIN, OLED_I2C_SCL_PIN);
    if ((ret == ESP_OK) || (ret == ESP_ERR_INVALID_STATE))
    {
        /* We either successfully initialized I2C master bus or it was already initialized.
         * In either case, we can proceed to get the bus handle. 
         * The bus handle will be used for all I2C operations for our OLED display */
        ret = i2c_manager_get_bus_handle(&i2c_bus_handle);
        if (ret != ESP_OK) 
        {
            ESP_LOGE(TAG, "Failed to get I2C bus handle: %s", esp_err_to_name(ret));
            return oled_init_cleanup(ret);
        }
        else
        {
            ESP_LOGI(TAG, "oled_init successfully retrieved the I2C bus handle");
        }
    }
    else
    {
        ESP_LOGE(TAG, "Failed to initialize I2C master bus: %s", esp_err_to_name(ret));
        return oled_init_cleanup(ret);
    }

    /* #02 - Initialize the panel IO */
    /* A "panel" refers to the display module itself, including its controller chip (like the SSD1306 for our OLED).
     * The esp_lcd_panel_handle_t (panel_handle in your code) is a software handle that represents this physical display panel.
     **/
    ESP_LOGI(TAG, "Installing panel IO");
    /* Define the panel IO configuration (I2C communication with the SSD1306 controller) */
    esp_lcd_panel_io_i2c_config_t io_config =
    {
        .dev_addr = OLED_I2C_ADDR, // I2C address of the OLED display
        .scl_speed_hz = OLED_PIXEL_CLOCK_HZ, // I2C clock speed (400KHz for fast mode)
        .control_phase_bytes = 1, // After sending the slave address, you typically send 1 control byte before sending command or data bytes (aligns with SSD1306 datasheet)
        .lcd_cmd_bits = 8,        // SSD1306 commands are 8 bits (1 byte) wide (aligns with SSD1306 datasheet)
        .lcd_param_bits = 8,      // Parameters associated with commands, as well as display RAM data, are also sent as 8-bit units. (aligns with SSD1306 datasheet)
        .dc_bit_offset = 6,       // The Data/Command (D/C#) select bit is indeed bit 6 (aligns with SSD1306 datasheet)
    };

    /* Create a new panel IO instance using the global I2C bus handle and the defined panel IO configuration */
    ret = esp_lcd_new_panel_io_i2c(i2c_bus_handle, &io_config, &io_handle);
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "New panel IO failed: %s", esp_err_to_name(ret));
        return oled_init_cleanup(ret); // Cleanup resources allocated so far and return the error
    }

    /* #03 - Initialize the panel driver itself using the panel IO handle for communication */
    /* Earlier we initialized the panel's IO which is the communication interface between the ESP32 and the panel controller,
     * now we initialize the panel driver which is responsible for controlling the panel's behavior and settings
     **/
    ESP_LOGI(TAG, "Installing SSD1306 panel driver (Height: %d)", OLED_HEIGHT);
    esp_lcd_panel_dev_config_t panel_config =
    {
        .bits_per_pixel = 1,  // 1 bit per pixel for monochrome display
        .reset_gpio_num = -1, // No reset GPIO used for I2C OLED
        .vendor_config = &(esp_lcd_panel_ssd1306_config_t) // vendor config structure, only parameter is height
        {
            .height = OLED_HEIGHT,
        }
    };

    /* Use the panel IO handle and the defined panel configuration to create a new panel under our global panel_handle */
    ret = esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle);
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "New panel failed: %s", esp_err_to_name(ret));
        return oled_init_cleanup(ret); // Cleanup resources allocated so far and return the error
    }

    /* #04 - Initialize the panel */
    ret = esp_lcd_panel_reset(panel_handle); // Perform software reset
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "Panel reset failed: %s", esp_err_to_name(ret));
        return oled_init_cleanup(ret); // Cleanup resources allocated so far and return the error
    }
    ret = esp_lcd_panel_init(panel_handle); // Initialize panel registers
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "Panel init failed: %s", esp_err_to_name(ret));
        return oled_init_cleanup(ret); // Cleanup resources allocated so far and return the error
    }

    /* #05 - Mirror the LCD panel on specific axis (x,y).
     * Try different combinations if needed:
     *      (false, false) -> No mirroring
     *      (true, true)   -> 180 degree rotation (often default) - correct option in our case
     *      (true, false)  -> Mirror on X-axis only
     *      (false, true)  -> Mirror on Y-axis only
     **/
    ESP_LOGI(TAG, "Setting panel mirror");
    ret = esp_lcd_panel_mirror(panel_handle, true, true);
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "Panel mirror failed: %s", esp_err_to_name(ret));
        return oled_init_cleanup(ret); // Cleanup resources allocated so far and return the error
    }

    /* #06 - Turn on the display and clear the buffer */
    ret = esp_lcd_panel_disp_on_off(panel_handle, true);
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "Panel turn on failed: %s", esp_err_to_name(ret));
        return oled_init_cleanup(ret); // Cleanup resources allocated so far and return the error
    }
    oled_clear_buffer();  // Clear the buffer before first use
    ret = oled_refresh(); // Refresh the display with the cleared buffer, resulting in a blank screen (clean slate)
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "Initial clear/refresh failed: %s", esp_err_to_name(ret));
        return oled_init_cleanup(ret); // Cleanup resources allocated so far and return the error
    }

    ESP_LOGI(TAG, "OLED initialized successfully");
    return ret; // Should be ESP_OK if we reach here
}


esp_err_t oled_refresh(void) 
{
    /* Make sure the panel handle is valid before trying to draw */
    if (!panel_handle) 
    {
        return ESP_ERR_INVALID_STATE;
    }

    /* Draws the contents of the OLED buffer (the bitmap) onto the physical display. */
    /* The esp_lcd_panel_draw_bitmap function parameters:
     *     - panel_handle The handle to the initialized LCD panel driver instance.
     *     - x_start The starting horizontal coordinate (0) on the panel.
     *     - y_start The starting vertical coordinate (0) on the panel.
     *     - x_end The ending horizontal coordinate (OLED_WIDTH) on the panel (exclusive).
     *     - y_end The ending vertical coordinate (OLED_HEIGHT) on the panel (exclusive).
     *     - oled_buffer Pointer to the buffer containing the bitmap data to be drawn. 
     *       The size of this buffer should correspond to OLED_WIDTH * OLED_HEIGHT.   
     * 
     * The oled_buffer contains the pixel data for the display, where each bit represents a pixel.
     * 
     ******************* Let's understand how to oled_buffer is mapped to our SSD1306 128x32 display:***************************
     * 
     * The `oled_buffer` is a linear array of `uint8_t` with a size of `OLED_WIDTH * (OLED_HEIGHT / 8)`, 
     * which is 128 * (32 / 8) = 512 bytes for this display.
     * The SSD1306 organizes its display memory (GDDRAM) into "pages". Each page consists of 8 rows of pixels.
     * For a 128x32 display, there are 32 rows / 8 rows/page = 4 pages (Page 0 to Page 3).
     *
     * Buffer Layout:
     * - The buffer stores the pixel data page by page, column by column within each page.
     * - Bytes 0 to 127 correspond to Page 0 (Rows 0-7), covering columns 0 to 127 (oled_buffer[0] to oled_buffer[127]).
     * - Bytes 128 to 255 correspond to Page 1 (Rows 8-15), covering columns 0 to 127 (oled_buffer[128] to oled_buffer[255]).
     * - Bytes 256 to 383 correspond to Page 2 (Rows 16-23), covering columns 0 to 127 (oled_buffer[256] to oled_buffer[383]).
     * - Bytes 384 to 511 correspond to Page 3 (Rows 24-31), covering columns 0 to 127 (oled_buffer[384] to oled_buffer[511]).
     *
     * Byte-to-Pixel Mapping:
    * - Each `uint8_t` value in the `oled_buffer` represents a single vertical column of 8 pixels
    *   within a specific page and at a specific horizontal position (column). Since 1 page = 8 rows, 
    *   this perfectly lines up with our 8-bit column height, which also works with our 8-bit font (font8x8_basic).
    * - The index of the byte determines the page and column:
    *   - `page = index / OLED_WIDTH`       (0 for indices 0-127, 1 for 128-255, etc.)
    *   - `column = index % OLED_WIDTH`     (0-127, OLED_WIDTH-1 is the max due to the modulo operation)
    * 
    * - The bits within each byte correspond to the 8 vertical pixels in that column segment:
    *   - Bit 0 (LSB - Least Significant Bit) controls the top pixel of the 8-pixel segment (row `page * 8 + 0`).
    *   - Bit 1 controls the next pixel down (row `page * 8 + 1`).
    *   - ...
    *   - Bit 7 (MSB - Most Significant Bit) controls the bottom pixel of the 8-pixel segment (row `page * 8 + 7`).
    * 
    * - A bit value of '1' typically means the pixel is ON (lit), and '0' means OFF (dark).
    *
    * Example:
    *   - `oled_buffer[0]` controls the pixels in the top-left corner: Column 0, Rows 0-7.
    *   - `oled_buffer[0] & 0x01` (Bit 0) corresponds to pixel (0, 0).
    *   - `oled_buffer[0] & 0x80` (Bit 7) corresponds to pixel (0, 7).
    *   - `oled_buffer[1]` controls Column 1, Rows 0-7.
    *   - `oled_buffer[128]` controls Column 0, Rows 8-15.
    *   - `oled_buffer[128] & 0x01` (Bit 0) corresponds to pixel (0, 8).
    *   - `oled_buffer[128] & 0x80` (Bit 7) corresponds to pixel (0, 15).
    * 
    * So for example if oled_buffer[0] = 0xFF, all pixels on the first page, in the first column (0,0) to (0,7) are ON.
    * If oled_buffer[128] = 0xFF, all pixels on the second page, in the first column (0,8) to (0,15) are ON. 
    * and so on.
    *
    * Coordinate System: (0,0) is the top-left pixel. X increases to the right, Y increases downwards.
    *
    * This mapping is consistent with how the `oled_draw_char` function calculates the buffer index (`buf_idx`)
    * and sets the individual bits (`oled_buffer[buf_idx] |= (1 << bit_in_page)`). The `esp_lcd_panel_draw_bitmap`
    * function sends this buffer to the display controller, which interprets it according to this structure.
    *
    **/
    return esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, OLED_WIDTH, OLED_HEIGHT, oled_buffer);
}


void oled_clear_buffer(void) 
{
    /* Clear the OLED buffer by setting all bits to 0 (empty screen) */
    memset(oled_buffer, 0, sizeof(oled_buffer));
}


void oled_write_string(uint8_t line, const char *text) 
{
    /* Make sure the line number is valid (0-3 if we are using 8x8 font on a 128x32 display) */
    if (line >= SSD1306_128x32_NUM_OF_LINES) 
    {
        ESP_LOGW(TAG, "Invalid line number: %d", line);
        return;
    }

    /* Check if the font size can fit on the display and is not zero */
    if ((FONT_WIDTH == 0) || (FONT_HEIGHT == 0) || (FONT_WIDTH > OLED_WIDTH) || (FONT_HEIGHT > OLED_HEIGHT))
    {
        /* Check if the font width and height are valid */
        ESP_LOGW(TAG, "Invalid font size: %dx%d", FONT_WIDTH, FONT_HEIGHT);
        return;
    }

    /* #01 - Initialize the starting coordinates for the character drawing */
    /* The y-coordinate is calculated based on the line number and the font height which results in the top-left corner of the line 
     * The x-coordinate is initialized to 0, as we start drawing from the leftmost position on the line 
     * Remember that on SSD1306 (0,0) coordinate is the top-left corner of the display. X grows to the right, Y grows downwards.
     **/
    int y = line * FONT_HEIGHT; // (For 8x8 font this would be 0 for line 0, 8 for line 1, 16 for line 2, 24 for line 3)
    int x = 0;

    /* #02 - Calculate which pages the text will occupy */
    int page_start = y / SSD1306_PAGE_HEIGHT; // (Page 0 for y==0, Page 1 for y==8, Page 2 for y==16, Page 3 for y==24)
    int page_end = (y + (FONT_HEIGHT - 1)) / SSD1306_PAGE_HEIGHT; // (For fonts up to 8 pixels high, this will be the same as page_start, since it fits in one page)

    /* #03 - Clear the buffer for the pages that will be used by this line */
    /* Iterate through the pages that will be used by this line and clear the buffer for those pages */
    for (int page = page_start; ((page <= page_end) && (page < SSD1306_128x32_NUM_OF_PAGES)); page++) 
    {
        /* Calculate the starting index for this page in the oled_buffer */
        uint16_t start_idx = SSD1306_PAGE_START_INDEX(page);
        /* Specify how many bytes (columns) we need to clear for this line */
        size_t clear_len = OLED_WIDTH; // Clear the entire width of the display for this page
        /* Assert that the calculated clear operation stays within the buffer bounds.
         * This should always be true if OLED_WIDTH, OLED_HEIGHT, and oled_buffer are defined correctly. */
        assert(start_idx + clear_len <= sizeof(oled_buffer));
        /* Clear the buffer for this page by setting all bits to 0 (empty screen) */
        memset(&oled_buffer[start_idx], 0, clear_len);
    }
    /* #04 - Draw the string character by character on the clean pages */
    /* Iterate through each character in the string until we reach the null terminator or until we reach the end of the line (OLED_WIDTH).
     *
     * Note: we do not directly validate the length of the string here - an error is NOT
     * returned if the string is too long. Instead, we just stop drawing when we reach the end of the line, implicitly truncating the string.
     * This is a design choice to keep the function simple and avoid unnecessary complexity.
     * If you want to handle long strings differently, you can add additional logic here.
     **/
    while (*text) // Iterate through each character in the string until we reach the null terminator
    {
        /* Draw the character at the current position (x, y) */
        oled_draw_char(x, y, *text); // This is done first because we assume we can ALWAYS draw at least the first character
        /* Move to the next character position to the right of the current character (based on font width) */
        x += FONT_WIDTH;
        /* Check if we have reached the end of the line (OLED_WIDTH). If so, stop drawing - the string will be truncated */
        if (x >= OLED_WIDTH) 
        {
            ESP_LOGW(TAG, "String truncated at line %d", line);
            break;
        }
        /* Move to the next character in the string */
        text++;
    }
}

/*******************************************************************************/
/*                     STATIC FUNCTION DEFINITIONS                             */
/*******************************************************************************/

static void oled_draw_char(int x, int y, char char_to_draw) 
{
    /* #01 - Check if the character can fit on the display and the coordinates are valid */
    if ((x < 0) || (y < 0) || (x + FONT_WIDTH > OLED_WIDTH) || (y + FONT_HEIGHT > OLED_HEIGHT))
    {
        ESP_LOGW(TAG, "Char '%c' potentially out of bounds (%d, %d)", char_to_draw, x, y);
        return; // Do not draw if out of bounds
    }

    /* We assume the character provided to this function is a valid ASCII character from the Standard ASCII table (0-127, 7-bit).
     * The font8x8_basic array which is used to provide the bitmap font data for the character is...
     *      ...designed to support the full 7-bit ASCII character set (0-127).
     *      ...indexed by the ASCII value of the character, so we can directly use it to get the font data.
     *      ...a 2D array where each row corresponds to a character and each column corresponds to a pixel column in that character.
     *      ...declared in the font8x8_basic.h file included at the top of this file.
     * 
     * But since relying on assumptions is not a good practice, let's make sure the character value is within (0-127) range to avoid
     * any potential out-of-bounds access to the font8x8_basic array.
     **/
    if ((char_to_draw > 127) /* || (char_to_draw < 0) (on this platform this is always false since char is unsigned. 
                              Might need to be checked on other platforms though) */) 
    {
        /* If the character is not in the supported range, log a warning and set it to '?'. 
         * This is a fallback mechanism to handle unsupported characters gracefully. 
         * The '?' character (ASCII 63) is a common placeholder for unsupported characters. 
         * This will also help in debugging if we see a lot of '?' characters on the display. */
        ESP_LOGW(TAG, "Unsupported character: %c (ASCII: %d)", char_to_draw, (unsigned char)char_to_draw);
        char_to_draw = '?'; // Default to '?' for unsupported characters
    }

    /* #02 - Now that we know the character value is within the supported range (0-127), we can safely use it to index the font8x8_basic array */
    /* font8x8_basic is a 2D array of font data but if we reference just the row (character) it will decay to a pointer to the first element of that row.
     * The first dimension is the character code (0-127), and the second dimension is the 8 bytes defining the character bitmap.
     * So we can directly use the character value to get the font data for that character. */
    const uint8_t *font_ptr = font8x8_basic[(uint8_t)char_to_draw];

    /* #03 - Draw the characters pixel by pixel */
    /* #03.1 - Iterate through each column (x-axis) of the font character (0-7). We are drawing the character column by column */
    for (int font_col = 0; font_col < FONT_WIDTH; font_col++) 
    {
        uint8_t col_data = font_ptr[font_col]; // Get the 8 bits for this vertical column

        /* #03.2 - Iterate through each row (y-axis) of the font character (0-7). We are drawing the character row by row */
        for (int font_row = 0; font_row < FONT_HEIGHT; font_row++) 
        {
            /* Calculate the screen coordinates for the current pixel in the character */
            int screen_x = x + font_col;
            int screen_y = y + font_row;

            /* Double-check the screen coordinates are within the bounds of the display */
            if ((screen_x < 0) || (screen_y < 0) || (screen_x >= OLED_WIDTH) || (screen_y >= OLED_HEIGHT))
            {
                continue; // Skip the pixel if it's out of bounds
            }

            /* Calculate the page number and the bit position within that page */
            /* The SSD1306 display is organized in pages, each page is 8 pixels high.
             * The page number is determined by dividing the y-coordinate by the page height (8 pixels).
             * The bit position within that page is determined by taking the modulo of the y-coordinate by the page height (8 pixels).
             **/
            int page = screen_y / SSD1306_PAGE_HEIGHT; // Page number (0-3 for 128x32 display)
            int bit_in_page = screen_y % SSD1306_PAGE_HEIGHT; // Bit position within the column on the page (0-7, page is 8 pixels high)
            
            /* Calculate the buffer index for the OLED buffer based on the screen coordinates */
            uint16_t buf_idx = screen_x + (page * OLED_WIDTH); // Buffer index (0-511 for 128x32 display)

            /* Triple-check the buffer index is within the bounds of the OLED buffer (better safe than sorry I guess) */
            if (buf_idx >= sizeof(oled_buffer)) 
            {
                ESP_LOGE(TAG, "Buffer overflow attempt: idx=%u, size=%u", buf_idx, sizeof(oled_buffer));
                continue;
            }

            /* #03.3 - Set or clear the corresponding bit (pixel) in the OLED buffer based on the font data */
            /* The font data is stored in the font8x8_basic array, where each bit represents a pixel in the character.
             *     - If the bit is set (1), we want to set the corresponding pixel in the OLED buffer to ON (1).
             *     - If the bit is clear (0), we want to leave it as is (since we already cleared the buffer before drawing).
             **/
            if ((col_data >> font_row) & 0x01) /* Pixel should be 1 (ON) */
            {
                /* Set the corresponding bit in the buffer at (screen_x, screen_y) */    
                oled_buffer[buf_idx] |= (1 << bit_in_page);
            } 
            else /* Pixel should be 0 (OFF) */
            {
                /* Do nothing since we already cleared the buffer before drawing */
            }
        }
    }
}

static esp_err_t oled_init_cleanup(esp_err_t log_ret)
{
    ESP_LOGE(TAG, "OLED initialization failed with error: %s", esp_err_to_name(log_ret));
    /* Cleanup resources allocated during initialization. Set handles to NULL to avoid dangling pointers */
    /* Note: The order of deletion is important. Always delete resources in reverse order of allocation. 
     * In this case, we delete the panel first, then the IO handle.
     **/
    if (panel_handle)
    {
        esp_lcd_panel_del(panel_handle);
        panel_handle = NULL;
        ESP_LOGI(TAG, "Deleted panel handle.");
    }
    if (io_handle)
    {
        esp_lcd_panel_io_del(io_handle);
        io_handle = NULL;
        ESP_LOGI(TAG, "Deleted panel IO handle.");
    }

    return log_ret; // Return the original error code
}
