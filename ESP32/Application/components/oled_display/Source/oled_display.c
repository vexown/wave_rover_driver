#include "oled_display.h"
#include "driver/i2c_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h" // Includes esp_lcd_panel_ssd1306.h
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"
#include "esp_check.h"
#include <string.h> // For memset

// Configuration from ugv_config.h / oled_ctrl.h
#define OLED_I2C_PORT       I2C_NUM_0 // Choose I2C port 0 or 1
#define OLED_I2C_SDA_PIN    GPIO_NUM_32 // Correct SDA pin from ugv_config.h
#define OLED_I2C_SCL_PIN    GPIO_NUM_33 // Correct SCL pin from ugv_config.h
#define OLED_I2C_ADDR       0x3C // SSD1306_I2C_ADDRESS
#define OLED_WIDTH          128
#define OLED_HEIGHT         32  // Important: Set correct height (32 or 64)
#define OLED_PIXEL_CLOCK_HZ (400 * 1000) // 400KHz

// Font (replace with a proper font later if needed)
// Simple 8x8 font placeholder - requires font8x8_basic.h/c
#include "font8x8_basic.h"
#define FONT_WIDTH 8
#define FONT_HEIGHT 8

static const char *TAG = "OLED_DISPLAY";

static i2c_master_bus_handle_t i2c_bus_handle = NULL;
static esp_lcd_panel_io_handle_t io_handle = NULL;
static esp_lcd_panel_handle_t panel_handle = NULL;

// Internal buffer to draw text before refreshing the panel
// Size = width * (height / 8) for monochrome 1bpp displays
static uint8_t oled_buffer[OLED_WIDTH * (OLED_HEIGHT / 8)];

esp_err_t oled_init(void) {
    esp_err_t ret = ESP_OK;

    ESP_LOGI(TAG, "Initializing I2C master");
    // Initialize I2C bus if not already done
    if (!i2c_bus_handle) {
        i2c_master_bus_config_t i2c_mst_config = {
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .i2c_port = OLED_I2C_PORT,
            .scl_io_num = OLED_I2C_SCL_PIN,
            .sda_io_num = OLED_I2C_SDA_PIN,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true,
        };
        ret = i2c_new_master_bus(&i2c_mst_config, &i2c_bus_handle);
        ESP_GOTO_ON_ERROR(ret, err, TAG, "I2C new master bus failed: %s", esp_err_to_name(ret));
    }

    ESP_LOGI(TAG, "Installing panel IO");
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = OLED_I2C_ADDR,
        .scl_speed_hz = OLED_PIXEL_CLOCK_HZ,
        .control_phase_bytes = 1, // According to SSD1306 datasheet (Co = 0, D/C = 0)
        .lcd_cmd_bits = 8,        // According to SSD1306 datasheet
        .lcd_param_bits = 8,      // According to SSD1306 datasheet
        .dc_bit_offset = 6,       // According to SSD1306 datasheet (D/C bit is 6)
        // .flags = { .disable_control_phase = 0 } // Default is control phase enabled
    };
    ret = esp_lcd_new_panel_io_i2c(i2c_bus_handle, &io_config, &io_handle);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "New panel IO failed: %s", esp_err_to_name(ret));

    ESP_LOGI(TAG, "Installing SSD1306 panel driver (Height: %d)", OLED_HEIGHT);
    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = 1,
        .reset_gpio_num = -1, // No reset pin used in original config
        // .color_space = ESP_LCD_COLOR_SPACE_MONOCHROME, // Implicit for 1bpp
        .vendor_config = &(esp_lcd_panel_ssd1306_config_t){
             .height = OLED_HEIGHT, // Set the specific height
        }
    };
    ret = esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "New panel failed: %s", esp_err_to_name(ret));

    ret = esp_lcd_panel_reset(panel_handle); // Perform software reset
    ESP_GOTO_ON_ERROR(ret, err, TAG, "Panel reset failed: %s", esp_err_to_name(ret));
    ret = esp_lcd_panel_init(panel_handle); // Initialize panel registers
    ESP_GOTO_ON_ERROR(ret, err, TAG, "Panel init failed: %s", esp_err_to_name(ret));

    // *** ADDED MIRROR CONTROL HERE ***
    // Explicitly set mirroring. Try different combinations if needed:
    // (false, false) -> No mirroring
    // (true, true)   -> 180 degree rotation (often default)
    // (true, false)  -> Mirror X only
    // (false, true)  -> Mirror Y only
    ESP_LOGI(TAG, "Setting panel mirror");
    ret = esp_lcd_panel_mirror(panel_handle, true, true);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "Panel mirror failed: %s", esp_err_to_name(ret));
    // *** END MIRROR CONTROL ***

    // Turn on the display
    ret = esp_lcd_panel_disp_on_off(panel_handle, true);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "Panel turn on failed: %s", esp_err_to_name(ret));

    // Clear the internal buffer and the display
    oled_clear_buffer();
    ret = oled_refresh();
    ESP_GOTO_ON_ERROR(ret, err, TAG, "Initial clear/refresh failed: %s", esp_err_to_name(ret));

    ESP_LOGI(TAG, "OLED initialized successfully");
    return ESP_OK;

err:
    ESP_LOGE(TAG, "OLED initialization failed!");
    if (panel_handle) {
        esp_lcd_panel_del(panel_handle);
        panel_handle = NULL; // Nullify handle after deletion
    }
    if (io_handle) {
        esp_lcd_panel_io_del(io_handle);
        io_handle = NULL; // Nullify handle after deletion
    }
    // Don't delete the bus here if it might be shared or if init failed before panel creation
    // if (i2c_bus_handle) {
    //     i2c_del_master_bus(i2c_bus_handle);
    //     i2c_bus_handle = NULL; // Nullify handle after deletion
    // }
    return ret;
}

void oled_clear_buffer(void) {
    memset(oled_buffer, 0, sizeof(oled_buffer));
}

// Helper to draw a character into the buffer (Attempting Transposition + Character Mirror Fix)
static void oled_draw_char(int x, int y, char c) {
    // Check character bounds (top-left corner x, y)
    if (x < 0 || x + FONT_HEIGHT > OLED_WIDTH || y < 0 || y + FONT_WIDTH > OLED_HEIGHT) {
         ESP_LOGW(TAG, "Char '%c' potentially out of bounds after transpose (%d, %d)", c, x, y);
    }

    uint16_t char_index = (uint8_t)c; // Use ASCII value directly for font8x8_basic
    const uint8_t *font_ptr = &font8x8_basic[char_index][0];

    // Iterate through each column (x-axis) of the font character (0-7)
    for (int font_col = 0; font_col < FONT_WIDTH; font_col++) {
        uint8_t col_data = font_ptr[font_col]; // Get the 8 bits for this vertical column

        // Iterate through each row (y-axis) of the font character (0-7)
        for (int font_row = 0; font_row < FONT_HEIGHT; font_row++) {

            // --- TRANSPOSE LOGIC with Character Mirror Fix ---
            // Use font_row for X offset (mirrored) and font_col for Y offset
            int screen_x = x + (FONT_HEIGHT - 1 - font_row); // Mirror horizontal mapping within char
            int screen_y = y + font_col;
            // --- END TRANSPOSE LOGIC ---

            // Ensure transposed coordinates are within screen bounds before buffer access
            if (screen_x < 0 || screen_x >= OLED_WIDTH || screen_y < 0 || screen_y >= OLED_HEIGHT) {
                continue;
            }

            // Calculate the buffer index and bit position based on transposed coords
            int page = screen_y / 8;
            int bit_in_page = screen_y % 8;
            uint16_t buf_idx = screen_x + page * OLED_WIDTH;

            // Check buffer bounds (safety check)
            if (buf_idx >= sizeof(oled_buffer)) {
                ESP_LOGE(TAG, "Buffer overflow attempt: idx=%u, size=%u", buf_idx, sizeof(oled_buffer));
                continue;
            }

            // Check if the font pixel (at font_col, font_row) is set
            if ((col_data >> font_row) & 0x01) {
                // Set the corresponding bit in the buffer at (screen_x, screen_y)
                oled_buffer[buf_idx] |= (1 << bit_in_page);
            } else {
                // Clear the corresponding bit in the buffer
                oled_buffer[buf_idx] &= ~(1 << bit_in_page);
            }
        }
    }
}

void oled_write_string(uint8_t line, const char *text) {
    if (line >= (OLED_HEIGHT / FONT_HEIGHT)) {
        ESP_LOGW(TAG, "Invalid line number: %d", line);
        return;
    }
    int y = line * FONT_HEIGHT;
    int x = 0;

    // --- Clear the line area in the buffer first ---
    // Calculate the start and end pages this line occupies
    int page_start = y / 8;
    int page_end = (y + FONT_HEIGHT - 1) / 8;
    // Calculate the number of bytes per line in the buffer
    size_t bytes_per_line_segment = OLED_WIDTH; // Clear full width

    for (int p = page_start; p <= page_end && p < (OLED_HEIGHT / 8); ++p) {
        // Calculate the starting index for this page segment
        uint16_t start_idx = p * OLED_WIDTH;
        // Ensure we don't write past the buffer end
        size_t clear_len = bytes_per_line_segment;
        if (start_idx + clear_len > sizeof(oled_buffer)) {
             clear_len = sizeof(oled_buffer) - start_idx;
             ESP_LOGW(TAG, "Line clear truncated for page %d", p);
        }
        // Clear the relevant part of the buffer for this page
        memset(&oled_buffer[start_idx], 0, clear_len);
    }
    // --- End Clear Line ---


    while (*text) {
        oled_draw_char(x, y, *text);
        x += FONT_WIDTH;
        // Stop if string exceeds screen width
        if (x >= OLED_WIDTH) {
             ESP_LOGW(TAG, "String truncated at line %d", line);
             break;
        }
        text++;
    }
}

esp_err_t oled_refresh(void) {
    if (!panel_handle) {
        return ESP_ERR_INVALID_STATE;
    }
    // Draw the entire buffer to the display
    // esp_lcd_panel_draw_bitmap takes full buffer for monochrome
    return esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, OLED_WIDTH, OLED_HEIGHT, oled_buffer);
}

// Note: You still need the font8x8_basic.h/c files as mentioned before.