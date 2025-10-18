#include "Common.h"
#include <stdarg.h>

#define BUF_SIZE 1024

void log_to_rpi(const char *format, ...) 
{
    char buffer[BUF_SIZE];  // Buffer to hold the formatted string
    va_list args;  // Variable argument list
    va_start(args, format);  // Initialize the argument list
    int len = vsnprintf(buffer, sizeof(buffer), format, args);  // Format the input variable arguments into the buffer
    va_end(args);  // Clean up the argument list
    if (len > 0)  // Check if formatting was successful (len > 0 means no error)
    {
        comms_uart_send((const uint8_t *)buffer, len);  // Send the buffer contents to RPi via UART0
    }
}