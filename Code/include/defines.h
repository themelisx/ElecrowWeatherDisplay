#ifndef DEFINES_h
#define DEFINES_h

#include "configuration.h"

// Position in EEPROM
#define EEPROM_DAYLIGHT 1

// Colors
#define COLOR_WHITE 0xFFFFFF
#define COLOR_BLACK 0
#define COLOR_RED 0xFF0000
#define COLOR_BLUE 0x0077FF
#define COLOR_BLUE_LIGHT 0x22A8FD
#define COLOR_GREY 0x8F8F8F

// Device Elecrow 5 & 7 inches
#define SD_MOSI 11
#define SD_MISO 13
#define SD_SCK 12
#define SD_CS 10

#define I2S_DOUT      17
#define I2S_BCLK      42
#define I2S_LRC       18

#define GPIO_D  38

#define UART_RX 44
#define UART_TX 43
#define I2C_SDA 19
#define I2C_SCL 20
#define LCD_Backlight	2

// Other defines
#define DELAY_MAIN_TASK 500
#define OLED_RESET     -1 

#define TFT_BL 2
#define LGFX_USE_V1

#endif
