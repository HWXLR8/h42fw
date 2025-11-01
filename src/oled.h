#pragma once

#include <stdint.h>

#define I2C_INST    i2c0
#define I2C_SDA_PIN 0
#define I2C_SCL_PIN 1

#define OLED_ADDR 0x3C
#define WIDTH     128
#define HEIGHT    32

void oled_init(void);
void oled_clear(void);
void oled_print(uint8_t page, uint8_t col, const char *s);
