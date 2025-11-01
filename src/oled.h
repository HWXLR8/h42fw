#pragma once

#include <stdint.h>

void oled_init(void);
void oled_clear(void);
void oled_print(uint8_t page, uint8_t col, const char *s);
