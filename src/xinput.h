#pragma once
#include <stdint.h>

typedef struct __attribute__((packed, aligned(1))){
  uint8_t rid;
  uint8_t rsize;
  uint8_t digital_buttons_1;
  uint8_t digital_buttons_2;
  uint8_t lt;
  uint8_t rt;
  int16_t l_x;
  int16_t l_y;
  int16_t r_x;
  int16_t r_y;
  uint8_t reserved_1[6];
} xinput_report;

extern xinput_report xinput_data;

void xinput_init_report(void);
