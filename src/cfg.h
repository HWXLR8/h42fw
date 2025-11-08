#pragma once

#include <stdint.h>

#include "hardware/flash.h"

typedef struct __attribute__((packed)) { // force 1B alignment of members
  uint32_t magic;
  uint16_t version;
  uint8_t  enable_leds;
  uint8_t  enable_oled;
  // padding accounting for each of the above members
  uint8_t  _pad[FLASH_PAGE_SIZE - 4 - 2 - 1 - 1];
} cfg_t;

static void init_cfg(cfg_t* c);
bool cfg_load(cfg_t* cfg);
void cfg_save(const cfg_t* cfg);
