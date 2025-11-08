#include "cfg.h"

#include "pico/stdlib.h"
#include "hardware/sync.h"

#include <string.h>
#include <stdint.h>

// last 4KB of flash reserved
#define NVM_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)
#define MAGIC      0xBEEFBABE
#define VERSION    1

// set cfg to sane defaults
static void init_cfg(cfg_t* c) {
  memset(c, 0, sizeof(*c)); // zero all fields
  c->magic = MAGIC;
  c->version = VERSION;
  c->enable_leds = 1;
  c->enable_oled = 1;
}

bool cfg_load(cfg_t* cfg) {
  // return a pointer to the cfg region in XIP
  const cfg_t* f = (const cfg_t*)(XIP_BASE + NVM_OFFSET);
  if (f->magic == MAGIC && f->version == VERSION) {
    memcpy(cfg, f, sizeof(*cfg));
    return true;
  }
  init_cfg(cfg);
  return false;
}

void cfg_save(const cfg_t* cfg) {
  // temporary buffer
  uint8_t page[FLASH_PAGE_SIZE] = {0};
  memcpy(page, cfg, sizeof(*cfg));

  uint32_t irq = save_and_disable_interrupts();
  flash_range_erase(NVM_OFFSET, FLASH_SECTOR_SIZE);
  flash_range_program(NVM_OFFSET, page, FLASH_PAGE_SIZE);
  restore_interrupts(irq);
}
