#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/gpio.h"
#include "bsp/board.h"
#include "tusb.h"

#define BTN_COUNT 21
#define BOOTSEL_PIN 27
static const uint BTN_PINS[] = {
  5,  3, 4,
  2,
  10,11,12,13,
  6,  7, 8, 9,
  27,   18,
  26,   19,
  14,   21, 20, 16, 17,
};


void init_btns() {
  for (uint i = 0; i < BTN_COUNT; ++i) {
    uint p = BTN_PINS[i];
    gpio_init(p);
    gpio_pull_up(p);
    gpio_set_dir(p, GPIO_IN);
  }
}

static void check_bootsel_hold(void) {
  static bool was_low = false;
  static absolute_time_t t0;

  bool low = (gpio_get(BOOTSEL_PIN) == 0) ;
  if (low) { // button currently held
    if (!was_low) {
      was_low = true;
      t0 = get_absolute_time();
    } else {
      // 5s elapsed?
      if (absolute_time_diff_us(t0, get_absolute_time()) >= 5000000) {
        reset_usb_boot(0, 0);
      }
    }
  } else { // button was released
    was_low = false;
  }
}

typedef struct __attribute__((packed)) {
  uint32_t buttons;  // bit0 -> bit20 for 21 btns
} gamepad_report_t;

static inline uint32_t read_buttons() {
  uint32_t mask = 0;
  for (int i = 0; i < BTN_COUNT; ++i) {
    if (!gpio_get(BTN_PINS[i])) { // active low, button pressed
      mask |= (uint32_t)(1u << i);
    }
  }
  return mask;
}

int main(void) {
  board_init();
  tusb_init();
  init_btns();

  uint32_t prev = 0;

  while (true) {
    tud_task(); // TinyUSB device task
    check_bootsel_hold();

    if (tud_hid_ready()) {
      uint32_t curr = read_buttons();
      if (curr != prev) {
        gamepad_report_t rpt = {.buttons = curr};
        tud_hid_report(0, &rpt, sizeof(rpt)); // single report, no ID
        prev = curr;
      }
    }
  }
  return 0;
}

/* ---- TinyUSB HID callbacks (minimal stubs) ---- */
uint16_t tud_hid_get_report_cb(uint8_t itf, uint8_t report_id,
                               hid_report_type_t report_type,
                               uint8_t* buffer, uint16_t reqlen) {
  (void)itf; (void)report_id; (void)report_type; (void)buffer; (void)reqlen;
  return 0; // not used
}

void tud_hid_set_report_cb(uint8_t itf, uint8_t report_id,
                           hid_report_type_t report_type,
                           uint8_t const* buffer, uint16_t bufsize) {
  (void)itf; (void)report_id; (void)report_type; (void)buffer; (void)bufsize;
  // no output/feature reports
}

void tud_hid_report_complete_cb(uint8_t itf, uint8_t const* report, uint16_t len) {
  (void)itf; (void)report; (void)len;
}
