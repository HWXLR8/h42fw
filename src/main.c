#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/gpio.h"
#include "bsp/board.h"
#include "tusb.h"

#define BTN_PIN 2

static void check_bootsel_hold(void) {
  static bool was_low = false;
  static absolute_time_t t0;

  bool low = (gpio_get(BTN_PIN) == 0) ;
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
  uint8_t buttons;  // bit0 = Button 1, bits1..7 padding
  uint8_t pad;      // send 0
} gamepad_report_t;

static inline uint8_t read_button(void) {
  // Active low button to GND with pull-up
  return gpio_get(BTN_PIN) ? 0 : 1; // 1 when pressed
}

int main(void) {
  board_init();
  tusb_init();

  gpio_init(BTN_PIN);
  gpio_pull_up(BTN_PIN);
  gpio_set_dir(BTN_PIN, GPIO_IN);

  uint8_t last = 0;

  while (true) {
    tud_task(); // TinyUSB device task
    check_bootsel_hold();

    if (tud_hid_ready()) {
      uint8_t now = read_button();
      if (now != last) {
        gamepad_report_t rpt = { .buttons = (uint8_t)(now & 0x01) };
        tud_hid_report(0, &rpt, sizeof(rpt)); // single report, no ID
        last = now;
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
