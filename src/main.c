#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "ws2812.pio.h"
#include "bsp/board.h"
#include "tusb.h"

#include "config.h"

SOCD_MODE socd = LAST_INPUT;

static const button_map BTN_MAP[] = {
  {5,  LEFT},
  {3,  DOWN},
  {4,  RIGHT},
  {2,  B1},
  {10, B2},
  {11, B3},
  {12, B4},
  {13, B5},
  {6,  B6},
  {7,  B7},
  {8,  B8},
  {9,  B9},
  {27, B10},
  {18, B11},
  {26, UP},
  {19, B12},
  // no LEDs
  {14, B13},
  {21, B14},
  {20, B15},
  {16, B16},
  {17, B17},
};

void init_btns() {
  // enable inputs w/ pull ups
  for (uint i = 0; i < BTN_COUNT; ++i) {
    uint p = BTN_MAP[i].pin;
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
      if (absolute_time_diff_us(t0, get_absolute_time()) >= 500000) {
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

static inline uint32_t read_buttons(cd_state* cd) {
  uint32_t mask = 0;

  for (int i = 0; i < BTN_COUNT; ++i) {
    const uint bit = BTN_MAP[i].bit;
    const uint p   = BTN_MAP[i].pin;

    // if button pressed (active low)
    if (!gpio_get(p)) {
      mask |= (uint32_t)(1u << bit);

      // timestamp when pressed
      if (bit == LEFT)  { if (cd->left  == 0) cd->left  = ++cd->time;}
      if (bit == RIGHT) { if (cd->right == 0) cd->right = ++cd->time;}
      if (bit == UP)    { if (cd->up    == 0) cd->up    = ++cd->time;}
      if (bit == DOWN)  { if (cd->down  == 0) cd->down  = ++cd->time;}
    } else {
      if (bit == LEFT)  cd->left  = 0;
      if (bit == RIGHT) cd->right = 0;
      if (bit == UP)    cd->up    = 0;
      if (bit == DOWN)  cd->down  = 0;
    }
  }
  // SOCD cleaning
  if (socd == LAST_INPUT) {
    if (cd->left && cd->right) {
      if (cd->left < cd->right) mask &= ~(1 << LEFT);
      else mask &= ~(1 << RIGHT);
    }
    if (cd->up && cd->down) {
      if (cd->up < cd->down) mask &= ~(1 << UP);
      else mask &= ~(1 << DOWN);
    }
  } else if (socd == NEUTRAL) {
    if (cd->left && cd->right) {
      mask &= ~(1 << LEFT);
      mask &= ~(1 << RIGHT);
    }
    if (cd->up && cd->down) {
      mask &= ~(1 << UP);
      mask &= ~(1 << DOWN);
    }
  }
  return mask;
}

// reorder RGB -> GRB
static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)(g) << 16) | ((uint32_t)(r) << 8) | (uint32_t)(b);
}

int main(void) {
  board_init();
  tusb_init();
  init_btns();

  // ws2812
  PIO pio = pio0;
  uint sm = 0;
  uint offset = pio_add_program(pio, &ws2812_program);
  ws2812_program_init(pio, sm, offset, LED_PIN, 800000, false);
  // shut off all LEDs
  for (int i = 0; i < LED_BTN_COUNT; ++i) {
    pio_sm_put_blocking(pio, sm, urgb_u32(0, 0, 0) << 8);
  }
  sleep_us(T_RESET_US);

  for (int i = 0; i < LED_BTN_COUNT; ++i) {
    pio_sm_put_blocking(pio, sm, urgb_u32(5, 0, 5) << 8);
  }
  sleep_us(T_RESET_US);

  cd_state cd = {0};
  uint32_t prev = 0;

  while (true) {
    tud_task(); // TinyUSB device task
    check_bootsel_hold();

    if (tud_hid_ready()) {
      uint32_t curr = read_buttons(&cd);
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
