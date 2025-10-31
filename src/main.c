#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "ws2812.pio.h"
#include "bsp/board.h"
#include "tusb.h"

#include "config.h"

SOCD_MODE SOCD = LAST_INPUT;
bool LED_STATE = true;

static const button_map BTN_MAP[] = {
  {5,  LEFT,  5, 0, 5},
  {3,  DOWN,  5, 0, 5},
  {4,  RIGHT, 5, 0, 5},
  {2,  B1,    5, 0, 5},
  {10, B2,    5, 0, 5},
  {11, B3,    5, 0, 5},
  {12, B4,    5, 0, 5},
  {13, B5,    5, 0, 5},
  {6,  B6,    5, 0, 5},
  {7,  B7,    5, 0, 5},
  {8,  B8,    5, 0, 5},
  {9,  B9,    5, 0, 5},
  {27, B10,   5, 0, 5},
  {18, B11,   5, 0, 5},
  {19, B12,   5, 0, 5},
  {26, UP,    5, 0, 5},
  // no LEDs
  {14, B13,   0, 0, 0},
  {21, B14,   0, 0, 0},
  {20, B15,   0, 0, 0},
  {16, B16,   0, 0, 0},
  {17, B17,   0, 0, 0},
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

// reorder RGB -> GRB
static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)(g) << 16) | ((uint32_t)(r) << 8) | (uint32_t)(b);
}

static inline void leds_off(PIO pio, uint sm) {
  for (int i = 0; i < LED_BTN_COUNT; ++i) {
    pio_sm_put_blocking(pio, sm, urgb_u32(0, 0, 0) << 8);
  }
  sleep_us(LATCH_TIME);
}

static inline void leds_on(PIO pio, uint sm) {
  for (uint i = 0; i < LED_BTN_COUNT; ++i) {
    uint r, g, b;
    r = BTN_MAP[i].r;
    g = BTN_MAP[i].g;
    b = BTN_MAP[i].b;
    pio_sm_put_blocking(pio, sm, urgb_u32(r, g, b) << 8);
  }
  sleep_us(LATCH_TIME);
}

static inline void toggle_leds(PIO pio, uint sm) {
  if (LED_STATE) {
    leds_off(pio, sm);
  } else {
    leds_on(pio, sm);
  }
  LED_STATE = !LED_STATE;
}

static void process_live_config(PIO pio, uint sm) {
  //// BOOTSEL ////
  static bool was_low = false;
  static absolute_time_t t0;

  bool low = (gpio_get(BOOTSEL_PIN) == 0) ;
  if (low) { // button currently held
    if (!was_low) {
      was_low = true;
      t0 = get_absolute_time();
    } else {
      if (absolute_time_diff_us(t0, get_absolute_time()) >= BOOTSEL_DELAY) {
        reset_usb_boot(0, 0);
      }
    }
  } else { // button was released
    was_low = false;
  }

  //// LED TOGGLE ////
  static bool was_pressed = false;
  // next time a press is allowed
  static absolute_time_t unlock_time = {0};
  bool pressed = (gpio_get(LED_TOGGLE_PIN) == 0);
  absolute_time_t now = get_absolute_time();

  // accept first input
  if (pressed &&
      !was_pressed &&
      absolute_time_diff_us(now, unlock_time) <= 0) {
    toggle_leds(pio, sm);
  }

  // on release, wait for debounce
  if (!pressed && was_pressed) {
    unlock_time = delayed_by_us(now, TAC_DEBOUNCE);
  }

  was_pressed = pressed;
}

typedef struct __attribute__((packed)) {
  uint32_t buttons;  // bit0 -> bit20 for 21 btns
} gamepad_report_t;

static inline uint32_t read_buttons(btn_state* bstate) {
  uint32_t mask = 0;

  for (int i = 0; i < BTN_COUNT; ++i) {
    const uint bit = BTN_MAP[i].bit;
    const uint p   = BTN_MAP[i].pin;
    bool pressed = !gpio_get(p);

    // if button pressed (active low)
    if (pressed) {
      mask |= (uint32_t)(1u << bit);

      // timestamp when pressed
      if (bit == LEFT)  { if (bstate->left  == 0) bstate->left  = ++bstate->time;}
      if (bit == RIGHT) { if (bstate->right == 0) bstate->right = ++bstate->time;}
      if (bit == UP)    { if (bstate->up    == 0) bstate->up    = ++bstate->time;}
      if (bit == DOWN)  { if (bstate->down  == 0) bstate->down  = ++bstate->time;}
    } else {
      if (bit == LEFT)  bstate->left  = 0;
      if (bit == RIGHT) bstate->right = 0;
      if (bit == UP)    bstate->up    = 0;
      if (bit == DOWN)  bstate->down  = 0;
    }
  }
  // SOCD cleaning
  if (SOCD == LAST_INPUT) {
    if (bstate->left && bstate->right) {
      if (bstate->left < bstate->right) mask &= ~(1 << LEFT);
      else mask &= ~(1 << RIGHT);
    }
    if (bstate->up && bstate->down) {
      if (bstate->up < bstate->down) mask &= ~(1 << UP);
      else mask &= ~(1 << DOWN);
    }
  } else if (SOCD == NEUTRAL) {
    if (bstate->left && bstate->right) {
      mask &= ~(1 << LEFT);
      mask &= ~(1 << RIGHT);
    }
    if (bstate->up && bstate->down) {
      mask &= ~(1 << UP);
      mask &= ~(1 << DOWN);
    }
  }
  return mask;
}

int main(void) {
  board_init();
  tusb_init();
  init_btns();

  // init LEDs
  PIO pio = pio0;
  uint sm = 0;
  uint offset = pio_add_program(pio, &ws2812_program);
  ws2812_program_init(pio, sm, offset, LED_PIN, 800000, false);
  leds_off(pio, sm);
  leds_on(pio,sm);

  btn_state bstate = {0};
  uint32_t prev = 0;

  while (true) {
    tud_task(); // TinyUSB device task
    process_live_config(pio, sm);

    if (tud_hid_ready()) {
      uint32_t curr = read_buttons(&bstate);
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
