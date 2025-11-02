#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"
#include "pico/bootrom.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "ws2812.pio.h"
#include "bsp/board.h"
#include "tusb.h"

#include "main.h"
#include "oled.h"
#include "config.h"

SOCD_MODE SOCD = LAST_INPUT;
bool LEDS_ON = true;

static const btn_cfg BTN_CFG[] = {
  /*
   pin    button   idle RGB   press RGB   debounce time */
  {5,     LEFT,    5, 0, 5,   100, 0, 100,    KEY_DEBOUNCE},
  {3,     DOWN,    5, 0, 5,   100, 0, 100,    KEY_DEBOUNCE},
  {4,     RIGHT,   5, 0, 5,   100, 0, 100,    KEY_DEBOUNCE},
  {2,     B1,      5, 0, 5,   100, 0, 100,    KEY_DEBOUNCE},
  {10,    B2,      5, 0, 5,   100, 0, 100,    KEY_DEBOUNCE},
  {11,    B3,      5, 0, 5,   100, 0, 100,    KEY_DEBOUNCE},
  {12,    B4,      5, 0, 5,   100, 0, 100,    KEY_DEBOUNCE},
  {13,    B5,      5, 0, 5,   100, 0, 100,    KEY_DEBOUNCE},
  {6,     B6,      5, 0, 5,   100, 0, 100,    KEY_DEBOUNCE},
  {7,     B7,      5, 0, 5,   100, 0, 100,    KEY_DEBOUNCE},
  {8,     B8,      5, 0, 5,   100, 0, 100,    KEY_DEBOUNCE},
  {9,     B9,      5, 0, 5,   100, 0, 100,    KEY_DEBOUNCE},
  {27,    B10,     5, 0, 5,   100, 0, 100,    KEY_DEBOUNCE},
  {18,    B11,     5, 0, 5,   100, 0, 100,    KEY_DEBOUNCE},
  {19,    B12,     5, 0, 5,   100, 0, 100,    KEY_DEBOUNCE},
  {26,    UP,      5, 0, 5,   100, 0, 100,    KEY_DEBOUNCE},
  // no LEDs
  {14,    B13,     0, 0, 0,   0, 0, 0,    TAC_DEBOUNCE},
  {21,    B14,     0, 0, 0,   0, 0, 0,    TAC_DEBOUNCE},
  {20,    B15,     0, 0, 0,   0, 0, 0,    TAC_DEBOUNCE},
  {16,    B16,     0, 0, 0,   0, 0, 0,    TAC_DEBOUNCE},
  {17,    B17,     0, 0, 0,   0, 0, 0,    TAC_DEBOUNCE},
};

void init_btns() {
  // enable inputs w/ pull ups
  for (uint i = 0; i < BTN_COUNT; ++i) {
    uint p = BTN_CFG[i].pin;
    gpio_init(p);
    gpio_pull_up(p);
    gpio_set_dir(p, GPIO_IN);
  }
}

typedef struct __attribute__((packed)) {
  // [0 ..20] -> buttons
  // [24..21] -> hat
  uint32_t buttons;
} gamepad_report_t;

static inline bool is_pressed(uint pin, int idx) {
  static bool is_stable[BTN_COUNT];
  static bool was_pressed[BTN_COUNT];
  // next time a press is allowed
  static absolute_time_t unlock_time[BTN_COUNT];

  bool pressed = (gpio_get(pin) == 0);
  absolute_time_t now = get_absolute_time();

  // accept first input
  if (pressed &&
      absolute_time_diff_us(now, unlock_time[idx]) <= 0) {
    is_stable[idx] = true;
  }

  if (!pressed && was_pressed[idx]) {
    unlock_time[idx] = delayed_by_us(now, BTN_CFG[idx].debounce);
    is_stable[idx] = false;
  }

  was_pressed[idx] = pressed;
  return is_stable[idx];
}

typedef struct {
  uint8_t rgb[LED_BTN_COUNT][3];
} led_frame;

// core0 writes, core1 reads
static queue_t ledq;

static void core1_main() {
  // init LEDs
  PIO pio = pio0;
  uint sm = 0;
  uint offset = pio_add_program(pio, &ws2812_program);
  ws2812_program_init(pio, sm, offset, LED_PIN, 800000, false);

  led_frame f;
  while (true) {
    queue_remove_blocking(&ledq, &f);

    for (uint i = 0; i < LED_BTN_COUNT; ++i) {
      // convert RGB -> GRB
      uint32_t color = ((uint32_t)f.rgb[i][1] << 16)
        | ((uint32_t)f.rgb[i][0] << 8)
        | (uint32_t)f.rgb[i][2];
      pio_sm_put_blocking(pio, sm , color << 8);
    }
    sleep_us(LATCH_TIME);
  }
}

static inline void build_led_frame(led_frame* out, const bool pressed[]) {
  for (uint i = 0; i < LED_BTN_COUNT; ++i) {
    uint8_t r = BTN_CFG[i].r;
    uint8_t g = BTN_CFG[i].g;
    uint8_t b = BTN_CFG[i].b;
    if (pressed[i]) {
      r = BTN_CFG[i].rp;
      g = BTN_CFG[i].gp;
      b = BTN_CFG[i].bp;
    }
    out->rgb[i][0] = r;
    out->rgb[i][1] = g;
    out->rgb[i][2] = b;
  }
}

static inline bool same_frame(const led_frame* a, const led_frame* b) {
  return memcmp(a, b, sizeof(*a)) == 0;
}

int main(void) {
  board_init();
  tusb_init();
  init_btns();

  // oled
  oled_init();
  oled_clear();
  oled_print(0, 0, "BEATMANIA IS NOT COOL");
  oled_print(1, 0, "IT'S NOT FUN");
  oled_print(2, 0, "IT'S NOT FRESH");
  oled_print(3, 0, "IT'S NOT GOOD");

  queue_init(&ledq, sizeof(led_frame), 4);
  multicore_launch_core1(core1_main);

  btn_state   bstate = {0};
  uint32_t    prev_bits = 0;
  bool        pressed_led[LED_BTN_COUNT] = {0};
  static bool led_toggle_held = false;
  led_frame   last_sent = {0};

  while (true) {
    tud_task();

    // scan buttons, returns mask + leds pressed
    memset(pressed_led, 0, sizeof(pressed_led));
    uint32_t bmask = 0;

    for (uint i = 0; i < BTN_COUNT; ++i) {
      const uint bit = BTN_CFG[i].bit;
      const uint p   = BTN_CFG[i].pin;
      bool pressed = is_pressed(BTN_CFG[i].pin, i);

      if (pressed) {
        bmask |= (1u << BTN_CFG[i].bit);
        if (i < LED_BTN_COUNT) pressed_led[i] = true;

        // timestamp when pressed
        if (bit == LEFT)  { if (bstate.left  == 0) bstate.left  = ++bstate.time;}
        if (bit == RIGHT) { if (bstate.right == 0) bstate.right = ++bstate.time;}
        if (bit == UP)    { if (bstate.up    == 0) bstate.up    = ++bstate.time;}
        if (bit == DOWN)  { if (bstate.down  == 0) bstate.down  = ++bstate.time;}

        // LED toggle
        if (p == LED_TOGGLE_PIN) {
          if (!led_toggle_held) {
            LEDS_ON = !LEDS_ON;
          }
          led_toggle_held = true;
        }

        // BOOTSEL
        if (p == BOOTSEL_PIN) reset_usb_boot(0, 0);

      } else {
        if (bit == LEFT)  bstate.left  = 0;
        if (bit == RIGHT) bstate.right = 0;
        if (bit == UP)    bstate.up    = 0;
        if (bit == DOWN)  bstate.down  = 0;
        if (p == LED_TOGGLE_PIN) led_toggle_held = false;
      }
    }
    // SOCD cleaning
    if (SOCD == LAST_INPUT) {
      if (bstate.left && bstate.right) {
        if (bstate.left < bstate.right) bmask &= ~(1 << LEFT);
        else bmask &= ~(1 << RIGHT);
      }
      if (bstate.up && bstate.down) {
        if (bstate.up < bstate.down) bmask &= ~(1 << UP);
        else bmask &= ~(1 << DOWN);
      }
    } else if (SOCD == NEUTRAL) {
      if (bstate.left && bstate.right) {
        bmask &= ~(1 << LEFT);
        bmask &= ~(1 << RIGHT);
      }
      if (bstate.up && bstate.down) {
        bmask &= ~(1 << UP);
        bmask &= ~(1 << DOWN);
      }
    }

    // get hat bits
    uint32_t dir = bmask & ((1u<<UP) | (1u<<RIGHT) | (1u<<DOWN) | (1u<<LEFT));
    uint8_t hat = 0x0F; // neutral (Null)
    bool up    = dir & (1u<<UP);
    bool right = dir & (1u<<RIGHT);
    bool down  = dir & (1u<<DOWN);
    bool left  = dir & (1u<<LEFT);
    if (up && right) hat = 1;
    else if (down && right) hat = 3;
    else if (down && left) hat = 5;
    else if (up && left) hat = 7;
    else if (up) hat = 0;
    else if (right) hat = 2;
    else if (down) hat = 4;
    else if (left) hat = 6;
    // remove dpad from buttons, then insert hat at bits 21..24
    uint32_t bits = bmask & ~((1u<<UP) | (1u<<RIGHT) | (1u<<DOWN) |(1u<<LEFT));
    bits |= ((uint32_t)(hat & 0x0F)) << 21;

    // send HID report
    if (tud_hid_ready() && bits != prev_bits) {
      gamepad_report_t rpt = {.buttons = bits};
      tud_hid_report(0, &rpt, sizeof(rpt));
      prev_bits = bits;
    }

    // enqueue LED frame only if LEDs changed
    led_frame f;
    if (LEDS_ON) {
      build_led_frame(&f, pressed_led);
    } else {
      // disable LEDS
      for (uint i = 0; i < LED_BTN_COUNT; ++i) {
        f.rgb[i][0] = 0;
        f.rgb[i][1] = 0;
        f.rgb[i][2] = 0;
      }
    }
    if (!same_frame(&f, &last_sent)) {
      // try to enqueue
      if (!queue_try_add(&ledq, &f)) {
        led_frame scratch;
        // free one slot
        queue_try_remove(&ledq, &scratch);
        queue_try_add(&ledq, &f);
      }
      last_sent = f;
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
