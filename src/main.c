#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"
#include "pico/bootrom.h"
#include "hardware/flash.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/sync.h"
#include "ws2812.pio.h"
#include "bsp/board.h"
#include "tusb.h"

#include "config.h"
#include OLED_IMG
#include "anim.h"
#include "font/font5x7.h"


typedef enum {
  PLAY,
  CFG_TURBO,
} CTRL_STATE;
CTRL_STATE CSTATE = PLAY;

typedef enum {
  TURBO_0HZ,
  TURBO_15HZ,
  TURBO_30HZ,
  TURBO_60HZ,
} TURBO_RATE;


typedef enum {
  NEUTRAL,    // L+R = 0 or U+D = 0
  LAST_INPUT, // last input wins
} SOCD_MODE;
SOCD_MODE SOCD = LAST_INPUT;


typedef enum {
  CMD_NONE = 0,
  CMD_TOGGLE_OLED,
  CMD_TOGGLE_LEDS,
} FIFO_CMD;


typedef enum {
  LEFT = 0,
  DOWN,
  RIGHT,
  B1,
  B2,
  B3,
  B4,
  B5,
  B6,
  B7,
  B8,
  B9,
  B10,
  B11,
  B12,
  UP,
  TURBO,
  B13,
  B14,
  B15,
  B16,
} BTN_BIT;


typedef struct {
  uint pin;         // MCU pin
  uint bit;         // bit in the HID report
  uint r, g, b;     // idle color
  uint rp, gp, bp;  // pressed color
  uint debounce;    // debounce time
  TURBO_RATE turbo; // turbo rate
} btn_cfg;


// When remapping buttons, the pin ordering of elements must remain
// unchanged since it corresponds to the order in which the LEDs are
// laid out on the gamepad.
//
// Example:
// pin5 = LED0
// pin3 = LED1
// ...
// pin26 = LED15
//
// To remap buttons, only change the second field. Buttons with no
// LEDs must always be placed at the end.
static btn_cfg BTN_CFG[] = {
// pin   button  idle RGB      press RGB     debounce time  turbo
  {5,    LEFT,     5,  0,  5,  100,  0,100,  KEY_DEBOUNCE,  TURBO_0HZ},
  {3,    DOWN,     5,  0,  5,  100,  0,100,  KEY_DEBOUNCE,  TURBO_0HZ},
  {4,    RIGHT,    5,  0,  5,  100,  0,100,  KEY_DEBOUNCE,  TURBO_0HZ},
  {2,    B1,       5,  0,  5,  100,  0,100,  KEY_DEBOUNCE,  TURBO_0HZ},
  {10,   B2,       5,  0,  5,  100,  0,100,  KEY_DEBOUNCE,  TURBO_0HZ},
  {11,   B3,       5,  0,  5,  100,  0,100,  KEY_DEBOUNCE,  TURBO_0HZ},
  {12,   B4,       5,  0,  5,  100,  0,100,  KEY_DEBOUNCE,  TURBO_0HZ},
  {13,   B5,       5,  0,  5,  100,  0,100,  KEY_DEBOUNCE,  TURBO_0HZ},
  {6,    B6,       5,  0,  5,  100,  0,100,  KEY_DEBOUNCE,  TURBO_0HZ},
  {7,    B7,       5,  0,  5,  100,  0,100,  KEY_DEBOUNCE,  TURBO_0HZ},
  {8,    B8,       5,  0,  5,  100,  0,100,  KEY_DEBOUNCE,  TURBO_0HZ},
  {9,    B9,       5,  0,  5,  100,  0,100,  KEY_DEBOUNCE,  TURBO_0HZ},
  {27,   B10,      5,  0,  5,  100,  0,100,  KEY_DEBOUNCE,  TURBO_0HZ},
  {18,   B11,      5,  0,  5,  100,  0,100,  KEY_DEBOUNCE,  TURBO_0HZ},
  {19,   B12,      5,  0,  5,  100,  0,100,  KEY_DEBOUNCE,  TURBO_0HZ},
  {26,   UP,       5,  0,  5,  100,  0,100,  KEY_DEBOUNCE,  TURBO_0HZ},
  // no LEDs
  {14,   TURBO,    0,  0,  0,    0,  0,  0,  TAC_DEBOUNCE,  TURBO_0HZ},
  {21,   B13,      0,  0,  0,    0,  0,  0,  TAC_DEBOUNCE,  TURBO_0HZ},
  {20,   B14,      0,  0,  0,    0,  0,  0,  TAC_DEBOUNCE,  TURBO_0HZ},
  {16,   B15,      0,  0,  0,    0,  0,  0,  TAC_DEBOUNCE,  TURBO_0HZ},
  {17,   B16,      0,  0,  0,    0,  0,  0,  TAC_DEBOUNCE,  TURBO_0HZ},
};


// cardinal directions
typedef struct {
  uint l;
  uint r;
  uint u;
  uint d;
  uint64_t time;
} dpad_state;


typedef struct {
  const uint8_t (*frames)[1024];
  uint16_t frame_n;  // frame count
  uint16_t frame_i;  // frame index
  uint8_t page;      // 0 -> 7
  uint8_t col;       // 0 -> 127
  uint32_t frame_ms; // frame time
  bool frame_sent;   // has the frame been sent
  absolute_time_t start; // frame start
} oled_anim;


typedef struct __attribute__((packed)) { // force 1B alignment of members
  uint32_t magic;
  uint16_t version;
  uint8_t  enable_leds;
  uint8_t  enable_oled;
  // padding accounting for each of the above members
  uint8_t  _pad[FLASH_PAGE_SIZE - 4 - 2 - 1 - 1];
} pad_cfg;
pad_cfg cfg;


typedef struct {
  uint32_t rgb[LED_BTN_COUNT][3];
} led_frame;


typedef struct __attribute__((packed)) {
  // [0 ..20] -> buttons
  // [24..21] -> hat
  uint32_t buttons;
} gamepad_report;


typedef struct {
  bool active;
  bool triggered;
  uint32_t btns;
  absolute_time_t expiry;
} timer;


// function pointer type for combo actions
typedef void (*combo_action)(void);


// we need a list of combos
// uint32_t mask
// duration
typedef struct {
  uint32_t btns;
  absolute_time_t duration;
  combo_action action;
} combo;


// set cfg to sane defaults
static void init_cfg(pad_cfg* c) {
  memset(c, 0, sizeof(*c)); // zero all fields
  c->magic = MAGIC;
  c->version = VERSION;
  c->enable_leds = 1;
  c->enable_oled = 1;
}


bool cfg_load(pad_cfg* cfg) {
  // return a pointer to the cfg region in XIP
  const pad_cfg* f = (const pad_cfg*)(XIP_BASE + NVM_OFFSET);
  if (f->magic == MAGIC && f->version == VERSION) {
    memcpy(cfg, f, sizeof(*cfg));
    return true;
  }
  init_cfg(cfg);
  return false;
}


void cfg_save(const pad_cfg* cfg) {
  // temporary buffer
  uint8_t page[FLASH_PAGE_SIZE] = {0};
  memcpy(page, cfg, sizeof(*cfg));

  uint32_t irq = save_and_disable_interrupts();
  flash_range_erase(NVM_OFFSET, FLASH_SECTOR_SIZE);
  flash_range_program(NVM_OFFSET, page, FLASH_PAGE_SIZE);
  restore_interrupts(irq);
}


static void oled_write_cmd(uint8_t cmd) {
  uint8_t buf[2] = {0x00, cmd};
  i2c_write_blocking(I2C_INST, OLED_ADDR, buf, 2, false);
}


static void oled_write_data(const uint8_t *data, uint16_t len) {
  while (len) {
    uint16_t n = len > 128 ? 128 : len; // safe chunk
    uint8_t buf[1 + 128];
    buf[0] = 0x40;                      // Co=0, D/C#=1 => data stream
    memcpy(&buf[1], data, n);
    i2c_write_blocking(I2C_INST, OLED_ADDR, buf, n + 1, false);
    data += n;
    len  -= n;
  }
}


void oled_init(void) {
    i2c_init(I2C_INST, 400 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    sleep_ms(10);
    oled_write_cmd(0xAE);                       // display off
    oled_write_cmd(0xD5); oled_write_cmd(0x80); // clk div
    oled_write_cmd(0xA8); oled_write_cmd(HEIGHT == 64 ? 0x3F : 0x1F); // multiplex
    oled_write_cmd(0xD3); oled_write_cmd(0x00); // display offset
    oled_write_cmd(0x40);                       // start line
    oled_write_cmd(0x8D); oled_write_cmd(0x14); // charge pump on
    oled_write_cmd(0x20); oled_write_cmd(0x02); // page addressing mode
    oled_write_cmd(0xA1);                       // segment remap
    oled_write_cmd(0xC8);                       // COM scan dec
    oled_write_cmd(0xDA); oled_write_cmd(HEIGHT == 64 ? 0x12 : 0x02); // compins
    oled_write_cmd(0x81); oled_write_cmd(0x8F); // contrast
    oled_write_cmd(0xD9); oled_write_cmd(0xF1); // precharge
    oled_write_cmd(0xDB); oled_write_cmd(0x40); // vcomh
    oled_write_cmd(0xA4);                       // display follows RAM
    oled_write_cmd(0xA6);                       // normal display
    oled_write_cmd(0xAF);                       // display on
}


static void oled_set_cursor(uint8_t page, uint8_t col) {
    // page: 0..3 for 32px tall; 0..7 for 64px tall
    oled_write_cmd(0xB0 | (page & 0x07));
    oled_write_cmd(0x00 | (col & 0x0F)); // lower col nibble
    oled_write_cmd(0x10 | (col >> 4));   // upper col nibble
}


void oled_clear(void) {
  static const uint8_t Z[1024] = {0};

  // set column range 0 -> 127
  oled_write_cmd(0x21);
  oled_write_cmd(0x00);
  oled_write_cmd(0x7F);

  // set page range 0 -> 7
  oled_write_cmd(0x22);
  oled_write_cmd(0x00);
  oled_write_cmd(0x07);

  // horizontal addressing mode
  oled_write_cmd(0x20);
  oled_write_cmd(0x00);

  // send all zeros at once
  oled_write_data(Z, 1024);

  // restore page addressing mode
  oled_write_cmd(0x20);
  oled_write_cmd(0x02);
}


// Each character is 5 columns plus one blank column.
// glyph[] bytes are vertical columns, bit0 = top pixel.
static void oled_putc(uint8_t page, uint8_t *col, char c) {
    if (c < 32 || c > 127) c = '?';
    oled_set_cursor(page, *col);
    oled_write_data(FONT5x7[c - 32], 5);
    uint8_t space = 0x00;
    oled_write_data(&space, 1);
    *col += 6;
}
void oled_print(uint8_t page, uint8_t col, const char *s) {
    while (*s && col + 6 <= WIDTH) oled_putc(page, &col, *s++);
}


void oled_sleep(bool enable) {
  if (enable) {
    // display OFF (sleep)
    oled_write_cmd(0xAE);
    // charge pump OFF
    oled_write_cmd(0x8D);
    oled_write_cmd(0x10);
  } else {
    // charge pump ON
    oled_write_cmd(0x8D);
    oled_write_cmd(0x14);
    // display ON
    oled_write_cmd(0xAF);
  }
}


void oled_blit_img(const uint8_t* img) {
  // cols 0 -> 127
  oled_write_cmd(0x21);
  oled_write_cmd(0x00);
  oled_write_cmd(0x7F);
  // pages 0 -> 7
  oled_write_cmd(0x22);
  oled_write_cmd(0x00);
  oled_write_cmd(0x07);
  // horizontal addressing mode
  oled_write_cmd(0x20);
  oled_write_cmd(0x00);

  // send all pixels
  oled_write_data(img, 128 * (64/8));  // 1024 bytes

  // restore page addressing for text
  oled_write_cmd(0x20); oled_write_cmd(0x02);
}


void oled_anim_init(oled_anim* a,
                    const uint8_t frames[][1024],
                    uint16_t frame_n,
                    uint32_t frame_ms) {
  a->frames = frames;
  a->frame_n = frame_n;
  a->frame_i = 0;
  a->frame_sent = false;
  a->frame_ms = frame_ms;
  a->start = get_absolute_time();
}


void oled_anim_tick(oled_anim* a) {
  if (!a->frame_sent) {
    // cols 0 -> 127
    oled_write_cmd(0x21);
    oled_write_cmd(0x00);
    oled_write_cmd(0x7F);
    // pages 0 -> 7
    oled_write_cmd(0x22);
    oled_write_cmd(0x00);
    oled_write_cmd(0x07);
    // horizontal addressing mode
    oled_write_cmd(0x20);
    oled_write_cmd(0x00);

    // send frame
    const uint8_t* frame = a->frames[a->frame_i];
    oled_write_data(frame, 1024);

    // restore page mode
    oled_write_cmd(0x20);
    oled_write_cmd(0x02);

    a->frame_sent = true;
  }

  // re-init once frame done
  uint64_t us = absolute_time_diff_us(a->start, get_absolute_time());
  if (us >= (uint64_t)a->frame_ms * 1000ull) {
    a->frame_i = (a->frame_i + 1) % a->frame_n;
    a->start = get_absolute_time();
    a->frame_sent = false;
  }
}


void oled_play_anim(const uint8_t frames[][1024],
                    uint16_t count,
                    uint32_t frame_ms,
                    uint16_t loops) {
  for (uint16_t L = 0; loops == 0 || L < loops; ++L) {
    for (uint16_t i = 0; i < count; ++i) {
      oled_blit_img(frames[i]);
      sleep_ms(frame_ms);
    }
  }
}


void init_btns() {
  // enable inputs w/ pull ups
  for (uint i = 0; i < BTN_COUNT; ++i) {
    uint p = BTN_CFG[i].pin;
    gpio_init(p);
    gpio_pull_up(p);
    gpio_set_dir(p, GPIO_IN);
  }
}


// checks button state, filtered via rising edge debouncing.
static bool is_pressed(uint pin, int idx) {
  static bool debounced[BTN_COUNT] = {0};
  // next time a press is allowed
  static absolute_time_t lock_until[BTN_COUNT];

  bool raw = (gpio_get(pin) == 0); // active low
  absolute_time_t now = get_absolute_time();

  bool lock_expired = absolute_time_diff_us(now, lock_until[idx]) <= 0;

  if (lock_expired) {
    // accept first input
    if (!debounced[idx] && raw) {
      debounced[idx] = true;
      lock_until[idx] = delayed_by_us(now, BTN_CFG[idx].debounce);
    } else if (debounced[idx] && !raw) { // release
      debounced[idx] = false;
      lock_until[idx] = delayed_by_us(now, BTN_CFG[idx].debounce);
    }
  }

  return debounced[idx];
}


static void build_play_frame(led_frame* f, const bool* pressed) {
  for (uint i = 0; i < LED_BTN_COUNT; ++i) {
    uint8_t r = BTN_CFG[i].r;
    uint8_t g = BTN_CFG[i].g;
    uint8_t b = BTN_CFG[i].b;
    if (pressed[i]) {
      r = BTN_CFG[i].rp;
      g = BTN_CFG[i].gp;
      b = BTN_CFG[i].bp;
    }
    f->rgb[i][0] = r;
    f->rgb[i][1] = g;
    f->rgb[i][2] = b;
  }
}


static void build_turbo_frame(led_frame* f) {
  for (uint i = 0; i < LED_BTN_COUNT; ++i) {
    if (BTN_CFG[i].turbo == TURBO_0HZ) {
      f->rgb[i][0] = 0; // r
      f->rgb[i][1] = 0; // g
      f->rgb[i][2] = 0; // b
    } else if (BTN_CFG[i].turbo == TURBO_15HZ) {
      f->rgb[i][0] = 255;
      f->rgb[i][1] = 150;
      f->rgb[i][2] = 0;
    } else if (BTN_CFG[i].turbo == TURBO_30HZ) {
      f->rgb[i][0] = 255;
      f->rgb[i][1] = 80;
      f->rgb[i][2] = 0;
    } else if (BTN_CFG[i].turbo == TURBO_60HZ) {
      f->rgb[i][0] = 255;
      f->rgb[i][1] = 0;
      f->rgb[i][2] = 0;
    }
  }
}


static void act_bootsel() {
  reset_usb_boot(0, 0);
}


static void act_led_toggle() {
  multicore_fifo_push_timeout_us(CMD_TOGGLE_LEDS, 0);
}


static void act_oled_toggle() {
  multicore_fifo_push_timeout_us(CMD_TOGGLE_OLED, 0);
}


static void act_turbo() {
  CSTATE = CFG_TURBO;
}


static const combo combos[] = {
  {(1 << B8) | (1 << B9), 0,       act_bootsel},
  {(1 << B13),            500,     act_led_toggle},
  {(1 << B14),            500,     act_oled_toggle},
  {(1 << TURBO),          500,     act_turbo},
};


static void read_button_combos(uint32_t bmask) {
  static timer t = {0};

  for (size_t i = 0; i < (sizeof combos / sizeof combos[0]); ++i) {
    uint32_t btns = combos[i].btns;

    if (bmask == btns) {
      // arm timer only if we were not tracking this combo this will
      // overwrite any existing timers which did not reach expiry
      if (!t.active || t.btns != btns) {
        t.active = true;
        t.triggered = false;
        t.btns = btns;
        t.expiry = delayed_by_us(get_absolute_time(), combos[i].duration);
      }
      // execute action
      if (!t.triggered && get_absolute_time() > t.expiry) {
        combos[i].action();
        t.triggered = true;
      }
    } else {
      // disable timer if we release
      if (t.btns == btns) {
        t.active = false;
        t.triggered = false;
        t.btns = 0;
      }
    }
  }
}


static uint32_t read_buttons(bool* pressed_led, uint32_t* raw_bits) {
  static dpad_state dpad = {0};
  static bool held[BTN_COUNT] = {0};
  uint32_t bmask = 0;

  for (uint i = 0; i < BTN_COUNT; ++i) {
    const uint bit = BTN_CFG[i].bit;
    const uint p   = BTN_CFG[i].pin;
    bool pressed = is_pressed(BTN_CFG[i].pin, i);

    if (pressed) {
      bmask |= (1u << BTN_CFG[i].bit);
      if (i < LED_BTN_COUNT) pressed_led[i] = true;

      // timestamp when pressed
      if (bit == LEFT)  {if (dpad.l == 0) dpad.l = ++dpad.time;}
      if (bit == RIGHT) {if (dpad.r == 0) dpad.r = ++dpad.time;}
      if (bit == UP)    {if (dpad.u == 0) dpad.u = ++dpad.time;}
      if (bit == DOWN)  {if (dpad.d == 0) dpad.d = ++dpad.time;}

    } else {
      if (bit == LEFT)  dpad.l = 0;
      if (bit == RIGHT) dpad.r = 0;
      if (bit == UP)    dpad.u = 0;
      if (bit == DOWN)  dpad.d = 0;
      if (p == LED_TOGGLE_PIN)  held[i] = false;
      if (p == OLED_TOGGLE_PIN) held[i] = false;
    }
  }

  // check button combos
  read_button_combos(bmask);

  // SOCD cleaning
  if (SOCD == LAST_INPUT) {
    if (dpad.l && dpad.r) {
      if (dpad.l < dpad.r) bmask &= ~(1u << LEFT);
      else bmask &= ~(1u << RIGHT);
    }
    if (dpad.u && dpad.d) {
      if (dpad.u < dpad.d) bmask &= ~(1u << UP);
      else bmask &= ~(1u << DOWN);
    }
  } else if (SOCD == NEUTRAL) {
    if (dpad.l && dpad.r) {
      bmask &= ~(1u << LEFT);
      bmask &= ~(1u << RIGHT);
    }
    if (dpad.u && dpad.d) {
      bmask &= ~(1u << UP);
      bmask &= ~(1u << DOWN);
    }
  }

  // disable LEDs to reflect SOCD
  for (uint i = 0; i < LED_BTN_COUNT; ++i) {
    pressed_led[i] = ((bmask >> BTN_CFG[i].bit) & 1u) != 0;
  }

  // send raw bits out for used by other states like CFG_TURBO
  if (raw_bits) *raw_bits = bmask;


  // get hat bits
  uint32_t dir = bmask & ((1u << UP) |
                          (1u << RIGHT) |
                          (1u << DOWN) |
                          (1u << LEFT));
  uint8_t hat = 0x0F; // neutral (Null)
  bool up    = dir & (1u << UP);
  bool right = dir & (1u << RIGHT);
  bool down  = dir & (1u << DOWN);
  bool left  = dir & (1u << LEFT);
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
  return bits;
}


// core0 writes, core1 reads
static queue_t ledq;


static void set_leds(PIO pio, uint sm, led_frame* f) {
  for (uint i = 0; i < LED_BTN_COUNT; ++i) {
    uint32_t color = (f->rgb[i][1] << 16 | // G
                      f->rgb[i][0] << 8  | // R
                      f->rgb[i][2]);       // B
    pio_sm_put_blocking(pio, sm, color << 8);
  }
  sleep_us(LATCH_TIME);
}


static void process_turbo(uint32_t* bits) {
  uint32_t now_ms = to_ms_since_boot(get_absolute_time());
  for (uint i = 0; i < LED_BTN_COUNT; ++i) {
    TURBO_RATE rate = BTN_CFG[i].turbo;
    // skip buttons with turbo disabled
    if (rate == TURBO_0HZ) continue;
    uint32_t mask = 1u << BTN_CFG[i].bit;
    // skip buttons which are not pressed
    if ((*bits & mask) == 0) continue;
    // is turbo "on" at this moment?

    bool active = false;
    switch (rate) {
    case TURBO_15HZ: active = ((now_ms >> 5) & 1u) == 0; break; // period 64ms, ~15.6 Hz
    case TURBO_30HZ: active = ((now_ms >> 4) & 1u) == 0; break; // period 32ms, ~31.25 Hz
    case TURBO_60HZ: active = ((now_ms >> 3) & 1u) == 0; break; // period 16 ms, ~62.5 Hz
    case TURBO_0HZ:
    default: active = true;
    }

    if (active) {
      *bits |= mask;
    } else {
      *bits &= ~mask;
    }
  }
}


static void core1_main() {
  CTRL_STATE prev_cstate = CSTATE;
  bool leds_on = true;
  bool oled_on = true;
  led_frame fcache = {0};
  led_frame null_frame = {0};

  // init oled
  oled_init();
  oled_clear();
  oled_blit_img(IMG128x64);

  static oled_anim anim;
  oled_anim_init(&anim, ANIM_FRAMES, ANIM_NUM_FRAMES, ANIM_FRAME_MS);

  // init LEDs
  PIO pio = pio0;
  uint sm = 0;
  uint offset = pio_add_program(pio, &ws2812_program);
  ws2812_program_init(pio, sm, offset, LED_PIN, 800000, false);

  led_frame f;
  while (true) {
    if (CSTATE == PLAY) {
      // clear OLED on state change
      if (prev_cstate != PLAY) {
        oled_clear();
        set_leds(pio, sm, &fcache);
        prev_cstate = PLAY;
      }

      // update oled
      oled_anim_tick(&anim);

      // drain FIFO
      while (multicore_fifo_rvalid()) {
        uint32_t cmd = multicore_fifo_pop_blocking();
        if (cmd == CMD_TOGGLE_OLED) {
          oled_sleep(oled_on);
          oled_on = !oled_on;
        } else if (cmd == CMD_TOGGLE_LEDS) {
          leds_on = !leds_on;
          // shut off LEDs
          if (!leds_on) {
            set_leds(pio, sm, &null_frame);
          } else {
            // restore from cache
            set_leds(pio, sm, &fcache);
          }
        }
      }

      if (queue_try_remove(&ledq, &f)) {
        // cache the frame
        fcache = f;
        if (!leds_on) {
          set_leds(pio, sm, &null_frame);
        } else {
          set_leds(pio, sm, &f);
        }
      }
    } else if (CSTATE == CFG_TURBO) {
      build_turbo_frame(&f);
      set_leds(pio, sm, &f);

      // clear OLED on state change
      if (prev_cstate != CFG_TURBO) {
        oled_clear();
        prev_cstate = CFG_TURBO;
      }
      oled_print(0, 0, "- TURBO CONFIG MODE -");
      oled_print(1, 0, "");
      oled_print(2, 0, "RED LED    = 60Hz");
      oled_print(3, 0, "ORANGE LED = 30Hz");
      oled_print(4, 0, "YELLOW LED = 15Hz");
      oled_print(5, 0, "OFF LED    = OFF");
    }
  }
}


int main() {
  board_init();
  tusb_init();
  init_btns();

  // load config from flash
  cfg_load(&cfg);

  queue_init(&ledq, sizeof(led_frame), 4);
  multicore_launch_core1(core1_main);

  bool pressed_led[LED_BTN_COUNT] = {0};
  uint32_t prev_bits = 0;
  led_frame last_sent = {0};

  bool was_pressed[BTN_COUNT] = {0};

  while (true) {
    tud_task();

    memset(pressed_led, 0, sizeof(pressed_led));
    uint32_t raw_bits;
    uint32_t bits = read_buttons(pressed_led, &raw_bits);

    if (CSTATE == PLAY) {
      process_turbo(&bits);

      // send HID report
      if (tud_hid_ready() && bits != prev_bits) {
        gamepad_report rpt = {.buttons = bits};
        tud_hid_report(0, &rpt, sizeof(rpt));
        prev_bits = bits;
      }

      // enqueue LED frame only if LEDs changed
      led_frame f;
      build_play_frame(&f, pressed_led);

      // if this frame different from last
      if (memcmp(&f, &last_sent, sizeof(f)) != 0) {
        // try to enqueue
        if (!queue_try_add(&ledq, &f)) {
          led_frame scratch;
          // free one slot
          queue_try_remove(&ledq, &scratch);
          queue_try_add(&ledq, &f);
        }
        last_sent = f;
      }
    } else if (CSTATE == CFG_TURBO) {
      for (uint i = 0; i < LED_BTN_COUNT; ++i) {
        bool pressed = (raw_bits >> BTN_CFG[i].bit) & 1u;
        if (pressed && !was_pressed[i]) {
          BTN_CFG[i].turbo = (BTN_CFG[i].turbo + 1) % 4;
        }
        was_pressed[i] = pressed;
      }
      // check if TURBO was released
      if (!(raw_bits & (1 << TURBO))) {
        CSTATE = PLAY;
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
