#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"
#include "pico/bootrom.h"
#include "pico/unique_id.h"
#include "hardware/flash.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/sync.h"
#include "ws2812.pio.h"
#include "bsp/board.h"
#include "tusb.h"
#include "device/usbd_pvt.h"
#include "xsm3/xsm3.h"

#include "font/font5x7.h"


// key config
#define BTN_COUNT       21
#define LED_BTN_COUNT   16
#define LED_PIN         28
#define LED_TOGGLE_PIN  21
#define OLED_TOGGLE_PIN 20
#define BOOTSEL_PIN     14
#define LATCH_TIME      350  // us
#define TAC_DEBOUNCE    8000 // us
#define KEY_DEBOUNCE    8000 // us


// HID metadata
#define VENDOR_ID        0xBEEF
#define PRODUCT_ID       0xFEEF
#define MANUFACTURER     "SEGV"
#define PRODUCT          "BEEFPAD"
#define SERIAL_NUM       "00000000"


// oled
#define I2C_INST    i2c0
#define I2C_SDA_PIN 0
#define I2C_SCL_PIN 1
#define OLED_ADDR   0x3C
#define WIDTH       128
#define HEIGHT      64
#define OLED_IMG    "sega.h"
#define OLED_CHUNK  128


// settings
#define NVM_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)
#define MAGIC      0xBEEFBABE
#define VERSION    1


volatile uint32_t core1_cmd_mask = 0;

// Dummy OLED image/animation data (placeholders)
static const uint8_t IMG128x64[1024] = {0};
static const uint8_t ANIM_FRAMES[1][1024] = {{0}};
#define ANIM_NUM_FRAMES 1
#define ANIM_FRAME_MS 1000


enum {
  CORE1_CMD_TOGGLE_OLED = 1u << 0,
  CORE1_CMD_TOGGLE_LEDS = 1u << 1,
};


typedef enum {
  USB_MODE_HID = 0,
  USB_MODE_XINPUT = 1,
} USB_MODE;


// Xbox 360 Authentication Request Types
typedef enum {
    XSM360_GET_SERIAL         = 0x81,
    XSM360_INIT_AUTH          = 0x82,
    XSM360_RESPOND_CHALLENGE  = 0x83,
    XSM360_AUTH_KEEPALIVE     = 0x84,
    XSM360_REQUEST_STATE      = 0x86,
    XSM360_VERIFY_AUTH        = 0x87,
} XSM360AuthRequest;

#define X360_AUTHLEN_CONSOLE_INIT  34
#define X360_AUTHLEN_DONGLE_SERIAL 29
#define X360_AUTHLEN_DONGLE_INIT   46
#define X360_AUTHLEN_CHALLENGE     22

typedef enum {
    AUTH_IDLE_STATE = 0,
    AUTH_PROCESSING = 1,
    AUTH_READY = 2,
} AUTH_STATE;

typedef struct {
    AUTH_STATE state;
    uint8_t init_buffer[X360_AUTHLEN_DONGLE_INIT];
    uint8_t challenge_buffer[X360_AUTHLEN_CHALLENGE];
    uint8_t serial_buffer[X360_AUTHLEN_DONGLE_SERIAL];
    bool initialized;
} xinput_auth;


typedef enum {
  PLAY,
  CFG_TURBO,
} CTRL_STATE;
CTRL_STATE CSTATE = PLAY;


typedef enum {
  OLED_MODE_TEXT,
  OLED_MODE_IMG,
  OLED_MODE_ANIM,
  OLED_MODE_OFF,
} OLED_MODE;
OLED_MODE oled_mode = OLED_MODE_TEXT;


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


typedef struct __attribute__((packed)) { // force 1B alignment of members
  uint32_t magic;
  uint16_t version;
  uint8_t  enable_leds;
  uint8_t  oled_mode;
  uint8_t  usb_mode;
  // padding accounting for each of the above members
  uint8_t  _pad[FLASH_PAGE_SIZE - 4 - 2 - 1 - 1 - 1];
} pad_cfg;


pad_cfg cfg;
bool leds_on = true;
USB_MODE usb_mode = USB_MODE_HID;
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


typedef struct {
  uint32_t rgb[LED_BTN_COUNT][3];
} led_frame;


typedef struct __attribute__((packed)) {
  // [0 ..20] -> buttons
  // [24..21] -> hat
  uint32_t buttons;
} gamepad_report;


// XINPUT
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


xinput_report xinput_data;
static uint8_t xinput_endpoint_in = 0;
static uint8_t xinput_endpoint_out = 0;
static xinput_auth xauth = {0};
static uint8_t vendor_buffer[64] = {0};
static uint8_t xinput_send_count = 0;
static uint8_t xinput_out_buffer[32] = {0};
static xinput_report last_xinput_data = {0};


static void send_xinput_report(uint32_t bits) {
  // Set report size
  xinput_data.rsize = 20;

  // Map bits to XInput format
  xinput_data.digital_buttons_1 = 0;
  xinput_data.digital_buttons_2 = 0;

  if (bits & (1 << UP))    xinput_data.digital_buttons_1 |= 0x01; // UP
  if (bits & (1 << DOWN))  xinput_data.digital_buttons_1 |= 0x02; // DOWN
  if (bits & (1 << LEFT))  xinput_data.digital_buttons_1 |= 0x04; // LEFT
  if (bits & (1 << RIGHT)) xinput_data.digital_buttons_1 |= 0x08; // RIGHT

  if (bits & (1 << B1))    xinput_data.digital_buttons_1 |= 0x10; // START
  if (bits & (1 << B11))   xinput_data.digital_buttons_1 |= 0x10; // START
  if (bits & (1 << B16))   xinput_data.digital_buttons_1 |= 0x10; // START

  if (bits & (1 << B10))   xinput_data.digital_buttons_1 |= 0x20; // BACK
  if (bits & (1 << B15))   xinput_data.digital_buttons_1 |= 0x20; // BACK

  // if (bits & (1 << B15))   xinput_data.digital_buttons_1 |= 0x40; // L3
  // if (bits & (1 << B16))   xinput_data.digital_buttons_1 |= 0x80; // R3

  if (bits & (1 << B5))  xinput_data.digital_buttons_2 |= 0x01; // LB
  if (bits & (1 << B4))  xinput_data.digital_buttons_2 |= 0x02; // RB
  if (bits & (1 << B9))  xinput_data.digital_buttons_2 |= 0x04; // GUIDE

  if (bits & (1 << B1))  xinput_data.digital_buttons_2 |= 0x10; // A
  if (bits & (1 << B6))  xinput_data.digital_buttons_2 |= 0x10; // A
  if (bits & (1 << B12)) xinput_data.digital_buttons_2 |= 0x10; // A

  if (bits & (1 << B7))  xinput_data.digital_buttons_2 |= 0x20; // B

  if (bits & (1 << B2))  xinput_data.digital_buttons_2 |= 0x40; // X
  if (bits & (1 << B3))  xinput_data.digital_buttons_2 |= 0x80; // Y

  // Triggers (mapped to buttons)
  xinput_data.lt = (bits & (1 << B9)) ? 0xFF : 0x00;
  xinput_data.rt = (bits & (1 << B8)) ? 0xFF : 0x00;

  // Analog sticks (centered, could map to other buttons if needed)
  xinput_data.l_x = 0;
  xinput_data.l_y = 0;
  xinput_data.r_x = 0;
  xinput_data.r_y = 0;

  // only send if report changed
  if (memcmp(&last_xinput_data, &xinput_data, sizeof(xinput_report)) != 0) {
    if (tud_ready() && xinput_endpoint_in != 0 && !usbd_edpt_busy(0, xinput_endpoint_in)) {
      usbd_edpt_claim(0, xinput_endpoint_in);
      usbd_edpt_xfer(0, xinput_endpoint_in, (uint8_t*)&xinput_data, 20);
      usbd_edpt_release(0, xinput_endpoint_in);

      // save the report we just sent
      memcpy(&last_xinput_data, &xinput_data, sizeof(xinput_report));
      xinput_send_count++;
    }
  }
}


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
static void cfg_init() {
  memset(&cfg, 0, sizeof(cfg)); // zero all fields
  cfg.magic = MAGIC;
  cfg.version = VERSION;
  cfg.enable_leds = 1;
  cfg.oled_mode = OLED_MODE_TEXT;
  cfg.usb_mode = USB_MODE_HID;
}


bool cfg_load() {
  // return a pointer to the cfg region in XIP
  const pad_cfg* f = (const pad_cfg*)(XIP_BASE + NVM_OFFSET);
  // did we find a valid config?
  if (f->magic == MAGIC && f->version == VERSION) {
    memcpy(&cfg, f, sizeof(cfg));
    return true;
  }
  // set defaults
  cfg_init();
  return false;
}


// apply the settings from cfg
void cfg_apply() {
  leds_on = cfg.enable_leds;
  oled_mode = cfg.oled_mode;
  usb_mode = cfg.usb_mode;
}


void cfg_save(void) {
  uint8_t page[FLASH_PAGE_SIZE] = {0};
  memcpy(page, &cfg, sizeof(cfg));

  // block core1 from running code from flash
  multicore_lockout_start_blocking();

  uint32_t irq = save_and_disable_interrupts();
  flash_range_erase(NVM_OFFSET, FLASH_SECTOR_SIZE);
  flash_range_program(NVM_OFFSET, page, FLASH_PAGE_SIZE);
  restore_interrupts(irq);

  // unblock core1
  multicore_lockout_end_blocking();
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
  core1_cmd_mask |= CORE1_CMD_TOGGLE_LEDS;
  cfg.enable_leds ^= 1;
}


static void act_oled_toggle() {
  core1_cmd_mask |= CORE1_CMD_TOGGLE_OLED;
  cfg.oled_mode = (cfg.oled_mode + 1) % 4;
}


static void act_turbo() {
  CSTATE = (CSTATE == PLAY) ? CFG_TURBO : PLAY;
}


static void act_usb_mode_toggle() {
  cfg.usb_mode = (cfg.usb_mode == USB_MODE_HID) ? USB_MODE_XINPUT : USB_MODE_HID;
  cfg_save();
}


static const combo combos[] = {
  {(1 << B8)    | (1 << B9),  0,       act_bootsel},
  {(1 << TURBO) | (1 << B13), 1000000, act_led_toggle},
  {(1 << TURBO) | (1 << B14), 1000000, act_oled_toggle},
  {(1 << TURBO) | (1 << B15), 1000000, act_usb_mode_toggle},
  {(1 << TURBO) | (1 << B16), 1000000, cfg_save},
  {(1 << TURBO),              1000000, act_turbo},
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


static void oled_draw_hud() {
  if (usb_mode == USB_MODE_HID) {
    oled_print(0, 0, "HID");
  } else if (usb_mode == USB_MODE_XINPUT) {
    oled_print(0, 0, "XINPUT");
    if (xauth.initialized) {
      if (xauth.state == AUTH_IDLE_STATE) {
        oled_print(1, 0, "AUTH: IDLE");
      } else if (xauth.state == AUTH_PROCESSING) {
        oled_print(2, 0, "AUTH: PROC");
      } else if (xauth.state == AUTH_READY) {
        oled_print(3, 0, "AUTH: OK");
      }
    }
  }
}


static void core1_main() {
  multicore_lockout_victim_init();

  CTRL_STATE prev_cstate = CSTATE;
  led_frame fcache = {0};
  led_frame null_frame = {0};

  // init OLED
  oled_init();
  if (oled_mode == OLED_MODE_OFF) {
    oled_sleep(true);
  } else {
    oled_clear();
  }

  static oled_anim anim;
  oled_anim_init(&anim, ANIM_FRAMES, ANIM_NUM_FRAMES, ANIM_FRAME_MS);

  // init LEDs
  PIO pio = pio0;
  uint sm = 0;
  uint offset = pio_add_program(pio, &ws2812_program);
  ws2812_program_init(pio, sm, offset, LED_PIN, 800000, false);

  led_frame f;
  static OLED_MODE prev_oled_mode = OLED_MODE_TEXT;
  while (true) {
    if (CSTATE == PLAY) {
      // restore LED/OLED on CSTATE change
      if (prev_cstate != PLAY) {
        oled_clear();
        if (oled_mode == OLED_MODE_OFF) {
          oled_sleep(true);
        } else {
          oled_sleep(false);
        }

        if (leds_on) {
          set_leds(pio, sm, &fcache);
        } else {
          set_leds(pio, sm, &null_frame);
        }
        prev_cstate = PLAY;
      }

      // handle oled mode state changes
      if (oled_mode != prev_oled_mode) {
        if (oled_mode == OLED_MODE_OFF) {
          oled_sleep(true);
        } else {
          oled_sleep(false);
          oled_clear();
        }
        prev_oled_mode = oled_mode;
      }

      // update oled based on mode
      if (oled_mode == OLED_MODE_TEXT) {
        oled_draw_hud();
      } else if (oled_mode == OLED_MODE_IMG) {
        oled_blit_img(IMG128x64);
      } else if (oled_mode == OLED_MODE_ANIM) {
        oled_anim_tick(&anim);
      }

      // process commands from core0
      uint32_t cmd = core1_cmd_mask;
      if (cmd & CORE1_CMD_TOGGLE_OLED) {
        core1_cmd_mask &= ~CORE1_CMD_TOGGLE_OLED;
        oled_mode = (oled_mode + 1) % 4;
      }
      if (cmd & CORE1_CMD_TOGGLE_LEDS) {
        core1_cmd_mask &= ~CORE1_CMD_TOGGLE_LEDS;
        leds_on = !leds_on;
        if (!leds_on) {
          set_leds(pio, sm, &null_frame);
        } else {
          set_leds(pio, sm, &fcache);
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

      // clear OLED and force it on
      if (prev_cstate != CFG_TURBO) {
        oled_clear();
        prev_cstate = CFG_TURBO;
        oled_sleep(false);
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
  init_btns();

  // load config from flash
  cfg_load();
  cfg_apply();

  tusb_init();

  queue_init(&ledq, sizeof(led_frame), 4);
  multicore_launch_core1(core1_main);

  bool pressed_led[LED_BTN_COUNT] = {0};
  uint32_t prev_bits = 0;
  led_frame last_sent = {0};

  bool was_pressed[BTN_COUNT] = {0};

  while (true) {
    memset(pressed_led, 0, sizeof(pressed_led));
    uint32_t raw_bits;
    uint32_t bits = read_buttons(pressed_led, &raw_bits);

    if (CSTATE == PLAY) {
      process_turbo(&bits);

      // send report based on USB mode
      if (usb_mode == USB_MODE_XINPUT) {
        send_xinput_report(raw_bits);
      } else {
        if (bits != prev_bits || raw_bits != prev_bits) {
          if (tud_hid_ready()) {
            gamepad_report rpt = {.buttons = bits};
            tud_hid_report(0, &rpt, sizeof(rpt));
          }
        }
      }
      prev_bits = bits;

      // queue initial OUT endpoint transfer for Xbox 360
      if (usb_mode == USB_MODE_XINPUT &&
          tud_ready() &&
          xinput_endpoint_out != 0 &&
          !usbd_edpt_busy(0, xinput_endpoint_out)) {
        usbd_edpt_claim(0, xinput_endpoint_out);
        usbd_edpt_xfer(0, xinput_endpoint_out, xinput_out_buffer, sizeof(xinput_out_buffer));
        usbd_edpt_release(0, xinput_endpoint_out);
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
    }
    tud_task();
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


/* ---- XInput Driver Implementation ---- */
static bool xinput_driver_control_request(uint8_t rhport, uint8_t stage, tusb_control_request_t const * request) {
  (void)rhport;
  (void)stage;
  (void)request;
  return true;
}

static bool xinput_driver_control_xfer(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request) {
  uint16_t len = 0;

  // Handle IN requests (device to host)
  if (request->bmRequestType_bit.direction == TUSB_DIR_IN) {
    if (stage == CONTROL_STAGE_SETUP) {
      switch (request->bRequest) {
        case XSM360_GET_SERIAL:
          if (!xauth.initialized) return false;
          memcpy(vendor_buffer, xauth.serial_buffer, X360_AUTHLEN_DONGLE_SERIAL);
          len = X360_AUTHLEN_DONGLE_SERIAL;
          break;

        case XSM360_REQUEST_STATE:
          {
            uint16_t state = (xauth.state == AUTH_READY) ? 2 : 1;
            memcpy(vendor_buffer, &state, sizeof(state));
            len = sizeof(state);
          }
          break;

        case XSM360_RESPOND_CHALLENGE:
          if (xauth.state != AUTH_READY) return false;

          // check length to determine which response to send
          if (request->wLength == X360_AUTHLEN_DONGLE_INIT) {
            // init response (46 bytes)
            memcpy(vendor_buffer, xauth.init_buffer, X360_AUTHLEN_DONGLE_INIT);
            len = X360_AUTHLEN_DONGLE_INIT;
          } else if (request->wLength == X360_AUTHLEN_CHALLENGE) {
            // verify response (22 bytes)
            memcpy(vendor_buffer, xauth.challenge_buffer, X360_AUTHLEN_CHALLENGE);
            len = X360_AUTHLEN_CHALLENGE;
          } else {
            return false;
          }
          break;

        case XSM360_AUTH_KEEPALIVE:
          len = 0;
          break;

        default:
          break;
      }
      tud_control_xfer(rhport, request, vendor_buffer, len);
    }
  }
  // Handle OUT requests (host to device)
  else if (request->bmRequestType_bit.direction == TUSB_DIR_OUT) {
    if (stage == CONTROL_STAGE_SETUP) {
      tud_control_xfer(rhport, request, vendor_buffer, request->wLength);
    }
    else if (stage == CONTROL_STAGE_DATA) {
      switch (request->bRequest) {
        case XSM360_INIT_AUTH:
          if (request->wLength == X360_AUTHLEN_CONSOLE_INIT) {
            xauth.state = AUTH_PROCESSING;
            xsm3_do_challenge_init(vendor_buffer);
            memcpy(xauth.init_buffer, xsm3_challenge_response, X360_AUTHLEN_DONGLE_INIT);
            xauth.state = AUTH_READY;
          }
          break;

        case XSM360_VERIFY_AUTH:
          if (request->wLength == X360_AUTHLEN_CHALLENGE) {
            xauth.state = AUTH_PROCESSING;
            xsm3_do_challenge_verify(vendor_buffer);
            memcpy(xauth.challenge_buffer, xsm3_challenge_response, X360_AUTHLEN_CHALLENGE);
            xauth.state = AUTH_READY;
          }
          break;
      }
    }
  }

  return true;
}

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request) {
  // in XINPUT mode, handle vendor requests for Xbox 360 auth
  if (usb_mode == USB_MODE_XINPUT) {
    return xinput_driver_control_xfer(rhport, stage, request);
  }
  return false;
}


static void xinput_driver_init(void) {
  memset(&xinput_data, 0, sizeof(xinput_data));
  xinput_data.rid = 0x00;
  xinput_data.rsize = 0x14;

  // Initialize XSM3 authentication
  if (usb_mode == USB_MODE_XINPUT) {
    // Generate serial from Pico unique ID
    uint8_t serial[0x0C];
    pico_unique_board_id_t board_id;
    pico_get_unique_board_id(&board_id);
    for(int i = 0; i < 0x0C; i++) {
      serial[i] = 'A' + (board_id.id[i % PICO_UNIQUE_BOARD_ID_SIZE_BYTES] % 25);
    }

    // Initialize xsm3 with Microsoft controller identity
    xsm3_set_vid_pid(serial, 0x045E, 0x028E);
    xsm3_initialise_state();
    xsm3_set_identification_data(xsm3_id_data_ms_controller);

    // Copy serial to auth structure
    memcpy(xauth.serial_buffer, xsm3_id_data_ms_controller, X360_AUTHLEN_DONGLE_SERIAL);
    xauth.state = AUTH_IDLE_STATE;
    xauth.initialized = true;
  }
}

static void xinput_driver_reset(uint8_t rhport) {
  (void)rhport;
  xinput_endpoint_in = 0;
  xinput_endpoint_out = 0;
}

static uint16_t xinput_driver_open(uint8_t rhport, tusb_desc_interface_t const *itf_desc, uint16_t max_len) {
  uint16_t driver_length = 0;

  // Xbox 360 has 4 interfaces: Control (0x5D/0x01), Audio (0x5D/0x03), Plugin (0x5D/0x02), Security (0xFD/0x13)
  if (TUSB_CLASS_VENDOR_SPECIFIC == itf_desc->bInterfaceClass) {
    driver_length = sizeof(tusb_desc_interface_t) + (itf_desc->bNumEndpoints * sizeof(tusb_desc_endpoint_t));
    TU_VERIFY(max_len >= driver_length, 0);

    tusb_desc_interface_t* p_desc = (tusb_desc_interface_t*)itf_desc;

    // Control, Audio, or Plugin Module interfaces (0x5D)
    if (itf_desc->bInterfaceSubClass == 0x5D &&
        ((itf_desc->bInterfaceProtocol == 0x01) ||  // Control
         (itf_desc->bInterfaceProtocol == 0x02) ||  // Plugin Module
         (itf_desc->bInterfaceProtocol == 0x03))) { // Audio
      // get xbox 360 definition
      p_desc = (tusb_desc_interface_t*)tu_desc_next(p_desc);
      TU_VERIFY(0x21 == p_desc->bDescriptorType, 0);
      driver_length += p_desc->bLength;
      p_desc = (tusb_desc_interface_t*)tu_desc_next(p_desc);
      // control endpoints used for gamepad i/o
      if (itf_desc->bInterfaceProtocol == 0x01) {
        TU_ASSERT(usbd_open_edpt_pair(rhport,
                                      (const uint8_t*)p_desc,
                                      itf_desc->bNumEndpoints,
                                      TUSB_XFER_INTERRUPT,
                                      &xinput_endpoint_out,
                                      &xinput_endpoint_in), 0);
      }
    } else if (itf_desc->bInterfaceSubClass == 0xFD &&
               itf_desc->bInterfaceProtocol == 0x13) {
      p_desc = (tusb_desc_interface_t*)tu_desc_next(p_desc);
      TU_VERIFY(0x41 == p_desc->bDescriptorType, 0);
      driver_length += p_desc->bLength;
    }
  }

  return driver_length;
}

static bool xinput_driver_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes) {
  (void)rhport;
  (void)result;
  (void)xferred_bytes;

  // queue another transfer on OUT endpoint when data is received
  if (ep_addr == xinput_endpoint_out) {
    usbd_edpt_xfer(0, xinput_endpoint_out,
                   xinput_out_buffer,
                   sizeof(xinput_out_buffer));
  }
  return true;
}

static usbd_class_driver_t const xinput_driver = {
#if CFG_TUSB_DEBUG >= 2
  .name = "XINPUT",
#endif
  .init = xinput_driver_init,
  .reset = xinput_driver_reset,
  .open = xinput_driver_open,
  .control_xfer_cb = xinput_driver_control_request,
  .xfer_cb = xinput_driver_xfer_cb,
  .sof = NULL
};

usbd_class_driver_t const *usbd_app_driver_get_cb(uint8_t *driver_count) {
  if (usb_mode == USB_MODE_XINPUT) {
    *driver_count = 1;
    return &xinput_driver;
  }
  *driver_count = 0;
  return NULL;
}


//// DESCRIPTORS
tusb_desc_device_t const desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_UNSPECIFIED,
    .bDeviceSubClass = 0x00,
    .bDeviceProtocol = 0x00,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor  = VENDOR_ID,
    .idProduct = PRODUCT_ID,
    .bcdDevice = 0x0100,
    .iManufacturer = 0x01,
    .iProduct      = 0x02,
    .iSerialNumber = 0x03,
    .bNumConfigurations = 0x01
};


tusb_desc_device_t const desc_device_xinput = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = 0xFF,
    .bDeviceSubClass = 0xFF,
    .bDeviceProtocol = 0xFF,
    .bMaxPacketSize0 = 0x40,
    .idVendor  = 0x045E,
    .idProduct = 0x028E,
    .bcdDevice = 0x0114,
    .iManufacturer = 0x01,
    .iProduct      = 0x02,
    .iSerialNumber = 0x03,
    .bNumConfigurations = 0x01
};


uint8_t const * tud_descriptor_device_cb(void) {
  if (usb_mode == USB_MODE_XINPUT) {
    return (uint8_t const *) &desc_device_xinput;
  }
  return (uint8_t const *) &desc_device;
}

// HID Report Descriptor
// 8 buttons, no axes, 2 bytes total (byte2 = padding for future use)
uint8_t const desc_hid_report[] = {
  0x05,0x01, // Generic Desktop
  0x09,0x05, // Game Pad
  0xA1,0x01, // Application

  // 21 button bits
  0x05,0x09, // Button
  0x19,0x01, // Button 1
  0x29,0x15, // Button 21
  0x15,0x00, // Logical Min 0
  0x25,0x01, // Logical Max 1
  0x95,0x15, // Report Count: 21 buttons
  0x75,0x01, // Report Size: 1 bit
  0x81,0x02, // Input (Data,Var,Abs)

  // hat swich (4 bits w/ null)
  0x05,0x01, // Generic Desktop
  0x09,0x39, // Hat switch
  0x15,0x00, // Logical Min 0
  0x25,0x07, // Logical Max 7
  0x75,0x04, // Report Size 4
  0x95,0x01, // Report Count 1
  0x81,0x42, // Input (Data,Var,Abs,Null)

  // pad to 32 bits
  0x95,0x07, // 7 bits padding
  0x75,0x01,
  0x81,0x03, // Input (Const,Var,Abs)
  0xC0
};


uint8_t const * tud_hid_descriptor_report_cb(uint8_t itf) {
  (void) itf;
  if (usb_mode == USB_MODE_XINPUT) {
    return NULL;
  }
  return desc_hid_report;
}

// Configuration descriptor
enum { ITF_NUM_HID, ITF_COUNT };
#define EPNUM_HID   0x81

#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN)

uint8_t const desc_configuration[] = {
  // Config
  TUD_CONFIG_DESCRIPTOR(1, ITF_COUNT, 0, CONFIG_TOTAL_LEN, 0x00, 100),

  // HID: 8-byte IN endpoint, 1ms interval
  TUD_HID_DESCRIPTOR(0, 0, HID_ITF_PROTOCOL_NONE, sizeof(desc_hid_report), EPNUM_HID, 8, 1),
};


// XInput configuration descriptor
const uint8_t desc_configuration_xinput[] = {
  // Configuration Descriptor
  0x09,        // bLength
  0x02,        // bDescriptorType (Configuration)
  0x99, 0x00,  // wTotalLength 153 bytes (0x99)
  0x04,        // bNumInterfaces 4 (Control, Audio, Plugin Module, Security)
  0x01,        // bConfigurationValue
  0x00,        // iConfiguration (String Index)
  0xA0,        // bmAttributes (remote wakeup)
  0xFA,        // bMaxPower 500mA

  // Interface 0: Control Interface (0xFF 0x5D 0x01)
  0x09,        // bLength
  0x04,        // bDescriptorType (Interface)
  0x00,        // bInterfaceNumber 0
  0x00,        // bAlternateSetting
  0x02,        // bNumEndpoints 2
  0xFF,        // bInterfaceClass
  0x5D,        // bInterfaceSubClass
  0x01,        // bInterfaceProtocol
  0x00,        // iInterface (String Index)

  // Gamepad Descriptor
  0x11,        // bLength
  0x21,        // bDescriptorType (HID)
  0x00, 0x01,  // bcdHID 1.10
  0x01,        // SUB_TYPE
  0x25,        // reserved2
  0x81,        // DEVICE_EPADDR_IN
  0x14,        // bMaxDataSizeIn
  0x00, 0x00, 0x00, 0x00, 0x13, // reserved3
  0x02,        // DEVICE_EPADDR_OUT
  0x08,        // bMaxDataSizeOut
  0x00, 0x00,  // reserved4

  // Report IN Endpoint 1.1
  0x07,        // bLength
  0x05,        // bDescriptorType (Endpoint)
  0x81,        // bEndpointAddress (IN/D2H)
  0x03,        // bmAttributes (Interrupt)
  0x20, 0x00,  // wMaxPacketSize 32
  0x01,        // bInterval 1

  // Report OUT Endpoint 1.2
  0x07,        // bLength
  0x05,        // bDescriptorType (Endpoint)
  0x02,        // bEndpointAddress (OUT/H2D)
  0x03,        // bmAttributes (Interrupt)
  0x20, 0x00,  // wMaxPacketSize 32
  0x08,        // bInterval 8

  // Interface 1: Audio (0xFF 0x5D 0x03)
  0x09,        // bLength
  0x04,        // bDescriptorType (Interface)
  0x01,        // bInterfaceNumber 1
  0x00,        // bAlternateSetting
  0x04,        // bNumEndpoints 4
  0xFF,        // bInterfaceClass
  0x5D,        // bInterfaceSubClass
  0x03,        // bInterfaceProtocol
  0x00,        // iInterface (String Index)

  // Audio Descriptor
  0x1B,        // bLength
  0x21,
  0x00, 0x01, 0x01, 0x01,
  0x83,        // XINPUT_MIC_IN
  0x40, 0x01,
  0x04,        // XINPUT_AUDIO_OUT
  0x20, 0x16,
  0x85,        // XINPUT_UNK_IN
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16,
  0x06,        // XINPUT_UNK_OUT
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  // Report IN Endpoint 2.1
  0x07,        // bLength
  0x05,        // bDescriptorType (Endpoint)
  0x83,        // bEndpointAddress (XINPUT_MIC_IN)
  0x03,        // bmAttributes (Interrupt)
  0x20, 0x00,  // wMaxPacketSize 32
  0x02,        // bInterval 2

  // Report OUT Endpoint 2.2
  0x07,        // bLength
  0x05,        // bDescriptorType (Endpoint)
  0x04,        // bEndpointAddress (XINPUT_AUDIO_OUT)
  0x03,        // bmAttributes (Interrupt)
  0x20, 0x00,  // wMaxPacketSize 32
  0x04,        // bInterval 4

  // Report IN Endpoint 2.3
  0x07,        // bLength
  0x05,        // bDescriptorType (Endpoint)
  0x85,        // bEndpointAddress (XINPUT_UNK_IN)
  0x03,        // bmAttributes (Interrupt)
  0x20, 0x00,  // wMaxPacketSize 32
  0x40,        // bInterval 64

  // Report OUT Endpoint 2.4
  0x07,        // bLength
  0x05,        // bDescriptorType (Endpoint)
  0x06,        // bEndpointAddress (XINPUT_UNK_OUT)
  0x03,        // bmAttributes (Interrupt)
  0x20, 0x00,  // wMaxPacketSize 32
  0x10,        // bInterval 16

  // Interface 2: Plugin Module (0xFF 0x5D 0x02)
  0x09,        // bLength
  0x04,        // bDescriptorType (Interface)
  0x02,        // bInterfaceNumber 2
  0x00,        // bAlternateSetting
  0x01,        // bNumEndpoints 1
  0xFF,        // bInterfaceClass
  0x5D,        // bInterfaceSubClass
  0x02,        // bInterfaceProtocol
  0x00,        // iInterface (String Index)

  // PluginModuleDescriptor
  0x09,        // bLength
  0x21,        // bDescriptorType
  0x00, 0x01,  // version 1.00
  0x01, 0x22,
  0x86,        // XINPUT_PLUGIN_MODULE_IN
  0x03, 0x00,

  // Report IN Endpoint 3.1
  0x07,        // bLength
  0x05,        // bDescriptorType (Endpoint)
  0x86,        // bEndpointAddress (XINPUT_PLUGIN_MODULE_IN)
  0x03,        // bmAttributes (Interrupt)
  0x20, 0x00,  // wMaxPacketSize 32
  0x10,        // bInterval 16

  // Interface 3: Security (0xFF 0xFD 0x13) ← AUTHENTICATION INTERFACE
  0x09,        // bLength
  0x04,        // bDescriptorType (Interface)
  0x03,        // bInterfaceNumber 3
  0x00,        // bAlternateSetting
  0x00,        // bNumEndpoints 0
  0xFF,        // bInterfaceClass
  0xFD,        // bInterfaceSubClass (SECURITY!)
  0x13,        // bInterfaceProtocol
  0x04,        // iInterface (String Index)

  // SecurityDescriptor (XSM3)
  0x06,        // bLength
  0x41,        // bDescriptorType (Xbox 360)
  0x00, 0x01, 0x01, 0x03
};


uint8_t const * tud_descriptor_configuration_cb(uint8_t index) {
  (void) index;
  if (usb_mode == USB_MODE_XINPUT) {
    return desc_configuration_xinput;
  }
  return desc_configuration;
}


// Strings
static char const* string_desc[] = {
  (const char[]) { 0x09, 0x04 }, // 0: LangID = English (0x0409)
  MANUFACTURER,
  PRODUCT,
  SERIAL_NUM,
};


static char const* string_desc_xinput[] = {
  (const char[]) { 0x09, 0x04 }, // 0: LangID = English (0x0409)
  "\xA9Microsoft Corporation",
  "Controller",
  SERIAL_NUM,
  "Xbox Security Method 3, Version 1.00, \xA9 2005 Microsoft Corporation. All rights reserved.",
};


static uint16_t _desc_str[96]; // Increased to hold XSM3 string (89 chars)
uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
  (void) langid;

  uint8_t chr_count;
  if (index == 0) {
    _desc_str[0] = (TUSB_DESC_STRING << 8) | (2 + 2);
    _desc_str[1] = 0x0409;
    return _desc_str;
  }

  char const** string_table = (usb_mode == USB_MODE_XINPUT) ? string_desc_xinput : string_desc;
  size_t table_size = (usb_mode == USB_MODE_XINPUT) ?
                      (sizeof(string_desc_xinput)/sizeof(string_desc_xinput[0])) :
                      (sizeof(string_desc)/sizeof(string_desc[0]));

  const char* str = (index < table_size) ? string_table[index] : NULL;
  if (!str) return NULL;

  chr_count = (uint8_t)strlen(str);
  if (chr_count > 95) chr_count = 95; // Match buffer size

  for (uint8_t i = 0; i < chr_count; i++) _desc_str[1 + i] = str[i];
  _desc_str[0] = (TUSB_DESC_STRING << 8) | (2*chr_count + 2);
  return _desc_str;
}
