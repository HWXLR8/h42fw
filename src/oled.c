#include "hardware/i2c.h"
#include "hardware/gpio.h"

#include <string.h>

#include "oled.h"
#include "config.h"
#include "font/font5x7.h"

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
  static const uint8_t Z[WIDTH] = {0}; // zero buffer
  for (uint8_t p = 0; p < (HEIGHT/8); ++p) {
    oled_set_cursor(p, 0);
    oled_write_data(Z, sizeof Z);
  }
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

void oled_anim_init(oled_anim_t* a,
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

void oled_anim_tick(oled_anim_t* a) {
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
