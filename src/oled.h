#pragma once

#include <stdint.h>

typedef struct {
  const uint8_t (*frames)[1024];
  uint16_t frame_n;  // frame count
  uint16_t frame_i;  // frame index
  uint8_t page;      // 0 -> 7
  uint8_t col;       // 0 -> 127
  uint32_t frame_ms; // frame time
  bool frame_sent;   // has the frame been sent
  absolute_time_t start; // frame start
} oled_anim_t;

void oled_init(void);
void oled_clear(void);
void oled_print(uint8_t page, uint8_t col, const char *s);
void oled_sleep(bool enable);
void oled_blit_img(const uint8_t* img);
void oled_anim_init(oled_anim_t* a,
                    const uint8_t frames[][1024],
                    uint16_t frame_n,
                    uint32_t frame_ms);
void oled_anim_tick(oled_anim_t* a);
void oled_play_anim(const uint8_t frames[][1024],
                    uint16_t count,
                    uint32_t frame_ms,
                    uint16_t loops);
