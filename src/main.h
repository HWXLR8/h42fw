#pragma once

typedef enum {
  NEUTRAL,    // L+R = 0 or U+D = 0
  LAST_INPUT, // last input wins
} SOCD_MODE;

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
  B13,
  B14,
  B15,
  B16,
  B17,
} BTN_BIT;

typedef struct {
  uint pin;
  uint bit;
  uint r, g, b; // idle color
  uint rp, gp, bp; // pressed color
  uint debounce;
} btn_cfg;

// cardinal directions
typedef struct {
  uint left;
  uint right;
  uint up;
  uint down;
  uint64_t time;
} btn_state;
