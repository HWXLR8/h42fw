#pragma once

#define BTN_COUNT       21
#define LED_BTN_COUNT   16
#define LED_PIN         28
#define LED_TOGGLE_PIN  21
#define BOOTSEL_PIN     14
#define BOOTSEL_DELAY   500000 // us
#define LATCH_TIME      400    // us
#define TAC_DEBOUNCE    500    // us
#define KEY_DEBOUNCE    500    // us

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
  uint r, g, b;
} button_map;

// cardinal directions
typedef struct {
  uint left;
  uint right;
  uint up;
  uint down;
  uint64_t time;
  uint led_toggle;
} btn_state;
