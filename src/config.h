#pragma once

// key config
#define BTN_COUNT       21
#define LED_BTN_COUNT   16
#define LED_PIN         28
#define LED_TOGGLE_PIN  21
#define OLED_TOGGLE_PIN 20
#define BOOTSEL_PIN     14
#define LATCH_TIME      350 // us
#define TAC_DEBOUNCE    8000 // us
#define KEY_DEBOUNCE    8000 // us

// HID metadata
#define VENDOR_ID    0xBEEF
#define PRODUCT_ID   0xFEEF
#define MANUFACTURER "SEGV"
#define PRODUCT      "BEEFPAD"
#define SERIAL_NUM   "00000000"

// USB mode
typedef enum {
  USB_MODE_HID = 0,
  USB_MODE_XINPUT = 1,
} USB_MODE;

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
