#include "tusb.h"

/* ---------------- Device descriptor ---------------- */
tusb_desc_device_t const desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_UNSPECIFIED,
    .bDeviceSubClass = 0x00,
    .bDeviceProtocol = 0x00,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor  = 0xCafe,
    .idProduct = 0x4000,
    .bcdDevice = 0x0100,
    .iManufacturer = 0x01,
    .iProduct      = 0x02,
    .iSerialNumber = 0x03,
    .bNumConfigurations = 0x01
};

uint8_t const * tud_descriptor_device_cb(void) {
  return (uint8_t const *) &desc_device;
}

// ---------------- HID Report Descriptor ----------------

// 8 buttons, no axes, 2 bytes total (byte2 = padding for future use)
uint8_t const desc_hid_report[] = {
  0x05,0x01,        // Generic Desktop
  0x09,0x05,        // Game Pad
  0xA1,0x01,        // Application
    0x05,0x09,      // Button
    0x19,0x01,      // Button 1
    0x29,0x15,      // Button 21
    0x15,0x00,      // Logical Min 0
    0x25,0x01,      // Logical Max 1
    0x95,0x15,      // Report Count: 21 buttons
    0x75,0x01,      // Report Size: 1 bit
    0x81,0x02,      // Input (Data,Var,Abs)
    0x95,0x0B,      // pad 11 bits to make 32 bits (4B)
    0x75,0x01,
    0x81,0x03,      // Input (Const,Var,Abs)
  0xC0
};

uint8_t const * tud_hid_descriptor_report_cb(uint8_t itf) {
  (void) itf;
  return desc_hid_report;
}

/* ---------------- Configuration descriptor ---------------- */
enum { ITF_NUM_HID, ITF_COUNT };
#define EPNUM_HID   0x81

#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN)

uint8_t const desc_configuration[] = {
  // Config
  TUD_CONFIG_DESCRIPTOR(1, ITF_COUNT, 0, CONFIG_TOTAL_LEN, 0x00, 100),

  // HID: 8-byte IN endpoint, 1ms interval
  TUD_HID_DESCRIPTOR(0, 0, HID_ITF_PROTOCOL_NONE, sizeof(desc_hid_report), EPNUM_HID, 8, 1),
};

uint8_t const * tud_descriptor_configuration_cb(uint8_t index) {
  (void) index;
  return desc_configuration;
}

/* ---------------- Strings ---------------- */
static char const* string_desc[] = {
  (const char[]) { 0x09, 0x04 },     // 0: LangID = English (0x0409)
  "beef",                            // 1: Manufacturer
  "beefpad",                         // 2: Product
  "696969",                          // 3: Serial
};

static uint16_t _desc_str[32];

uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
  (void) langid;

  uint8_t chr_count;
  if (index == 0) {
    _desc_str[0] = (TUSB_DESC_STRING << 8) | (2 + 2);
    _desc_str[1] = 0x0409;
    return _desc_str;
  }

  const char* str = (index < sizeof(string_desc)/sizeof(string_desc[0])) ? string_desc[index] : NULL;
  if (!str) return NULL;

  chr_count = (uint8_t)strlen(str);
  if (chr_count > 31) chr_count = 31;

  for (uint8_t i = 0; i < chr_count; i++) _desc_str[1 + i] = str[i];
  _desc_str[0] = (TUSB_DESC_STRING << 8) | (2*chr_count + 2);
  return _desc_str;
}
