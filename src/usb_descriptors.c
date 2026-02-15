#include "tusb.h"
#include "config.h"
#include "xinput.h"

extern USB_MODE usb_mode;

/* ---------------- Device descriptor ---------------- */
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
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor  = 0x045E,
    .idProduct = 0x028E,
    .bcdDevice = 0x0572,
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

// ---------------- HID Report Descriptor ----------------

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

// XInput configuration descriptor
const uint8_t desc_configuration_xinput[] = {
  // Configuration Descriptor
  0x09, // bLength
  0x02, // bDescriptorType
  0x30, 0x00, // wTotalLength (48 bytes)
  0x01, // bNumInterfaces
  0x01, // bConfigurationValue
  0x00, // iConfiguration
  0x80, // bmAttributes (Bus-powered)
  0xFA, // bMaxPower (500 mA)

  // Interface Descriptor
  0x09, // bLength
  0x04, // bDescriptorType
  0x00, // bInterfaceNumber
  0x00, // bAlternateSetting
  0x02, // bNumEndPoints
  0xFF, // bInterfaceClass (Vendor specific)
  0x5D, // bInterfaceSubClass
  0x01, // bInterfaceProtocol
  0x00, // iInterface

  // Unknown Descriptor (XInput specific)
  0x10, 0x21, 0x10, 0x01, 0x01, 0x24, 0x81, 0x14,
  0x03, 0x00, 0x03, 0x13, 0x02, 0x00, 0x03, 0x00,

  // Endpoint Descriptor (IN)
  0x07, // bLength
  0x05, // bDescriptorType
  0x81, // bEndpointAddress (IN endpoint 1)
  0x03, // bmAttributes (Interrupt)
  0x20, 0x00, // wMaxPacketSize (32 bytes)
  0x04, // bInterval (4 frames)

  // Endpoint Descriptor (OUT)
  0x07, // bLength
  0x05, // bDescriptorType
  0x02, // bEndpointAddress (OUT endpoint 2)
  0x03, // bmAttributes (Interrupt)
  0x20, 0x00, // wMaxPacketSize (32 bytes)
  0x08, // bInterval (8 frames)
};

uint8_t const * tud_descriptor_configuration_cb(uint8_t index) {
  (void) index;
  if (usb_mode == USB_MODE_XINPUT) {
    return desc_configuration_xinput;
  }
  return desc_configuration;
}

/* ---------------- Strings ---------------- */
static char const* string_desc[] = {
  (const char[]) { 0x09, 0x04 }, // 0: LangID = English (0x0409)
  MANUFACTURER,
  PRODUCT,
  SERIAL_NUM,
};

static char const* string_desc_xinput[] = {
  (const char[]) { 0x09, 0x04 }, // 0: LangID = English (0x0409)
  "GENERIC",
  "XINPUT CONTROLLER",
  "1.0",
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

  char const** string_table = (usb_mode == USB_MODE_XINPUT) ? string_desc_xinput : string_desc;
  size_t table_size = (usb_mode == USB_MODE_XINPUT) ?
                      (sizeof(string_desc_xinput)/sizeof(string_desc_xinput[0])) :
                      (sizeof(string_desc)/sizeof(string_desc[0]));

  const char* str = (index < table_size) ? string_table[index] : NULL;
  if (!str) return NULL;

  chr_count = (uint8_t)strlen(str);
  if (chr_count > 31) chr_count = 31;

  for (uint8_t i = 0; i < chr_count; i++) _desc_str[1 + i] = str[i];
  _desc_str[0] = (TUSB_DESC_STRING << 8) | (2*chr_count + 2);
  return _desc_str;
}
