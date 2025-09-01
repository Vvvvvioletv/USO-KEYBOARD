#include "tusb.h"
#include "usb_descriptors.h"
#include "pico/unique_id.h"

/* A combination of interfaces must have a unique product id */
#define _PID_MAP(itf, n)  ((CFG_TUD_##itf) << (n))

#define USB_BCD   0x0200

// Using Raspberry Pi's VID
#define USB_VID   0x2E8A  // Raspberry Pi Ltd.
#define USB_PID   (0x000A | _PID_MAP(CDC, 0) | _PID_MAP(HID, 2))

//--------------------------------------------------------------------+
// Device Descriptors
//--------------------------------------------------------------------+
tusb_desc_device_t const desc_device =
{
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = USB_BCD,
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor           = USB_VID,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x0100,

    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,

    .bNumConfigurations = 0x01
};

// Invoked when received GET DEVICE DESCRIPTOR
uint8_t const * tud_descriptor_device_cb(void)
{
  return (uint8_t const *) &desc_device;
}

// Interface numbers
enum {
    ITF_NUM_CDC_0 = 0,
    ITF_NUM_CDC_0_DATA,
    ITF_NUM_HID,
    ITF_NUM_TOTAL
};

//--------------------------------------------------------------------+
// HID Report Descriptor
//--------------------------------------------------------------------+

uint8_t const desc_hid_report[] =
{
  TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(REPORT_ID_KEYBOARD))
};

// Invoked when received GET HID REPORT DESCRIPTOR
uint8_t const * tud_hid_descriptor_report_cb(uint8_t instance)
{
  (void) instance;
  return desc_hid_report;
}

//--------------------------------------------------------------------+
// Configuration Descriptor
//--------------------------------------------------------------------+

#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_HID_DESC_LEN)

// Endpoint numbers for RP2040
#define EPNUM_CDC_0_NOTIF   0x81  // CDC notification endpoint
#define EPNUM_CDC_0_OUT     0x02  // CDC data OUT endpoint
#define EPNUM_CDC_0_IN      0x82  // CDC data IN endpoint
#define EPNUM_HID           0x83  // HID endpoint

uint8_t const desc_configuration[] =
{
  // Config number, interface count, string index, total length, attribute, power in mA
  TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

  // CDC Interface
  TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_0, 4, EPNUM_CDC_0_NOTIF, 8, EPNUM_CDC_0_OUT, EPNUM_CDC_0_IN, 64),
  
  // HID Keyboard Interface
  TUD_HID_DESCRIPTOR(ITF_NUM_HID, 5, HID_ITF_PROTOCOL_KEYBOARD, sizeof(desc_hid_report), EPNUM_HID, CFG_TUD_HID_EP_BUFSIZE, 5)
};

#if TUD_OPT_HIGH_SPEED
// Device qualifier for high-speed devices
tusb_desc_device_qualifier_t const desc_device_qualifier =
{
  .bLength            = sizeof(tusb_desc_device_qualifier_t),
  .bDescriptorType    = TUSB_DESC_DEVICE_QUALIFIER,
  .bcdUSB             = USB_BCD,

  .bDeviceClass       = TUSB_CLASS_MISC,
  .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
  .bDeviceProtocol    = MISC_PROTOCOL_IAD,

  .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
  .bNumConfigurations = 0x01,
  .bReserved          = 0x00
};

// Other speed configuration
uint8_t desc_other_speed_config[CONFIG_TOTAL_LEN];

uint8_t const* tud_descriptor_device_qualifier_cb(void)
{
  return (uint8_t const*) &desc_device_qualifier;
}

uint8_t const* tud_descriptor_other_speed_configuration_cb(uint8_t index)
{
  (void) index;
  memcpy(desc_other_speed_config, desc_configuration, CONFIG_TOTAL_LEN);
  desc_other_speed_config[1] = TUSB_DESC_OTHER_SPEED_CONFIG;
  return desc_other_speed_config;
}

#endif // highspeed

// Invoked when received GET CONFIGURATION DESCRIPTOR
uint8_t const * tud_descriptor_configuration_cb(uint8_t index)
{
  (void) index;
  return desc_configuration;
}

//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+

enum {
    STRID_LANGID = 0,
    STRID_MANUFACTURER,
    STRID_PRODUCT,
    STRID_SERIAL,
    STRID_CDC,
    STRID_HID,
};

char const* string_desc_arr[] =
{
  (const char[]) { 0x09, 0x04 }, // English
  "Raspberry Pi",                // Manufacturer
  "Pico HID+CDC Composite",      // Product
  NULL,                          // Serial (will use chip ID)
  "Pico CDC",                    // CDC Interface
  "Pico Keyboard"                // HID Interface
};

static uint16_t _desc_str[32];

// Helper function to convert Pico's unique ID to string
static void get_pico_serial_string(uint16_t* desc_str)
{
    pico_unique_board_id_t id;
    pico_get_unique_board_id(&id);
    
    // Convert 8-byte ID to 16-character hex string
    for (int i = 0; i < 8; i++) {
        uint8_t byte = id.id[i];
        desc_str[1 + i*2] = "0123456789ABCDEF"[byte >> 4];
        desc_str[1 + i*2 + 1] = "0123456789ABCDEF"[byte & 0x0F];
    }
}

uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
  (void) langid;
  uint8_t chr_count = 0;

  switch (index) {
    case STRID_LANGID:
      memcpy(&_desc_str[1], string_desc_arr[0], 2);
      chr_count = 1;
      break;
      
    case STRID_SERIAL:
      // Use RP2040's unique board ID as serial number
      get_pico_serial_string(_desc_str);
      chr_count = 16; // 8 bytes → 16 hex characters
      break;
      
    default:
      if (index >= sizeof(string_desc_arr)/sizeof(string_desc_arr[0]) || 
          !string_desc_arr[index]) {
        return NULL;
      }
      
      const char* str = string_desc_arr[index];
      chr_count = (uint8_t)strlen(str);
      if (chr_count > 31) chr_count = 31;

      for(uint8_t i = 0; i < chr_count; i++) {
        _desc_str[1 + i] = str[i];
      }
      break;
  }

  _desc_str[0] = (TUSB_DESC_STRING << 8) | (2 * chr_count + 2);
  return _desc_str;
}