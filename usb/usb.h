#ifndef __USB
#define __USB

#include <stdint.h>
#include "../include/resource.h"


void send_hid_report(uint8_t report_id, uint32_t btn);
void hid_task(adc_data *data);
void tud_hid_report_complete_cb(uint8_t instance, uint8_t const *report, uint16_t len);
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen);
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize);

#endif