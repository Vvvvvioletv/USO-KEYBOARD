#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "usb_descriptors.h"

#include "bsp/board.h"
#include "usb.h"
#include "tusb.h"
#include "../include/resource.h"

#define MAGNETIC_THRESHOLD 0


void send_hid_report(uint8_t report_id, uint32_t btn)
{
    if (!tud_hid_ready())
        return;
    if (report_id == REPORT_ID_KEYBOARD)
    {
        static bool has_keyboard_key = false;
        uint8_t keycode[6] = {0};
        uint8_t key_index = 0;

        if (btn & (1 << 0))
        {
            if (key_index < 6)
                keycode[key_index++] = HID_KEY_Q;
        }
        if (btn & (1 << 1))
        {
            if (key_index < 6)
                keycode[key_index++] = HID_KEY_W;
        }
        if (btn & (1 << 2))
        {
            if (key_index < 6)
                keycode[key_index++] = HID_KEY_E;
        }

        if (btn)
        {
            tud_hid_keyboard_report(REPORT_ID_KEYBOARD, 0, keycode);
            has_keyboard_key = true;
        }
        else
        {
            if (has_keyboard_key)
            {
                tud_hid_keyboard_report(REPORT_ID_KEYBOARD, 0, NULL);
                has_keyboard_key = false;
            }
        }
    }
}

void hid_task(adc_data *data)
{
    uint32_t btn = 0;
    static uint32_t last_btn = 0;
    if (data->adc0_value < MAGNETIC_THRESHOLD){
        btn |= (1 << 0);
    }
    if (data->adc1_value < MAGNETIC_THRESHOLD){
        btn |= (1 << 1);
    }
    if (data->adc2_value < MAGNETIC_THRESHOLD){
        btn |= (1 << 2);
    }
    if (btn != last_btn){
        if (!tud_suspended())
        {
            send_hid_report(REPORT_ID_KEYBOARD, btn);
        }
        else if (btn)
        {
            tud_remote_wakeup();
        }
        last_btn = btn;
    }
}


void tud_hid_report_complete_cb(uint8_t instance, uint8_t const *report, uint16_t len)
{
    (void)instance;
    (void)report;
    (void)len;
}

uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen)
{
    // TODO not Implemented
    (void)instance;
    (void)report_id;
    (void)report_type;
    (void)buffer;
    (void)reqlen;

    return 0;
}

// Invoked when received SET_REPORT control request
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize)
{
    (void)instance;
    (void)report_id;
    (void)report_type;
    (void)buffer;
    (void)bufsize;
}
