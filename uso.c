/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdio.h>
#include "ssd1306.h"

// static uint8_t frame_template[] = { 
// };

int main() {
    stdio_init_all();
    sleep_ms(80);

    i2c_init(i2c0, OLED_BAUD);
    gpio_set_function(PIN_SCL, GPIO_FUNC_I2C);
    gpio_set_function(PIN_SDA, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_SCL);
    gpio_pull_up(PIN_SDA);  
    sleep_ms(100);
    OLED_Init();
    OLED_Clear();
    OLED_initFrame(frameBuff[0].frame);

    //Render_initPage(frameBuff[0].frame);
    // OLED_RenderFrame(frame_template);

    uint8_t frame[1024]; 
    OLED_initFrame(frame); 
    OLED_WriteString(frame, 20, 20, "HELLO WORLD");

    // 渲染到屏幕
    OLED_RenderFrame(frame);

    while (true) {
        sleep_ms(100);
        printf("TEST");
    }
}