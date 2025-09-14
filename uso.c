/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/adc.h"
#include <stdio.h>
#include "ssd1306.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "pico/multicore.h"
#include "pico/util/queue.h"
#include "hardware/dma.h"

#include "bsp/board.h"
#include "tusb.h"
#include "usb_descriptors.h"
#include "usb/usb.h"

#include "resource.h"

#define ENC_A 6
#define ENC_B 7
#define ENC_SW 8

#define info(msg) tud_cdc_n_write(0, (uint8_t const *)msg, sizeof(msg) - 1)

void adc_dma_irq(void);

const uint32_t ADC_READ_INTERVAL_MS = 10;

static uint16_t last_encoder_count = 0;
static volatile int16_t encoder_count = 0;
static volatile uint16_t key_press_count = 0;
static bool encoder_changed = false;
static bool key_pressed = false;
static uint8_t last_a_state;

char fps_str[16];
char adc0_str[16];
char adc1_str[16];
char adc2_str[16];
char enc_str[16];
char key_str[16];
static volatile uint32_t frame_count = 0;
static volatile uint32_t fps = 0;

bool last_magnetic_state = false;
const uint32_t interval_ms = 10;
static uint32_t start_ms = 0;

queue_t adc_queue;
const uint8_t queue_data_size = 50;

static int dma_chan;
static adc_data adc_value = {0};
static uint16_t *adc_value_p = (uint16_t*)&adc_value;
static uint8_t adc_offset = 0;

void adc_dma_irq(){
    dma_channel_acknowledge_irq0(dma_chan);
    adc_run(false);
    adc_select_input(0);

    // queue_add_blocking(&adc_queue,&adc_value);
    dma_channel_set_write_addr(dma_chan,adc_value_p,true);
    adc_run(true);
}

void core1_task()
{
    adc_init();
    adc_gpio_init(26);
    adc_gpio_init(27);
    adc_gpio_init(28);

    adc_select_input(0);
    adc_set_round_robin(0b111);
    adc_fifo_setup(true, true, 1, false, false);

    /*-------------DMA---------------*/
    dma_chan = dma_claim_unused_channel(true);
    dma_channel_config cfg = dma_channel_get_default_config(dma_chan);
    
    channel_config_set_dreq(&cfg,DREQ_ADC);
    channel_config_set_read_increment(&cfg,false);
    channel_config_set_write_increment(&cfg,true);
    channel_config_set_transfer_data_size(&cfg,DMA_SIZE_16);

    dma_channel_configure(dma_chan,&cfg,adc_value_p,&adc_hw->fifo,3,true);

    dma_channel_set_irq0_enabled(dma_chan,true);
    irq_set_exclusive_handler(DMA_IRQ_0,adc_dma_irq);
    irq_set_enabled(DMA_IRQ_0,true);
    /*-------------DMA---------------*/

    adc_run(true);

    while (true)
    {
        tud_task();
        hid_task(&adc_value);
        printf("adc_value0: %d\n",adc_value.adc0_value);

    }
}



void adc_progress(uint8_t *frame, adc_data *adc_value)
{
    static bool initial = true;
    static uint16_t initial_adc0 = 0;
    static uint16_t initial_adc1 = 0;
    static uint16_t initial_adc2 = 0;
    static uint8_t init_count = 0;

    if (initial)
    {
        init_count++;
        if (init_count > 10)
        {
            initial_adc0 = adc_value->adc0_value;
            initial_adc1 = adc_value->adc1_value;
            initial_adc2 = adc_value->adc2_value;
            initial = false;
        }
    }

    int16_t delta0 = abs(adc_value->adc0_value - initial_adc0);
    int16_t delta1 = abs(adc_value->adc1_value - initial_adc1);
    int16_t delta2 = abs(adc_value->adc2_value - initial_adc2);

    uint16_t adc0_progress = (200 * delta0) / 4095;
    uint16_t adc1_progress = (200 * delta1) / 4095;
    uint16_t adc2_progress = (200 * delta2) / 4095;

    adc0_progress = adc0_progress > 32 ? 32 : adc0_progress;
    adc1_progress = adc1_progress > 32 ? 32 : adc1_progress;
    adc2_progress = adc2_progress > 32 ? 32 : adc2_progress;

    memset(frame + OLED_WIDTH * 2 + 45, 0, 32);
    memset(frame + OLED_WIDTH * 2 + 45, 0xFF, adc0_progress);
    memset(frame + OLED_WIDTH * 3 + 45, 0, 32);
    memset(frame + OLED_WIDTH * 3 + 45, 0xFF, adc1_progress);
    memset(frame + OLED_WIDTH * 4 + 45, 0, 32);
    memset(frame + OLED_WIDTH * 4 + 45, 0xFF, adc2_progress);

    partial_update(2, 45, frame + OLED_WIDTH * 2 + 45, 32);
    partial_update(3, 45, frame + OLED_WIDTH * 3 + 45, 32);
    partial_update(4, 45, frame + OLED_WIDTH * 4 + 45, 32);
}

void ec11_irq(uint gpio, uint32_t events)
{
    uint32_t gpio_state = 0;

    gpio_state = (gpio_get_all() >> 6) & 0b0111;

    static bool ccw_fall = 0;
    static bool cw_fall = 0;

    static bool sw_debounce = false; 
    static uint32_t sw_debounce_time = 0;

    uint8_t enc_value = 0;
    enc_value = (gpio_state & 0x03);

    if (gpio == ENC_A)
    {
        if ((!cw_fall) && (enc_value == 0b10))
            cw_fall = 1;
        if ((ccw_fall) && (enc_value == 0b00))
        {
            cw_fall = 0;
            ccw_fall = 0;
            encoder_count++;
        }
    }
    else if (gpio == ENC_B)
    {
        if ((!ccw_fall) && (enc_value == 0b01))
            ccw_fall = 1;

        if ((cw_fall) && (enc_value == 0b00))
        {
            cw_fall = 0;
            ccw_fall = 0;
            encoder_count--;
        }
    }
    else if (gpio == ENC_SW)
    {
        // if (!gpio_get(ENC_SW))
        // {
        //     key_pressed = true;
        //     key_press_count++;
        // }
        uint32_t now = board_millis();
        
        if (!gpio_get(ENC_SW) && !sw_debounce)
        {
            sw_debounce = true;
            sw_debounce_time = now;
        }
        else if (sw_debounce && now - sw_debounce_time >= 20)
        {
            if (!gpio_get(ENC_SW))
            {
                key_pressed = true;
                key_press_count++;
            }
            sw_debounce = false;
        }
    }
}

static bool first_partial_update = true;

void partial_update_frame(uint8_t *frame, adc_data *adc)
{
    uint32_t send_ps = fps;

    // uint16_t voltage0 = (adc->adc0_value * 3.3 * 1000) / 4095;
    // uint16_t voltage1 = (adc->adc1_value * 3.3 * 1000) / 4095;
    // uint16_t voltage2 = (adc->adc2_value * 3.3 * 1000) / 4095;

    snprintf(adc0_str, sizeof(adc0_str), "%4d", adc->adc0_value);
    snprintf(adc1_str, sizeof(adc1_str), "%4d", adc->adc1_value);
    snprintf(adc2_str, sizeof(adc2_str), "%4d", adc->adc2_value);
    // snprintf(adc0_str, sizeof(adc0_str), "%4d", voltage0);
    // snprintf(adc1_str, sizeof(adc1_str), "%4d", voltage1);
    // snprintf(adc2_str, sizeof(adc2_str), "%4d", voltage2);
    snprintf(fps_str, sizeof(fps_str), "%4d", send_ps);
    snprintf(enc_str, sizeof(enc_str), "%3d", encoder_count);
    snprintf(key_str, sizeof(key_str), "%3d", key_press_count);

    if (first_partial_update)
    {
        oled_write_string(frame, (position){0, 8}, "SEND_PS:");
        oled_write_string(frame, (position){60, 8}, fps_str);
        oled_write_string(frame, (position){0, 16}, "ADC_0:");
        oled_write_string(frame, (position){80, 16}, adc0_str);
        // oled_write_string(frame, (position){112, 16}, "MV");
        oled_write_string(frame, (position){0, 24}, "ADC_1:");
        oled_write_string(frame, (position){80, 24}, adc1_str);
        // oled_write_string(frame, (position){112, 24}, "MV");
        oled_write_string(frame, (position){0, 32}, "ADC_2:");
        oled_write_string(frame, (position){80, 32}, adc2_str);
        // oled_write_string(frame, (position){112, 32}, "MV");
        oled_write_string(frame, (position){0, 40}, "EC_COUNT:");
        oled_write_string(frame, (position){100, 40}, enc_str);
        oled_write_string(frame, (position){0, 48}, "KEY_COUNT:");
        oled_write_string(frame, (position){100, 48}, key_str);
        first_partial_update = false;
    }
    else
    {
        oled_write_string(frame, (position){60, 8}, fps_str);
        oled_write_string(frame, (position){80, 16}, adc0_str);
        oled_write_string(frame, (position){80, 24}, adc1_str);
        oled_write_string(frame, (position){80, 32}, adc2_str);
        oled_write_string(frame, (position){100, 40}, enc_str);
        oled_write_string(frame, (position){100, 48}, key_str);
    }

    adc_progress(frame, adc);
}

bool fps_timer_callback(struct repeating_timer *t)
{
    fps = frame_count;
    frame_count = 0;
    return true;
}

void ec11_init()
{

    gpio_init(ENC_A);
    gpio_init(ENC_B);
    gpio_init(ENC_SW);
    gpio_set_dir(ENC_A, GPIO_IN);
    gpio_set_dir(ENC_B, GPIO_IN);
    gpio_set_dir(ENC_SW, GPIO_IN);
    gpio_pull_up(ENC_A);
    gpio_pull_up(ENC_B);
    gpio_pull_up(ENC_SW);

    last_a_state = gpio_get(6);

    gpio_set_irq_enabled_with_callback(6, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, ec11_irq);
    gpio_set_irq_enabled_with_callback(7, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, ec11_irq);
    gpio_set_irq_enabled_with_callback(8, GPIO_IRQ_EDGE_FALL, true, ec11_irq);
}

void tud_cdc_rx_cb(uint8_t itf)
{
    // allocate buffer for the data in the stack
    uint8_t buf[CFG_TUD_CDC_RX_BUFSIZE];

    // read the available data
    // | IMPORTANT: also do this for CDC0 because otherwise
    // | you won't be able to print anymore to CDC0
    // | next time this function is called
    uint32_t count = tud_cdc_n_read(itf, buf, sizeof(buf));

    // check if the data was received on the second cdc interface
    if (itf == 1)
    {
        // process the received data
        buf[count] = 0; // null-terminate the string
        // now echo data back to the console on CDC 0

        // and echo back OK on CDC 1
        tud_cdc_n_write(itf, (uint8_t const *)"OK\r\n", 4);
        tud_cdc_n_write_flush(itf);
    }
}

int main()
{

    uint8_t frame[1024];

    gpio_init(23);
    gpio_set_dir(23, GPIO_OUT);

    // spi_init(OLED_SPI_PORT, OLED_SPI_BAUD);
    gpio_init(SPI0_DC);
    gpio_init(SPI0_RES);
    gpio_init(SPI0_CS);
    gpio_set_function(SPI0_SCK, GPIO_FUNC_SPI);
    gpio_set_function(SPI0_TX, GPIO_FUNC_SPI);
    gpio_set_dir(SPI0_DC, GPIO_OUT);
    gpio_set_dir(SPI0_CS, GPIO_OUT);
    gpio_set_dir(SPI0_RES, GPIO_OUT);
    gpio_put(SPI0_CS, 1);
    gpio_put(SPI0_RES, 0);
    sleep_ms(50);
    gpio_put(SPI0_RES, 1);
    sleep_ms(200);

    oled_init();
    oled_init_frame(frame);
    oled_clear();

    ec11_init();

    board_init();
    tusb_init();
    stdio_init_all();

    queue_init(&adc_queue, sizeof(adc_data), queue_data_size);
    multicore_launch_core1(core1_task);

    static struct repeating_timer fps_timer;
    add_repeating_timer_ms(1000, fps_timer_callback, NULL, &fps_timer);

    int progress_len = 0;
    uint8_t progress[1] = {0xFF};
    uint8_t clear_arr[128] = {0};
    adc_data get_data;
    static uint32_t last_screen_update = 0;
    const uint32_t SCREEN_UPDATE_INTERVAL_MS = 50;

    while (true)
    {
        // queue_remove_blocking(&adc_queue, &get_data);
        // queue_try_remove(&adc_queue, &get_data);
        // info("core 0\n");

        get_data = adc_value;

        frame_count++;
        partial_update_frame(frame, &get_data);

        if ((last_encoder_count >= 10 && encoder_count <= 9 && encoder_count == last_encoder_count - 1))
        {
            memset(frame + OLED_WIDTH * 5, 0, OLED_WIDTH);
            oled_write_string(frame, (position){0, 40}, "EC_COUNT:");
            row_update(5, frame + OLED_WIDTH * 5);
        }

        last_encoder_count = encoder_count;

        encoder_count = (encoder_count > 100) ? 100 : (encoder_count < 0 ? 0 : encoder_count);

        progress_len++;
        if (progress_len == 127)
        {
            row_update(0, clear_arr);
            progress_len = 0;
        }

        partial_update(0, progress_len - 1, progress, 1);
        partial_update(1, 0, frame + OLED_WIDTH, OLED_WIDTH);

        uint32_t now = board_millis();
        if (now - last_screen_update >= SCREEN_UPDATE_INTERVAL_MS)
        {
            last_screen_update = now;
            row_update(2, frame + OLED_WIDTH * 2);
            row_update(3, frame + OLED_WIDTH * 3);
            row_update(4, frame + OLED_WIDTH * 4);
            row_update(5, frame + OLED_WIDTH * 5);
            row_update(6, frame + OLED_WIDTH * 6);
        }
    }
}
