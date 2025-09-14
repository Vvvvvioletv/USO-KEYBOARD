#include "ssd1306.h"
#include "ssd1306_font.h"
#include "pico/platform.h"
#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "pico/time.h"
#include "pico/types.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>
#include "hardware/dma.h"
#include "hardware/spi.h"
#include "../build/spi.pio.h"
#include "hardware/pio.h"

static pio_spi_inst_t pio_spi;

// Function to get the absolute value of a signed integer
static inline int16_t tool_fast_abs(int16_t t)
{
    return (t ^ (t >> 15)) - (t >> 15);
}

// Initialize the OLED display
void oled_init(void)
{

    PIO pio = pio0;
    uint sm = 0;
    uint offset = pio_add_program(pio, &spi_cpha0_program);
    pio_spi_init(pio, sm, offset, 8, 1.0f, false, false, SPI0_SCK, SPI0_TX, SPI0_TX);
    pio_spi.cs_pin = SPI0_CS;
    pio_spi.pio = pio;
    pio_spi.sm = sm;

    uint8_t init_cmds[] = {
        OLED_DISPLAY_OFF,
        0xD5, 0xF1,           // 设置时钟分频
        OLED_SET_CHARGE_PUMP, // 启用电荷泵（必须）
        0x14,                 // 0x14（启用）, 0x10（禁用）
        OLED_SET_DISPLAY_START_LINE,
        0xA1,              // 0xA0（不翻转）或 0xA1（水平翻转）
        0xC8,              // 0xC0（不翻转）或 0xC8（垂直翻转）
        OLED_SET_CONTRAST, // 对比度
        0x7F,              // 默认值（0-255）
        OLED_SET_COM_PINS, // COM引脚配置
        0x12,              // 0x12（128x64）, 0x02（128x32）
        OLED_SET_VCOMH_DESELECT,
        0x20, // VCOMH电平（0x20-0x30）
        OLED_DISPLAY_ON};

    // uint8_t init_cmds_test[] = {
    //     OLED_DISPLAY_OFF,
    //     //Set start col
    //     0x00,
    //     0x10,
    //     //Set display start line
    //     0x40,
    //     //Set page address
    //     0xB0,
    //     //Set contrast control
    //     0x81,
    //     0x66,
    //     OLED_SET_SEG_REMAP,
    //     0xA6,
    //     0xA8,
    //     0x3F,
    //     0xC8,
    //     0xD3,
    //     0x00,
    //     0xD5,
    //     0xF1,
    //     0xD9,
    //     0x1F,
    //     0xDA,
    //     0x12,
    //     0xDB,
    //     0x30,
    //     0x8D,
    //     0x10,
    //     0xAF
    // };
    oled_write_cmd_list(init_cmds, count_of(init_cmds)); // Send initialization commands
    sleep_ms(20);                                        // Wait for the display to initialize
    // oled_write_cmd(OLED_DISPLAY_ON);                     // Turn on the display
}

void pio_spi_init(PIO pio, uint sm, uint prog_offs, uint n_bits,float clkdiv, bool cpha, bool cpol, uint pin_sck, uint pin_mosi, uint pin_miso)
{
    pio_sm_config c = cpha ? spi_cpha1_program_get_default_config(prog_offs) : spi_cpha0_program_get_default_config(prog_offs);
    sm_config_set_out_pins(&c, pin_mosi, 1);
    sm_config_set_in_pins(&c, pin_miso);
    sm_config_set_sideset_pins(&c, pin_sck);

    sm_config_set_out_shift(&c, false, true, n_bits);
    sm_config_set_in_shift(&c, false, true, n_bits);
    sm_config_set_clkdiv(&c, clkdiv);

    // MOSI, SCK output are low, MISO is input
    pio_sm_set_pins_with_mask(pio, sm, 0, (1u << pin_sck) | (1u << pin_mosi));
    pio_sm_set_pindirs_with_mask(pio, sm, (1u << pin_sck) | (1u << pin_mosi), (1u << pin_sck) | (1u << pin_mosi) | (1u << pin_miso));
    pio_gpio_init(pio, pin_mosi);
    pio_gpio_init(pio, pin_miso);
    pio_gpio_init(pio, pin_sck);

    // The pin muxes can be configured to invert the output (among other things
    // and this is a cheesy way to get CPOL=1
    gpio_set_outover(pin_sck, cpol ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);
    // SPI is synchronous, so bypass input synchroniser to reduce input delay.
    hw_set_bits(&pio->input_sync_bypass, 1u << pin_miso);

    pio_sm_init(pio, sm, prog_offs, &c);
    pio_sm_set_enabled(pio, sm, true);
}

void __time_critical_func(pio_spi_write8_blocking)(const pio_spi_inst_t *spi, const uint8_t *src, size_t len)
{
    size_t tx_remain = len, rx_remain = len;
    // Do 8 bit accesses on FIFO, so that write data is byte-replicated. This
    // gets us the left-justification for free (for MSB-first shift-out)
    io_rw_8 *txfifo = (io_rw_8 *)&spi->pio->txf[spi->sm];
    io_rw_8 *rxfifo = (io_rw_8 *)&spi->pio->rxf[spi->sm];
    while (tx_remain || rx_remain)
    {
        if (tx_remain && !pio_sm_is_tx_fifo_full(spi->pio, spi->sm))
        {
            *txfifo = *src++;
            --tx_remain;
        }
        if (rx_remain && !pio_sm_is_rx_fifo_empty(spi->pio, spi->sm))
        {
            (void)*rxfifo;
            --rx_remain;
        }
    }
}


// Write a single command to the OLED
static inline int oled_write_cmd(uint8_t cmd)
{
    gpio_put(SPI0_CS, 0);
    gpio_put(SPI0_DC, 0);
    pio_spi_write8_blocking(&pio_spi, &cmd, 1);
    // gpio_put(SPI0_CS, 1);
}

// Write a list of commands to the OLED
static inline void oled_write_cmd_list(uint8_t *list, unsigned int num)
{
    for (unsigned int i = 0; i < num; i++)
    {
        oled_write_cmd(list[i]); // Send each command
    }
}

// Clear the display by filling it with black
void oled_clear()
{
    oled_fill_screen_pure(0x00);
}

void oled_fill_white()
{
    uint8_t white_seq[OLED_SIZE];
    memset(white_seq, 0xff, count_of(white_seq));
    gpio_put(SPI0_CS, 0);
    oled_set_page(2);
    gpio_put(SPI0_DC, 1);
    pio_spi_write8_blocking(&pio_spi, white_seq, count_of(white_seq) + 1);

    gpio_put(SPI0_CS, 1);
}

// Fill the entire screen with a specific color
static inline void oled_fill_screen_pure(uint8_t color)
{
    gpio_put(SPI0_CS, 0);
    for (size_t j = 0; j < 8; j++)
    { // Loop through all pages
        oled_set_page(j);
        for (size_t i = 0; i < OLED_WIDTH; i++)
        {
            gpio_put(SPI0_DC, 1);
            pio_spi_write8_blocking(&pio_spi, &color, 1); // Write color data
        }
    }
    gpio_put(SPI0_CS, 1);
}

// Render an array of pixel data to the OLED
void oled_render_array(uint8_t *buf, uint16_t num)
{
    if (buf == NULL || num == 0)
        return; // Check for valid buffer

    oled_set_page(0); // Set to the first page

    // If the number of bytes is less than the OLED width
    if (num <= OLED_WIDTH)
    {
        uint8_t *tmp = malloc(num + 1); // Allocate memory for buffer
        if (tmp == NULL)
            return; // Check for allocation failure

        gpio_put(SPI0_DC, 1);
        gpio_put(SPI0_CS, 0);
        memcpy(tmp + 1, buf, num); // Copy data

        pio_spi_write8_blocking(&pio_spi, tmp, num + 1); // Send data
        gpio_put(SPI0_CS, 1);
        free(tmp);
        return;
    }

    uint8_t *tmp = malloc(OLED_WIDTH + 1); // Allocate memory
    if (tmp == NULL)
        return; // Check for allocation failure
    gpio_put(SPI0_DC, 1);
    uint8_t pages = num / OLED_WIDTH;     // Calculate full pages
    uint8_t remaining = num % OLED_WIDTH; // Calculate remaining bytes

    for (uint8_t page = 0; page < pages; page++)
    {
        oled_set_page(page);                                  // Set current page
        memcpy(tmp + 1, buf + page * OLED_WIDTH, OLED_WIDTH); // Copy the full page data
        gpio_put(SPI0_CS, 0);
        pio_spi_write8_blocking(&pio_spi, tmp, OLED_WIDTH + 1); // Send page data
        gpio_put(SPI0_CS, 1);
    }

    // Send any remaining data
    if (remaining != 0)
    {
        oled_set_page(pages);                                 // Set next page
        memcpy(tmp + 1, buf + pages * OLED_WIDTH, remaining); // Copy remaining data
        gpio_put(SPI0_CS, 0);
        pio_spi_write8_blocking(&pio_spi, tmp, remaining + 1); // Send remaining data
        gpio_put(SPI0_CS, 1);
    }

    free(tmp); // Free allocated memory
}

// Render the complete frame to the OLED
void oled_render_frame(uint8_t *frame)
{
    if (frame == NULL)
        return; // Check for valid frame

    for (uint8_t page = 0; page < OLED_PAGES; page++)
    {
        gpio_put(SPI0_CS, 0);
        gpio_put(SPI0_DC, 1);
        oled_set_page(page); // Set to current page
        oled_set_col(0);
        pio_spi_write8_blocking(&pio_spi, frame + OLED_WIDTH * page, OLED_WIDTH); // Send the data
        gpio_put(SPI0_CS, 1);
    }
}

// DMA version of rendering the frame
void oled_render_frame_dma(uint8_t *frame)
{
    if (!frame)
        return; // Check for valid frame

    uint8_t *buf0 = malloc(OLED_WIDTH + 1); // Allocate buffer
    if (!buf0)
        return; // Check for allocation failure
    gpio_put(SPI0_DC, 1);

    int oled_dma_chan = dma_claim_unused_channel(1);                                    // Claim an unused DMA channel
    dma_channel_config oled_dma_config = dma_channel_get_default_config(oled_dma_chan); // Get the default channel config
    channel_config_set_transfer_data_size(&oled_dma_config, DMA_SIZE_8);                // Set data size
    channel_config_set_read_increment(&oled_dma_config, true);                          // Enable read address increment
    channel_config_set_write_increment(&oled_dma_config, true);                         // Enable write address increment

    dma_channel_configure(
        oled_dma_chan,
        &oled_dma_config,
        buf0 + 1,   // Write address
        frame,      // Read address
        OLED_WIDTH, // Number of bytes to transfer
        true        // Start transfer immediately
    );

    for (uint8_t page = 1; page < OLED_PAGES + 1; page++)
    {
        oled_set_page(page - 1);                             // Set current page
        dma_channel_wait_for_finish_blocking(oled_dma_chan); // Wait for DMA transfer to finish
        gpio_put(SPI0_CS, 0);
        pio_spi_write8_blocking(&pio_spi, buf0, OLED_WIDTH + 1); // Send the data
        gpio_put(SPI0_CS, 1);
        dma_channel_set_read_addr(oled_dma_chan, frame + page * OLED_WIDTH, false); // Set next read address
        dma_channel_set_write_addr(oled_dma_chan, buf0 + 1, true);                  // Set buffer write address
    }

    dma_channel_cleanup(oled_dma_chan);
    dma_channel_unclaim(oled_dma_chan);
    free(buf0); // Free allocated memory
}

// Clear and render the frame via DMA
void oled_render_frame_dma_clear(uint8_t *frame)
{
    oled_render_frame_dma(frame); // Render the frame
    oled_init_frame(frame);       // Initialize the frame (clear it)
}

// Set the current page for the OLED
static inline void oled_set_page(uint8_t page)
{
    // SSD1306 使用 0x22 命令设置页地址范围（起始页和结束页）
    oled_write_cmd(0xB0 + page); // 页地址命令
    // oled_write_cmd(page); // 起始页
    // oled_write_cmd(page); // 结束页（如果只设置单页，起始=结束）

    // 设置列地址范围（0x21 + 起始列 + 结束列）
    // oled_write_cmd(0x21); // 列地址命令
    // oled_write_cmd(0);    // 起始列=0
    // oled_write_cmd(127);  // 结束列=127（128列）
}

// Initialize the frame buffer to 0 (clear)
void oled_init_frame(uint8_t *frame)
{
    memset(frame, 0x00, OLED_SIZE_BYTE); // Clear the frame buffer
}

// Set a pixel in the frame buffer
void oled_set_pixel(uint8_t *frame, position pos, uint8_t on)
{
    assert(x >= 0 && x < OLED_WIDTH && y >= 0 && y < OLED_HEIGHT); // Check pixel bounds
    uint16_t index = (pos.y / 8) * OLED_WIDTH + pos.x;             // Calculate index in buffer
    uint8_t *target_pixel_col = &frame[index];                     // Get pointer to target pixel column
    if (on)
    {
        *(target_pixel_col) |= 1 << (pos.y % 8); // Set the pixel
    }
    else
    {
        *(target_pixel_col) &= ~(1 << (pos.y % 8)); // Clear the pixel
    }
}

// Draw a line using Bresenham's algorithm
void oled_draw_line(uint8_t *frame, position pos, position pos1, uint8_t on)
{
    int16_t dx = tool_fast_abs(pos1.x - pos.x);
    int16_t sx = pos.x < pos1.x ? 1 : -1; // Determine the step direction
    int16_t dy = -tool_fast_abs(pos1.y - pos.y);
    int16_t sy = pos.y < pos1.y ? 1 : -1; // Determine the step direction
    int16_t err = dx + dy;                // Error value

    while (true)
    {
        oled_set_pixel(frame, pos, on); // Set the pixel
        if (pos.x == pos1.x && y0 == y1)
            break; // If reached the endpoint, break

        int16_t e2 = 2 * err;
        if (e2 >= dy)
        {
            err += dy;   // Adjust error value
            pos.x += sx; // Move in x direction
        }
        if (e2 <= dx)
        {
            err += dx;   // Adjust error value
            pos.y += sy; // Move in y direction
        }
    }
}

// Get the font index for a given character
static inline int16_t oled_get_font_index(uint8_t ch)
{
    if (ch >= 'A' && ch <= 'Z')
    {
        return ch - 'A' + 1; // Index for uppercase letters
    }
    else if (ch >= '0' && ch <= '9')
    {
        return ch - '0' + 27; // Index for digits
    }
    else
        return 0; // Default index
}

// Write a character to the frame buffer
void oled_write_char(uint8_t *frame, position pos, uint8_t ch)
{
    if (pos.x > OLED_WIDTH - 8 || pos.y > OLED_HEIGHT - 8)
        return; // Check bounds

    pos.y = pos.y / 8;                     // Adjust y for character height
    ch = toupper(ch);                      // Convert to uppercase
    int16_t idx = oled_get_font_index(ch); // Get font index
    int16_t fb_idx = pos.y * 128 + pos.x;  // Calculate frame buffer index

    for (int i = 0; i < 8; i++)
    {
        frame[fb_idx++] = font[idx * 8 + i]; // Copy font data to frame
    }
}

// Lower the character as a whole by one pixel
void oled_write_char_fix(uint8_t *frame, position pos, uint8_t ch)
{
    if (pos.x > OLED_WIDTH - 8 || pos.y > OLED_HEIGHT - 8)
        return; // Check bounds

    pos.y = pos.y / 8;                     // Adjust y for character height
    ch = toupper(ch);                      // Convert to uppercase
    int16_t idx = oled_get_font_index(ch); // Get font index
    int16_t fb_idx = pos.y * 128 + pos.x;  // Calculate frame buffer index

    for (int i = 0; i < 8; i++)
    {
        frame[fb_idx++] = font[idx * 8 + i] << 1; // Copy font(lower the character as a whole by one pixel) data to frame
    }
}

// Write a string to the frame buffer
void oled_write_string(uint8_t *frame, position pos, char *str)
{
    if (pos.x > OLED_WIDTH - 8 || pos.y > OLED_HEIGHT - 8)
        return; // Check bounds
    while (*str)
    {
        oled_write_char(frame, pos, *str++); // Write each character
        pos.x += 8;                          // Move to next character position
    }
}

// Draw a function on the OLED
void oled_draw_fun(uint8_t *frame, func f, position pos, position pos1)
{
    uint8_t y_val[OLED_WIDTH];
    memset(y_val, 0x00, OLED_WIDTH);
    uint8_t x = pos.x;
    for (int i = 0; x != pos1.x; x++, i++)
    {
        y_val[i] = f(x);
    }

    for (int i = 0; i < OLED_WIDTH; i++)
    {
        uint8_t y = y_val[i];
        if (y <= OLED_HEIGHT)
        {
            position pixel_pos = {i, y};
            oled_set_pixel(frame, pixel_pos, 1);
            //    oled_set_pixel(frame,i,y,1);
        }
    }
}

// Draw a circle using the midpoint algorithm
static void oled_draw_circle(uint8_t *frame, position center, uint8_t radius)
{
    int16_t x = radius;
    int16_t y = 0;
    int16_t radiusError = 1 - radius; // Initialize the decision parameter

    while (x >= y)
    {
        // Draw the eight symmetric points of the circle
        position pos1 = {center.x + x, center.y + y};
        oled_set_pixel(frame, pos1, 1);

        position pos2 = {center.x + y, center.y + x};
        oled_set_pixel(frame, pos2, 1);

        position pos3 = {center.x - y, center.y + x};
        oled_set_pixel(frame, pos3, 1);

        position pos4 = {center.x - x, center.y + y};
        oled_set_pixel(frame, pos4, 1);

        position pos5 = {center.x - x, center.y - y};
        oled_set_pixel(frame, pos5, 1);

        position pos6 = {center.x - y, center.y - x};
        oled_set_pixel(frame, pos6, 1);

        position pos7 = {center.x + y, center.y - x};
        oled_set_pixel(frame, pos7, 1);

        position pos8 = {center.x + x, center.y - y};
        oled_set_pixel(frame, pos8, 1);

        y++;
        if (radiusError < 0)
        {
            radiusError += 2 * y + 1; // Choose the next point
        }
        else
        {
            x--;                            // Move horizontally
            radiusError += 2 * (y - x + 1); // Adjust decision parameter
        }
    }
}

static inline void oled_set_col(uint8_t start_col)
{
    oled_write_cmd(0x00 | (start_col & 0x0F));
    oled_write_cmd(0x10 | (start_col >> 4));
}

void row_update(int page, uint8_t *line_buf)
{
    // 检查参数合法性
    if (page < 0 || page >= OLED_PAGES || line_buf == NULL)
    {
        return;
    }

    oled_set_page(page);

    // 发送数据到OLED
    gpio_put(SPI0_DC, 1);
    gpio_put(SPI0_CS, 0);
    pio_spi_write8_blocking(&pio_spi, line_buf, OLED_WIDTH);
    gpio_put(SPI0_CS, 1);
}

void partial_update(int page, uint8_t start_col, uint8_t *data, uint8_t len)
{
    if (page < 0 || page >= OLED_PAGES || start_col >= OLED_WIDTH)
    {
        return;
    }

    if (data == NULL)
    {
        return;
    }

    oled_set_page(page);
    oled_set_col(start_col);

    gpio_put(SPI0_DC, 1);
    gpio_put(SPI0_CS, 0);
    pio_spi_write8_blocking(&pio_spi, data, len);
    gpio_put(SPI0_CS, 1);
}
