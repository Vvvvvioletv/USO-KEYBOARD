#ifndef __SH1106
#define __SH1106

#include <stdint.h>
#include "../build/spi.pio.h"

#define OLED_WIDTH  128
#define OLED_HEIGHT 64
#define OLED_HEIGHT_BYTE    (OLED_HEIGHT / 8)
#define OLED_WIDTH_BYTE     (OLED_WIDTH / 8)
#define OLED_SIZE           (OLED_HEIGHT * OLED_WIDTH) 
#define OLED_SIZE_BYTE      (OLED_SIZE / 8)


// #define PIN_SCL PICO_DEFAULT_I2C_SCL_PIN 
// #define PIN_SDA PICO_DEFAULT_I2C_SDA_PIN

#define SPI0_DC 1
#define SPI0_SCK 2
#define SPI0_TX 3
#define SPI0_CS 5
#define SPI0_RES 14

#define OLED_SPI_PORT spi0
#define OLED_SPI_BAUD 10000000
// // #define OLED_I2C_ADDR 0x3C
// #define OLED_BAUD (4.07 * 100000)

//Commands
#define OLED_DISPLAY_OFF                _u(0xAE)
#define OLED_DISPLAY_ON                 _u(0xAF)

// SSD1306 特有指令
#define OLED_SET_COL_ADDR               _u(0x21)  // 设置列地址范围
#define OLED_SET_PAGE_ADDR              _u(0x22)  // 设置页地址范围
#define OLED_SET_SEG_REMAP              _u(0xA1)  // 段重映射（水平翻转）
#define OLED_SET_COM_SCAN_DIR           _u(0xC8)  // 扫描方向（垂直翻转）
#define OLED_SET_DISPLAY_START_LINE     _u(0x40)  // 起始行
#define OLED_SET_CONTRAST               _u(0x81)  // 对比度控制
#define OLED_SET_OSC_FREQ               _u(0xD5)  // 时钟分频/频率
#define OLED_SET_CHARGE_PUMP            _u(0x8D)  // 电荷泵使能
#define OLED_SET_COM_PINS               _u(0xDA)  // COM引脚配置
#define OLED_SET_VCOMH_DESELECT         _u(0xDB)  // VCOMH电平

#define OLED_PAGES                      _u(8)
#define FRAME_NUM                          2

typedef struct pio_spi_inst {
    PIO pio;
    uint sm;
    uint cs_pin;
} pio_spi_inst_t;

typedef struct{
    uint8_t frame[OLED_SIZE_BYTE];
}frame;

static frame frame_buff[FRAME_NUM];

typedef uint8_t (*func)(uint16_t);

typedef struct position{
    uint8_t x;
    uint8_t y;
}position;


void oled_init(void);
void oled_init_frame(uint8_t *frame);
void oled_clear();
void oled_fill_white();
void oled_render_frame(uint8_t *frame);
void oled_render_frame_dma(uint8_t *frame);
void oled_render_frame_dma_clear(uint8_t *frame);
void oled_set_pixel(uint8_t *frame, position pos,uint8_t on);
void oled_write_char(uint8_t *frame, position pos, uint8_t ch);
void oled_write_char_fix(uint8_t *frame, position pos, uint8_t ch);
void oled_draw_fun(uint8_t *frame,func f,position pos,position pos1);
void oled_write_string(uint8_t *frame, position pos, char *str);
void oled_render_array(uint8_t *buf,uint16_t num);
void oled_draw_line(uint8_t *frame, position pos, position pos1, uint8_t on);

void row_update(int page,uint8_t *line_buf);
void partial_update(int page,uint8_t start_col,uint8_t *data,uint8_t len);

void pio_spi_init(PIO pio, uint sm, uint prog_offs, uint n_bits,float clkdiv, bool cpha, bool cpol, uint pin_sck, uint pin_mosi, uint pin_miso);
void pio_spi_write8_blocking(const pio_spi_inst_t *spi, const uint8_t *src, size_t len);

static inline int oled_write_cmd(uint8_t cmd);
static inline void oled_set_page(uint8_t page);
static inline void oled_set_col(uint8_t start_col);
static inline void oled_fill_screen_pure(uint8_t clo);
static inline int16_t oled_get_font_index(uint8_t ch);
static inline void oled_write_cmd_list(uint8_t *list,unsigned int num);

static inline int16_t tool_fast_abs(int16_t t);
#endif // !__SH1106
