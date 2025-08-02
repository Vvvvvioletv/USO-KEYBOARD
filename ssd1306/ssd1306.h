#ifndef __SH1106
#define __SH1106

#include <stdint.h>

#define OLED_WIDTH  128
#define OLED_HEIGHT 64
#define OLED_HEIGHT_BYTE    (OLED_HEIGHT / 8)
#define OLED_WIDTH_BYTE     (OLED_WIDTH / 8)
#define OLED_SIZE           (OLED_HEIGHT * OLED_WIDTH) 
#define OLED_SIZE_BYTE      (OLED_SIZE / 8)

#define PIN_SCL PICO_DEFAULT_I2C_SCL_PIN 
#define PIN_SDA PICO_DEFAULT_I2C_SDA_PIN

#define OLED_I2C_ADDR 0x3C
#define OLED_BAUD (1.5 * 1000000)

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


typedef struct{
    uint8_t frame[OLED_SIZE_BYTE];
}frame;

static frame frameBuff[FRAME_NUM];

typedef uint8_t (*func)(uint16_t);


void OLED_Init(void);
void OLED_initFrame(uint8_t *frame);
void OLED_Clear();
void OLED_RenderFrame(uint8_t *frame);
void OLED_RenderFrame_DMA(uint8_t *frame);
void OLED_RenderFrame_DMA_Clear(uint8_t *frame);
void OLED_setPixel(uint8_t *frame, int_fast16_t x, int_fast16_t y, uint8_t on);
void OLED_WriteChar(uint8_t *frame, int_fast16_t x, int_fast16_t y, uint8_t ch);
void OLED_WriteChar_fix(uint8_t *frame, int_fast16_t x, int_fast16_t y, uint8_t ch);
void OLED_DrawFun(uint8_t *frame,func f,uint8_t x1,uint8_t x2);
void OLED_WriteString(uint8_t *frame, int_fast16_t x, int_fast16_t y, char *str);
void OLED_RenderArray(uint8_t *buf,uint16_t num);
void OLED_DrawLine(uint8_t *frame, int_fast16_t x0, int_fast16_t y0, int_fast16_t x1, int_fast16_t y1, uint8_t on);


static inline int OLED_WriteCmd(uint8_t cmd);
static inline void OLED_Set_Page(uint8_t page);
static inline void OLED_Fill_Screen_Pure(uint8_t clo);
static inline int_fast16_t OLED_GetFontIndex(uint8_t ch);
static inline void OLED_WriteCmdList(uint8_t *list,unsigned int num);

static inline int_fast16_t tool_Fast_abs(int_fast16_t t);
#endif // !__SH1106
