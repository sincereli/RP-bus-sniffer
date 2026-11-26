#ifndef ST7789_H
#define ST7789_H

#include <stdint.h>

/* 板载 1.14" ST7789，竖屏 135x240。
 * ST7789 内部 GRAM 是 240x320，135x240 面板需要行列偏移。 */
#define LCD_WIDTH   135
#define LCD_HEIGHT  240
#define LCD_X_OFFSET 52
#define LCD_Y_OFFSET 40

/* RGB565 */
#define LCD_BLACK   0x0000
#define LCD_WHITE   0xFFFF
#define LCD_GREEN   0x07E0
#define LCD_RED     0xF800
#define LCD_YELLOW  0xFFE0
#define LCD_CYAN    0x07FF
#define LCD_GREY    0x8410

void st7789_init(void);

/* LCD 与 TF 卡共用 GPIO10/11（SCK/MOSI），两套外设不能同时接管这对脚。
 * 需要用 SD 卡时先 release，用完再 acquire；acquire 之后屏幕内容还在，
 * 但需要自己重画（SPI 外设是重新初始化的）。 */
void st7789_bus_release(void);
void st7789_bus_acquire(void);

void st7789_fill(uint16_t color);
void st7789_draw_text(int x, int y, const char *text,
                      uint16_t fg, uint16_t bg, uint8_t scale);

#endif /* ST7789_H */
