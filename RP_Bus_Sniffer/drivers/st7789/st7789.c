#include <stdbool.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"

#include "board.h"
#include "st7789.h"

#define LCD_SPI      spi1
#define LCD_SPI_HZ   (24 * 1000 * 1000)
#define LCD_MAX_SCALE 3

/*
 * 5x7 点阵字库，索引 = ASCII - 32，覆盖 0x20..0x5F。
 * 每个字符 5 列，每列 bit0 为最上面一行。
 * 小写字母在 draw_text 里统一转成大写。
 */
static const uint8_t font5x7[64][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, /* space */
    {0x00, 0x00, 0x5F, 0x00, 0x00}, /* ! */
    {0x00, 0x07, 0x00, 0x07, 0x00}, /* " */
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, /* # */
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, /* $ */
    {0x23, 0x13, 0x08, 0x64, 0x62}, /* % */
    {0x36, 0x49, 0x55, 0x22, 0x50}, /* & */
    {0x00, 0x05, 0x03, 0x00, 0x00}, /* ' */
    {0x00, 0x1C, 0x22, 0x41, 0x00}, /* ( */
    {0x00, 0x41, 0x22, 0x1C, 0x00}, /* ) */
    {0x14, 0x08, 0x3E, 0x08, 0x14}, /* * */
    {0x08, 0x08, 0x3E, 0x08, 0x08}, /* + */
    {0x00, 0x50, 0x30, 0x00, 0x00}, /* , */
    {0x08, 0x08, 0x08, 0x08, 0x08}, /* - */
    {0x00, 0x60, 0x60, 0x00, 0x00}, /* . */
    {0x20, 0x10, 0x08, 0x04, 0x02}, /* / */
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, /* 0 */
    {0x00, 0x42, 0x7F, 0x40, 0x00}, /* 1 */
    {0x42, 0x61, 0x51, 0x49, 0x46}, /* 2 */
    {0x21, 0x41, 0x45, 0x4B, 0x31}, /* 3 */
    {0x18, 0x14, 0x12, 0x7F, 0x10}, /* 4 */
    {0x27, 0x45, 0x45, 0x45, 0x39}, /* 5 */
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, /* 6 */
    {0x01, 0x71, 0x09, 0x05, 0x03}, /* 7 */
    {0x36, 0x49, 0x49, 0x49, 0x36}, /* 8 */
    {0x06, 0x49, 0x49, 0x29, 0x1E}, /* 9 */
    {0x00, 0x36, 0x36, 0x00, 0x00}, /* : */
    {0x00, 0x56, 0x36, 0x00, 0x00}, /* ; */
    {0x08, 0x14, 0x22, 0x41, 0x00}, /* < */
    {0x14, 0x14, 0x14, 0x14, 0x14}, /* = */
    {0x00, 0x41, 0x22, 0x14, 0x08}, /* > */
    {0x02, 0x01, 0x51, 0x09, 0x06}, /* ? */
    {0x32, 0x49, 0x79, 0x41, 0x3E}, /* @ */
    {0x7C, 0x12, 0x11, 0x12, 0x7C}, /* A */
    {0x7F, 0x49, 0x49, 0x49, 0x36}, /* B */
    {0x3E, 0x41, 0x41, 0x41, 0x22}, /* C */
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, /* D */
    {0x7F, 0x49, 0x49, 0x49, 0x41}, /* E */
    {0x7F, 0x09, 0x09, 0x09, 0x01}, /* F */
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, /* G */
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, /* H */
    {0x00, 0x41, 0x7F, 0x41, 0x00}, /* I */
    {0x20, 0x40, 0x41, 0x3F, 0x01}, /* J */
    {0x7F, 0x08, 0x14, 0x22, 0x41}, /* K */
    {0x7F, 0x40, 0x40, 0x40, 0x40}, /* L */
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, /* M */
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, /* N */
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, /* O */
    {0x7F, 0x09, 0x09, 0x09, 0x06}, /* P */
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, /* Q */
    {0x7F, 0x09, 0x19, 0x29, 0x46}, /* R */
    {0x46, 0x49, 0x49, 0x49, 0x31}, /* S */
    {0x01, 0x01, 0x7F, 0x01, 0x01}, /* T */
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, /* U */
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, /* V */
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, /* W */
    {0x63, 0x14, 0x08, 0x14, 0x63}, /* X */
    {0x07, 0x08, 0x70, 0x08, 0x07}, /* Y */
    {0x61, 0x51, 0x49, 0x45, 0x43}, /* Z */
    {0x00, 0x7F, 0x41, 0x41, 0x00}, /* [ */
    {0x02, 0x04, 0x08, 0x10, 0x20}, /* \ */
    {0x00, 0x41, 0x41, 0x7F, 0x00}, /* ] */
    {0x04, 0x02, 0x01, 0x02, 0x04}, /* ^ */
    {0x40, 0x40, 0x40, 0x40, 0x40}, /* _ */
};

/* LCD 驱动只在 Core 0 调用，不需要重入保护 */
static uint8_t s_pixels[6 * LCD_MAX_SCALE * 8 * LCD_MAX_SCALE * 2];

static inline void cs_low(void)  { gpio_put(PIN_LCD_CS, 0); }
static inline void cs_high(void) { gpio_put(PIN_LCD_CS, 1); }
static inline void dc_cmd(void)  { gpio_put(PIN_LCD_DC, 0); }
static inline void dc_data(void) { gpio_put(PIN_LCD_DC, 1); }

static void lcd_tx(const uint8_t *buf, size_t len)
{
    spi_write_blocking(LCD_SPI, buf, len);
}

static void lcd_write_cmd(uint8_t cmd, const uint8_t *data, size_t len)
{
    cs_low();
    dc_cmd();
    lcd_tx(&cmd, 1);
    if (len) {
        dc_data();
        lcd_tx(data, len);
    }
    cs_high();
}

static void lcd_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t b[4];
    uint16_t xs = (uint16_t)(x0 + LCD_X_OFFSET);
    uint16_t xe = (uint16_t)(x1 + LCD_X_OFFSET);
    uint16_t ys = (uint16_t)(y0 + LCD_Y_OFFSET);
    uint16_t ye = (uint16_t)(y1 + LCD_Y_OFFSET);

    cs_low();

    dc_cmd();
    uint8_t cmd = 0x2A;
    lcd_tx(&cmd, 1);
    dc_data();
    b[0] = xs >> 8; b[1] = xs & 0xFF;
    b[2] = xe >> 8; b[3] = xe & 0xFF;
    lcd_tx(b, 4);

    dc_cmd();
    cmd = 0x2B;
    lcd_tx(&cmd, 1);
    dc_data();
    b[0] = ys >> 8; b[1] = ys & 0xFF;
    b[2] = ye >> 8; b[3] = ye & 0xFF;
    lcd_tx(b, 4);

    dc_cmd();
    cmd = 0x2C;
    lcd_tx(&cmd, 1);
    dc_data();
    /* CS 保持低，由调用者发送像素数据后拉高 */
}

void st7789_init(void)
{
    gpio_init(PIN_LCD_CS);
    gpio_init(PIN_LCD_DC);
    gpio_init(PIN_LCD_BL);
    gpio_set_dir(PIN_LCD_CS, GPIO_OUT);
    gpio_set_dir(PIN_LCD_DC, GPIO_OUT);
    gpio_set_dir(PIN_LCD_BL, GPIO_OUT);
    gpio_put(PIN_LCD_CS, 1);
    gpio_put(PIN_LCD_DC, 1);
    gpio_put(PIN_LCD_BL, LCD_BL_OFF_LEVEL);

    gpio_set_function(PIN_LCD_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_LCD_MOSI, GPIO_FUNC_SPI);

    spi_init(LCD_SPI, LCD_SPI_HZ);
    spi_set_format(LCD_SPI, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    static const uint8_t b2[]  = {0x0C, 0x0C, 0x00, 0x33, 0x33};
    static const uint8_t b7[]  = {0x35};
    static const uint8_t bb[]  = {0x19};
    static const uint8_t c0[]  = {0x2C};
    static const uint8_t c2[]  = {0x01};
    static const uint8_t c3[]  = {0x12};
    static const uint8_t c4[]  = {0x20};
    static const uint8_t c6[]  = {0x0F};
    static const uint8_t d0[]  = {0xA4, 0xA1};
    static const uint8_t e0[]  = {0xD0, 0x04, 0x0D, 0x11, 0x13, 0x2B, 0x3F,
                                  0x54, 0x4C, 0x18, 0x0D, 0x0B, 0x1F, 0x23};
    static const uint8_t e1[]  = {0xD0, 0x04, 0x0C, 0x11, 0x13, 0x2C, 0x3F,
                                  0x44, 0x51, 0x2F, 0x1F, 0x1F, 0x20, 0x23};
    static const uint8_t colmod[] = {0x55};
    static const uint8_t madctl[] = {0x00};

    lcd_write_cmd(0x01, NULL, 0);       /* software reset */
    sleep_ms(150);
    lcd_write_cmd(0x11, NULL, 0);       /* sleep out */
    sleep_ms(120);

    lcd_write_cmd(0x3A, colmod, sizeof(colmod)); /* RGB565 */
    lcd_write_cmd(0x36, madctl, sizeof(madctl)); /* portrait, RGB */
    lcd_write_cmd(0xB2, b2, sizeof(b2));
    lcd_write_cmd(0xB7, b7, sizeof(b7));
    lcd_write_cmd(0xBB, bb, sizeof(bb));
    lcd_write_cmd(0xC0, c0, sizeof(c0));
    lcd_write_cmd(0xC2, c2, sizeof(c2));
    lcd_write_cmd(0xC3, c3, sizeof(c3));
    lcd_write_cmd(0xC4, c4, sizeof(c4));
    lcd_write_cmd(0xC6, c6, sizeof(c6));
    lcd_write_cmd(0xD0, d0, sizeof(d0));
    lcd_write_cmd(0xE0, e0, sizeof(e0));
    lcd_write_cmd(0xE1, e1, sizeof(e1));

    lcd_write_cmd(0x21, NULL, 0);       /* inversion on（多数 1.14" 面板需要） */
    lcd_write_cmd(0x13, NULL, 0);       /* normal display */
    lcd_write_cmd(0x29, NULL, 0);       /* display on */
    sleep_ms(20);

    gpio_put(PIN_LCD_BL, LCD_BL_ON_LEVEL);
}

void st7789_fill(uint16_t color)
{
    uint8_t buf[128];
    for (int i = 0; i < 64; i++) {
        buf[2 * i] = (uint8_t)(color >> 8);
        buf[2 * i + 1] = (uint8_t)(color & 0xFF);
    }

    lcd_window(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1);
    uint32_t left = (uint32_t)LCD_WIDTH * LCD_HEIGHT;
    while (left) {
        uint32_t n = left > 64 ? 64 : left;
        lcd_tx(buf, n * 2);
        left -= n;
    }
    cs_high();
}

/* 画一个字符：6x8 单元（含 1 像素间隔），每个点放大 scale 倍 */
static void draw_char(int x, int y, char ch, uint16_t fg, uint16_t bg,
                      uint8_t scale)
{
    if (ch < 32 || ch > 95) {
        ch = '?';
    }
    const uint8_t *glyph = font5x7[ch - 32];

    uint16_t w = (uint16_t)(6 * scale);
    uint16_t h = (uint16_t)(8 * scale);
    uint32_t n = 0;

    for (uint16_t py = 0; py < h; py++) {
        uint16_t row = py / scale;
        for (uint16_t px = 0; px < w; px++) {
            uint16_t col = px / scale;
            bool on = (col < 5 && row < 7) &&
                      ((glyph[col] >> row) & 1u);
            uint16_t c = on ? fg : bg;
            s_pixels[n++] = (uint8_t)(c >> 8);
            s_pixels[n++] = (uint8_t)(c & 0xFF);
        }
    }

    lcd_window((uint16_t)x, (uint16_t)y,
               (uint16_t)(x + w - 1), (uint16_t)(y + h - 1));
    lcd_tx(s_pixels, (size_t)w * h * 2);
    cs_high();
}

void st7789_draw_text(int x, int y, const char *text,
                      uint16_t fg, uint16_t bg, uint8_t scale)
{
    if (scale == 0) {
        scale = 1;
    } else if (scale > LCD_MAX_SCALE) {
        scale = LCD_MAX_SCALE;
    }

    int cx = x;
    while (*text) {
        char ch = *text++;
        if (ch >= 'a' && ch <= 'z') {
            ch = (char)(ch - 'a' + 'A');
        }
        if (cx + 6 * scale > LCD_WIDTH) {
            break;
        }
        draw_char(cx, y, ch, fg, bg, scale);
        cx += 6 * scale;
    }
}
