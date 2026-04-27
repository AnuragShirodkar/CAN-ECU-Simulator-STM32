#include "st7735.h"
#include "fonts.h"
#include "spi.h"
#include <string.h>

/* ---- LOW LEVEL ---- */
static void ST7735_Select(void) {
    HAL_GPIO_WritePin(ST7735_CS_PORT, ST7735_CS_PIN, GPIO_PIN_RESET);
}

static void ST7735_Unselect(void) {
    HAL_GPIO_WritePin(ST7735_CS_PORT, ST7735_CS_PIN, GPIO_PIN_SET);
}

static void ST7735_DC_Cmd(void) {
    HAL_GPIO_WritePin(ST7735_DC_PORT, ST7735_DC_PIN, GPIO_PIN_RESET);
}

static void ST7735_DC_Data(void) {
    HAL_GPIO_WritePin(ST7735_DC_PORT, ST7735_DC_PIN, GPIO_PIN_SET);
}

static void ST7735_Reset(void) {
    HAL_GPIO_WritePin(ST7735_RST_PORT, ST7735_RST_PIN, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(ST7735_RST_PORT, ST7735_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(120);
}

static void ST7735_WriteCmd(uint8_t cmd) {
    ST7735_Select();
    ST7735_DC_Cmd();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    ST7735_Unselect();
}

static void ST7735_WriteData(uint8_t *data, uint16_t size) {
    ST7735_Select();
    ST7735_DC_Data();
    HAL_SPI_Transmit(&hspi1, data, size, HAL_MAX_DELAY);
    ST7735_Unselect();
}

static void ST7735_WriteData8(uint8_t data) {
    ST7735_WriteData(&data, 1);
}

static void ST7735_WriteData16(uint16_t data) {
    uint8_t buf[2] = { data >> 8, data & 0xFF };
    ST7735_WriteData(buf, 2);
}

/* ---- SET ADDRESS WINDOW ---- */
static void ST7735_SetAddrWindow(uint16_t x0, uint16_t y0,
                                  uint16_t x1, uint16_t y1)
{
    ST7735_WriteCmd(0x2A); /* Column address */
    ST7735_WriteData16(x0);
    ST7735_WriteData16(x1);

    ST7735_WriteCmd(0x2B); /* Row address */
    ST7735_WriteData16(y0);
    ST7735_WriteData16(y1);

    ST7735_WriteCmd(0x2C); /* Write to RAM */
}

/* ---- INIT ---- */
void ST7735_Init(void)
{
    /* CS and RST idle state */
    HAL_GPIO_WritePin(ST7735_CS_PORT,  ST7735_CS_PIN,  GPIO_PIN_SET);
    HAL_GPIO_WritePin(ST7735_RST_PORT, ST7735_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(10);

    ST7735_Reset();

    /* Software reset */
    ST7735_WriteCmd(0x01);
    HAL_Delay(150);

    /* Sleep out */
    ST7735_WriteCmd(0x11);
    HAL_Delay(500);

    /* Frame rate control */
    ST7735_WriteCmd(0xB1);
    ST7735_WriteData8(0x01);
    ST7735_WriteData8(0x2C);
    ST7735_WriteData8(0x2D);

    /* Power control */
    ST7735_WriteCmd(0xC0);
    ST7735_WriteData8(0xA2);
    ST7735_WriteData8(0x02);
    ST7735_WriteData8(0x84);

    ST7735_WriteCmd(0xC1);
    ST7735_WriteData8(0xC5);

    ST7735_WriteCmd(0xC2);
    ST7735_WriteData8(0x0A);
    ST7735_WriteData8(0x00);

    /* VCOM */
    ST7735_WriteCmd(0xC5);
    ST7735_WriteData8(0x0E);

    /* Memory access control — sets orientation */
    /* 0x60 = portrait, RGB order               */
    ST7735_WriteCmd(0x36);
    ST7735_WriteData8(0x60);

    /* Color mode — 16bit RGB565 */
    ST7735_WriteCmd(0x3A);
    ST7735_WriteData8(0x05);

    /* Gamma */
    ST7735_WriteCmd(0xE0);
    uint8_t gamma_p[] = {0x0F,0x1A,0x0F,0x18,0x2F,
                         0x28,0x20,0x22,0x1F,0x1B,
                         0x23,0x37,0x00,0x07,0x02,0x10};
    ST7735_WriteData(gamma_p, 16);

    ST7735_WriteCmd(0xE1);
    uint8_t gamma_n[] = {0x0F,0x1B,0x0F,0x17,0x33,
                         0x2C,0x29,0x2E,0x30,0x30,
                         0x39,0x3F,0x00,0x07,0x03,0x10};
    ST7735_WriteData(gamma_n, 16);

    /* Display ON */
    ST7735_WriteCmd(0x29);
    HAL_Delay(100);

    /* Clear screen to black */
    ST7735_FillScreen(ST7735_BLACK);
}

/* ---- DRAW PIXEL ---- */
void ST7735_DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
    if(x >= ST7735_WIDTH || y >= ST7735_HEIGHT) return;
    ST7735_SetAddrWindow(x, y, x, y);
    ST7735_WriteData16(color);
}

/* ---- FILL SCREEN — fast using line buffer ---- */
void ST7735_FillScreen(uint16_t color)
{
    ST7735_SetAddrWindow(0, 0, ST7735_WIDTH-1, ST7735_HEIGHT-1);

    /* Send one row at a time — much faster than pixel by pixel */
    uint8_t lineBuf[ST7735_WIDTH * 2];
    for(uint16_t i = 0; i < ST7735_WIDTH; i++) {
        lineBuf[i*2]   = color >> 8;
        lineBuf[i*2+1] = color & 0xFF;
    }

    ST7735_Select();
    ST7735_DC_Data();
    for(uint16_t row = 0; row < ST7735_HEIGHT; row++) {
        HAL_SPI_Transmit(&hspi1, lineBuf, sizeof(lineBuf), HAL_MAX_DELAY);
    }
    ST7735_Unselect();
}

/* ---- FILL RECTANGLE ---- */
void ST7735_FillRectangle(uint16_t x, uint16_t y,
                           uint16_t w, uint16_t h, uint16_t color)
{
    if(x + w > ST7735_WIDTH)  w = ST7735_WIDTH  - x;
    if(y + h > ST7735_HEIGHT) h = ST7735_HEIGHT - y;

    ST7735_SetAddrWindow(x, y, x+w-1, y+h-1);

    uint8_t buf[2] = { color >> 8, color & 0xFF };
    ST7735_Select();
    ST7735_DC_Data();
    for(uint32_t i = 0; i < (uint32_t)w * h; i++) {
        HAL_SPI_Transmit(&hspi1, buf, 2, HAL_MAX_DELAY);
    }
    ST7735_Unselect();
}

/* ---- DRAW RECTANGLE OUTLINE ---- */
void ST7735_DrawRectangle(uint16_t x, uint16_t y,
                           uint16_t w, uint16_t h, uint16_t color)
{
    /* Top and bottom lines */
    ST7735_FillRectangle(x,       y,       w, 1, color);
    ST7735_FillRectangle(x,       y+h-1,   w, 1, color);
    /* Left and right lines */
    ST7735_FillRectangle(x,       y,       1, h, color);
    ST7735_FillRectangle(x+w-1,   y,       1, h, color);
}

/* ---- WRITE CHARACTER ---- */
void ST7735_WriteChar(uint16_t x, uint16_t y, char ch,
                       uint16_t color, uint16_t bgcolor)
{
    if(x + 5 >= ST7735_WIDTH || y + 7 >= ST7735_HEIGHT) return;
    if(ch < 32 || ch > 127) ch = '?';

    /* Draw 5x7 character using font array */
    for(uint8_t col = 0; col < 5; col++)
    {
        uint8_t line = font5x7[(uint8_t)(ch - 32)][col];
        for(uint8_t row = 0; row < 8; row++)
        {
            if(line & 0x01)
                ST7735_DrawPixel(x + col, y + row, color);
            else
                ST7735_DrawPixel(x + col, y + row, bgcolor);
            line >>= 1;
        }
    }

    /* Space between characters */
    for(uint8_t row = 0; row < 8; row++)
        ST7735_DrawPixel(x + 5, y + row, bgcolor);
}

/* ---- WRITE STRING ---- */
void ST7735_WriteString(uint16_t x, uint16_t y, const char *str,
                         uint16_t color, uint16_t bgcolor)
{
    while(*str)
    {
        if(x + 6 > ST7735_WIDTH)  /* wrap to next line */
        {
            x  = 0;
            y += 9; /* 8px char + 1px gap */
        }
        if(y + 8 > ST7735_HEIGHT) return; /* off screen */

        ST7735_WriteChar(x, y, *str, color, bgcolor);
        x   += 6;
        str++;
    }
}
