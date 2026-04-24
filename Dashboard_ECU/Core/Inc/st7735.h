#ifndef __ST7735_H
#define __ST7735_H

#include "main.h"
#include "spi.h"
#include <stdint.h>

/* ---- PIN DEFINITIONS — Board 2 correct pins ---- */
#define ST7735_CS_PORT   GPIOA
#define ST7735_CS_PIN    GPIO_PIN_4   /* PA4 */

#define ST7735_DC_PORT   GPIOA
#define ST7735_DC_PIN    GPIO_PIN_3   /* PA3 */

#define ST7735_RST_PORT  GPIOA
#define ST7735_RST_PIN   GPIO_PIN_2   /* PA2 */

/* ---- DISPLAY SIZE ---- */
#define ST7735_WIDTH   128
#define ST7735_HEIGHT  160

/* ---- COLORS (RGB565) ---- */
#define ST7735_BLACK    0x0000
#define ST7735_WHITE    0xFFFF
#define ST7735_RED      0xF800
#define ST7735_GREEN    0x07E0
#define ST7735_BLUE     0x001F
#define ST7735_YELLOW   0xFFE0
#define ST7735_CYAN     0x07FF
#define ST7735_MAGENTA  0xF81F
#define ST7735_ORANGE   0xFC00
#define ST7735_DARKGREY 0x7BEF
#define ST7735_DARKGREEN 0x03E0

/* ---- FUNCTIONS ---- */
void ST7735_Init(void);
void ST7735_FillScreen(uint16_t color);
void ST7735_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
void ST7735_FillRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void ST7735_DrawRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void ST7735_WriteString(uint16_t x, uint16_t y, const char *str, uint16_t color, uint16_t bgcolor);
void ST7735_WriteChar(uint16_t x, uint16_t y, char ch, uint16_t color, uint16_t bgcolor);

#endif /* __ST7735_H */
