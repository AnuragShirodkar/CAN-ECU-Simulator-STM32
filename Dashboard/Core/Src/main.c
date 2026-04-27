/**
 ******************************************************************************
 * @file    main.c
 * @brief   STM32F407G — CAN TJA1050 | RPM/Speed | TFT ST7735 | Buzzer | LED
 ******************************************************************************
 */

#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* --- Hardware Handles --- */
ADC_HandleTypeDef  hadc1;
SPI_HandleTypeDef  hspi1;
CAN_HandleTypeDef  hcan1;

/* --- TFT ST7735 Defines --- */
#define TFT_CS_PORT   GPIOA
#define TFT_CS_PIN    GPIO_PIN_4
#define TFT_DC_PORT   GPIOC
#define TFT_DC_PIN    GPIO_PIN_4
#define TFT_RST_PORT  GPIOC
#define TFT_RST_PIN   GPIO_PIN_5

#define TFT_CS_LOW()   HAL_GPIO_WritePin(TFT_CS_PORT,  TFT_CS_PIN,  GPIO_PIN_RESET)
#define TFT_CS_HIGH()  HAL_GPIO_WritePin(TFT_CS_PORT,  TFT_CS_PIN,  GPIO_PIN_SET)
#define TFT_DC_LOW()   HAL_GPIO_WritePin(TFT_DC_PORT,  TFT_DC_PIN,  GPIO_PIN_RESET)
#define TFT_DC_HIGH()  HAL_GPIO_WritePin(TFT_DC_PORT,  TFT_DC_PIN,  GPIO_PIN_SET)
#define TFT_RST_LOW()  HAL_GPIO_WritePin(TFT_RST_PORT, TFT_RST_PIN, GPIO_PIN_RESET)
#define TFT_RST_HIGH() HAL_GPIO_WritePin(TFT_RST_PORT, TFT_RST_PIN, GPIO_PIN_SET)

#define BUZZER_PORT   GPIOB
#define BUZZER_PIN    GPIO_PIN_0
#define LED_PORT      GPIOD
#define LED_PIN       GPIO_PIN_12

/* ST7735 Commands */
#define ST7735_SWRESET 0x01
#define ST7735_SLPOUT  0x11
#define ST7735_NORON   0x13
#define ST7735_INVOFF  0x20
#define ST7735_DISPON  0x29
#define ST7735_CASET   0x2A
#define ST7735_RASET   0x2B
#define ST7735_RAMWR   0x2C
#define ST7735_MADCTL  0x36
#define ST7735_COLMOD  0x3A
#define ST7735_FRMCTR1 0xB1
#define ST7735_PWCTR1  0xC0
#define ST7735_PWCTR2  0xC1
#define ST7735_VMCTR1  0xC5

/* Colors */
#define BLACK   0x0000
#define WHITE   0xFFFF
#define RED     0xF800
#define GREEN   0x07E0
#define BLUE    0x001F
#define YELLOW  0xFFE0
#define CYAN    0x07FF
#define ORANGE  0xFD20
#define DKGRAY  0x4208
#define DGRAY   0x2104
#define GRAY    0x8410

#define TFT_WIDTH   128
#define TFT_HEIGHT  160

#define RPM_MAX          15000u
#define RPM_FAULT_LEVEL  14500u
#define SPEED_FACTOR     0.00833f
#define TEMP_MIN         32u
#define TEMP_MAX         36u
#define CAN_TX_ID        0x100u

/* Global State */
static uint32_t g_rpm        = 0;
static uint32_t g_speed      = 0;
static uint32_t g_temp       = 34;
static uint8_t  g_fault      = 0;
static uint32_t g_tick_led   = 0;
static uint8_t  g_led_state  = 0;
static uint8_t  g_prev_fault = 0xFF;

/* 5x7 Font Table */
static const uint8_t Font5x7[][5] = {
  {0x00,0x00,0x00,0x00,0x00}, {0x00,0x00,0x5F,0x00,0x00}, {0x00,0x07,0x00,0x07,0x00},
  {0x14,0x7F,0x14,0x7F,0x14}, {0x24,0x2A,0x7F,0x2A,0x12}, {0x23,0x13,0x08,0x64,0x62},
  {0x36,0x49,0x55,0x22,0x50}, {0x00,0x05,0x03,0x00,0x00}, {0x00,0x1C,0x22,0x41,0x00},
  {0x00,0x41,0x22,0x1C,0x00}, {0x14,0x08,0x3E,0x08,0x14}, {0x08,0x08,0x3E,0x08,0x08},
  {0x00,0x50,0x30,0x00,0x00}, {0x08,0x08,0x08,0x08,0x08}, {0x00,0x60,0x60,0x00,0x00},
  {0x20,0x10,0x08,0x04,0x02}, {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00},
  {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4B,0x31}, {0x18,0x14,0x12,0x7F,0x10},
  {0x27,0x45,0x45,0x45,0x39}, {0x3C,0x4A,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03},
  {0x36,0x49,0x49,0x49,0x36}, {0x06,0x49,0x49,0x29,0x1E}, {0x00,0x36,0x36,0x00,0x00},
  {0x00,0x56,0x36,0x00,0x00}, {0x08,0x14,0x22,0x41,0x00}, {0x14,0x14,0x14,0x14,0x14},
  {0x00,0x41,0x22,0x14,0x08}, {0x02,0x01,0x51,0x09,0x06}, {0x32,0x49,0x79,0x41,0x3E},
  {0x7E,0x11,0x11,0x11,0x7E}, {0x7F,0x49,0x49,0x49,0x36}, {0x3E,0x41,0x41,0x41,0x22},
  {0x7F,0x41,0x41,0x22,0x1C}, {0x7F,0x49,0x49,0x49,0x41}, {0x7F,0x09,0x09,0x09,0x01},
  {0x3E,0x41,0x49,0x49,0x7A}, {0x7F,0x08,0x08,0x08,0x7F}, {0x00,0x41,0x7F,0x41,0x00},
  {0x20,0x40,0x41,0x3F,0x01}, {0x7F,0x08,0x14,0x22,0x41}, {0x7F,0x40,0x40,0x40,0x40},
  {0x7F,0x02,0x0C,0x02,0x7F}, {0x7F,0x04,0x08,0x10,0x7F}, {0x3E,0x41,0x41,0x41,0x3E},
  {0x7F,0x09,0x09,0x09,0x06}, {0x3E,0x41,0x51,0x21,0x5E}, {0x7F,0x09,0x19,0x29,0x46},
  {0x46,0x49,0x49,0x49,0x31}, {0x01,0x01,0x7F,0x01,0x01}, {0x3F,0x40,0x40,0x40,0x3F},
  {0x1F,0x20,0x40,0x20,0x1F}, {0x3F,0x40,0x38,0x40,0x3F}, {0x63,0x14,0x08,0x14,0x63},
  {0x07,0x08,0x70,0x08,0x07}, {0x61,0x51,0x49,0x45,0x43}, {0x00,0x7F,0x41,0x41,0x00},
  {0x02,0x04,0x08,0x10,0x20}, {0x00,0x41,0x41,0x7F,0x00}, {0x04,0x02,0x01,0x02,0x04},
  {0x40,0x40,0x40,0x40,0x40}, {0x00,0x01,0x02,0x04,0x00}, {0x20,0x54,0x54,0x54,0x78},
  {0x7F,0x48,0x44,0x44,0x38}, {0x38,0x44,0x44,0x44,0x20}, {0x38,0x44,0x44,0x48,0x7F},
  {0x38,0x54,0x54,0x54,0x18}, {0x08,0x7E,0x09,0x01,0x02}, {0x0C,0x52,0x52,0x52,0x3E},
  {0x7F,0x08,0x04,0x04,0x78}, {0x00,0x44,0x7D,0x40,0x00}, {0x20,0x40,0x44,0x3D,0x00},
  {0x7F,0x10,0x28,0x44,0x00}, {0x00,0x41,0x7F,0x40,0x00}, {0x7C,0x04,0x18,0x04,0x78},
  {0x7C,0x08,0x04,0x04,0x78}, {0x38,0x44,0x44,0x44,0x38}, {0x7C,0x14,0x14,0x14,0x08},
  {0x08,0x14,0x14,0x18,0x7C}, {0x7C,0x08,0x04,0x04,0x08}, {0x48,0x54,0x54,0x54,0x20},
  {0x04,0x3F,0x44,0x40,0x20}, {0x3C,0x40,0x40,0x20,0x7C}, {0x1C,0x20,0x40,0x20,0x1C},
  {0x3C,0x40,0x30,0x40,0x3C}, {0x44,0x28,0x10,0x28,0x44}, {0x0C,0x50,0x50,0x50,0x3C},
  {0x44,0x64,0x54,0x4C,0x44}
};

/* Prototypes */
static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_CAN1_Init(void);
static void TFT_Init(void);
static void TFT_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color);
static void TFT_DrawString(uint8_t x, uint8_t y, const char *str, uint16_t fg, uint16_t bg, uint8_t scale);
static void TFT_DrawUI_Static(void);
static void TFT_UpdateValues(void);
static uint32_t ADC_ReadRPM(void);
static void CAN_SendFrame(uint32_t rpm, uint32_t speed);

/* Main */
int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_SPI1_Init();
    MX_CAN1_Init();

    HAL_CAN_Start(&hcan1);
    TFT_Init();
    TFT_FillRect(0, 0, TFT_WIDTH, TFT_HEIGHT, BLACK);
    TFT_DrawUI_Static();

    uint32_t last_can_tick = 0;
    while (1) {
        g_rpm = ADC_ReadRPM();
        g_speed = (uint32_t)(g_rpm * SPEED_FACTOR);

        if (g_rpm > RPM_FAULT_LEVEL) {
            g_fault = 1;
            HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
        } else {
            g_fault = 0;
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
        }

        TFT_UpdateValues();

        if ((HAL_GetTick() - last_can_tick) >= 100) {
            last_can_tick = HAL_GetTick();
            CAN_SendFrame(g_rpm, g_speed);
        }
        HAL_Delay(50);
    }
}

/* TFT Functions */
static void TFT_WriteCmd(uint8_t cmd) {
    TFT_DC_LOW(); TFT_CS_LOW();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, 10);
    TFT_CS_HIGH();
}

static void TFT_WriteData(uint8_t data) {
    TFT_DC_HIGH(); TFT_CS_LOW();
    HAL_SPI_Transmit(&hspi1, &data, 1, 10);
    TFT_CS_HIGH();
}

static void TFT_Init(void) {
    TFT_RST_LOW(); HAL_Delay(100); TFT_RST_HIGH(); HAL_Delay(100);
    TFT_WriteCmd(ST7735_SWRESET); HAL_Delay(150);
    TFT_WriteCmd(ST7735_SLPOUT);  HAL_Delay(200);
    TFT_WriteCmd(ST7735_COLMOD);  TFT_WriteData(0x05);
    TFT_WriteCmd(ST7735_DISPON);
}

static void TFT_SetWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) {
    TFT_WriteCmd(ST7735_CASET);
    TFT_WriteData(0x00); TFT_WriteData(x0); TFT_WriteData(0x00); TFT_WriteData(x1);
    TFT_WriteCmd(ST7735_RASET);
    TFT_WriteData(0x00); TFT_WriteData(y0); TFT_WriteData(0x00); TFT_WriteData(y1);
    TFT_WriteCmd(ST7735_RAMWR);
}

static void TFT_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color) {
    TFT_SetWindow(x, y, x + w - 1, y + h - 1);
    uint8_t data[2] = { color >> 8, color & 0xFF };
    TFT_DC_HIGH(); TFT_CS_LOW();
    for (uint32_t i = 0; i < (uint32_t)w * h; i++) {
        HAL_SPI_Transmit(&hspi1, data, 2, 10);
    }
    TFT_CS_HIGH();
}

static void TFT_DrawChar(uint8_t x, uint8_t y, char c, uint16_t fg, uint16_t bg, uint8_t scale) {
    if (c < 32 || c > 122) c = '?';
    const uint8_t *glyph = Font5x7[c - 32];
    for (uint8_t col = 0; col < 5; col++) {
        for (uint8_t row = 0; row < 7; row++) {
            if (glyph[col] & (1 << row))
                TFT_FillRect(x + col * scale, y + row * scale, scale, scale, fg);
            else
                TFT_FillRect(x + col * scale, y + row * scale, scale, scale, bg);
        }
    }
}

static void TFT_DrawString(uint8_t x, uint8_t y, const char *str, uint16_t fg, uint16_t bg, uint8_t scale) {
    while (*str) {
        TFT_DrawChar(x, y, *str++, fg, bg, scale);
        x += 6 * scale;
    }
}

/* UI Logic */
static void TFT_DrawUI_Static(void) {
    TFT_FillRect(0, 0, TFT_WIDTH, 18, BLUE);
    TFT_DrawString(10, 5, "DASHBOARD", WHITE, BLUE, 1);
}

static void TFT_UpdateValues(void) {
    char buf[10];
    snprintf(buf, sizeof(buf), "%05lu", g_rpm);
    TFT_DrawString(20, 40, buf, g_fault ? RED : GREEN, BLACK, 3);
}

/* Hardware Config */
static uint32_t ADC_ReadRPM(void) {
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint32_t val = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return (val * RPM_MAX) / 4095;
}

static void CAN_SendFrame(uint32_t rpm, uint32_t speed) {
    CAN_TxHeaderTypeDef hdr = { .StdId = CAN_TX_ID, .DLC = 8, .IDE = CAN_ID_STD, .RTR = CAN_RTR_DATA };
    uint8_t data[8] = { rpm >> 8, rpm & 0xFF, speed >> 8, speed & 0xFF, g_fault, g_temp, 0xAA, 0x55 };
    uint32_t mailbox;
    HAL_CAN_AddTxMessage(&hcan1, &hdr, data, &mailbox);
}

/* Standard STM32 Configuration Blocks (SystemClock, GPIO, ADC, SPI, CAN Init)
   ... Keep these as they were in your original code ...
*/

static void SystemClock_Config(void) {
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM = 8; osc.PLL.PLLN = 336; osc.PLL.PLLP = 2;
    HAL_RCC_OscConfig(&osc);
    clk.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV4;
    clk.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5);
}

static void MX_GPIO_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE(); __HAL_RCC_GPIOD_CLK_ENABLE();
    GPIO_InitTypeDef init = {0};
    init.Mode = GPIO_MODE_OUTPUT_PP; init.Pull = GPIO_NOPULL; init.Speed = GPIO_SPEED_FREQ_HIGH;
    init.Pin = TFT_CS_PIN; HAL_GPIO_Init(GPIOA, &init);
    init.Pin = TFT_DC_PIN | TFT_RST_PIN; HAL_GPIO_Init(GPIOC, &init);
    init.Pin = BUZZER_PIN; HAL_GPIO_Init(GPIOB, &init);
    init.Pin = LED_PIN; HAL_GPIO_Init(GPIOD, &init);

    init.Pin = GPIO_PIN_5 | GPIO_PIN_7; init.Mode = GPIO_MODE_AF_PP; init.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &init);
    init.Pin = GPIO_PIN_0 | GPIO_PIN_1; init.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOD, &init);
}

static void MX_ADC1_Init(void) {
    __HAL_RCC_ADC1_CLK_ENABLE();
    hadc1.Instance = ADC1;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    HAL_ADC_Init(&hadc1);
    ADC_ChannelConfTypeDef ch = { .Channel = ADC_CHANNEL_1, .Rank = 1, .SamplingTime = ADC_SAMPLETIME_84CYCLES };
    HAL_ADC_ConfigChannel(&hadc1, &ch);
}

static void MX_SPI1_Init(void) {
    __HAL_RCC_SPI1_CLK_ENABLE();
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    HAL_SPI_Init(&hspi1);
}

static void MX_CAN1_Init(void) {
    __HAL_RCC_CAN1_CLK_ENABLE();
    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 6;
    hcan1.Init.Mode = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_6TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_7TQ;
    HAL_CAN_Init(&hcan1);
    CAN_FilterTypeDef f = { .FilterBank = 0, .FilterMode = CAN_FILTERMODE_IDMASK, .FilterScale = CAN_FILTERSCALE_32BIT, .FilterActivation = ENABLE };
    HAL_CAN_ConfigFilter(&hcan1, &f);
}

void Error_Handler(void) { while(1); }
