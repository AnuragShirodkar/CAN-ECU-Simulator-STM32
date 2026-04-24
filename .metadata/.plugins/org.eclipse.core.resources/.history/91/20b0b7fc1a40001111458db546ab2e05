/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Engine ECU — STM32F407 Discovery
  * Merged v2.1: Dynamic Temp Logic + Raw ADC Monitoring
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "can.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PD */
#define OVERTEMP_THRESHOLD_C   40.0f
#define RPM_MIN   800U
#define RPM_MAX   15000U
/* USER CODE END PD */

/* USER CODE BEGIN PV */
typedef enum {
    SYS_NORMAL  = 0,
    SYS_FAULT   = 1,
    SYS_LIMP    = 2,
    SYS_RECOVER = 3
} SystemState_t;

/* Shared sensor data */
volatile uint16_t      g_rpm          = RPM_MIN;
volatile uint8_t       g_speed        = 0;
volatile float         g_engTemp      = 25.0f;
volatile uint32_t      g_adcRaw       = 0;

/* State machine variables */
volatile uint8_t       g_faultActive  = 0;
volatile SystemState_t g_sysState     = SYS_NORMAL;
volatile uint32_t      g_faultTimer   = 0;
volatile uint32_t      g_limpTimer    = 0;
volatile uint32_t      g_recoverTimer = 0;

/* Buzzer control */
volatile uint8_t       g_buzzerFlag   = 0;
volatile uint8_t       g_buzzerBeeps  = 0;

/* CAN TX resources */
CAN_TxHeaderTypeDef    g_txHeader;
uint8_t                g_txData[8];
uint32_t               g_txMailbox;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);

/* USER CODE BEGIN PFP */
static void  Task_SensorRead   (void *argument);
static void  Task_CANTransmit  (void *argument);
static void  Task_FaultDetect  (void *argument);
static void  Task_LEDHeartbeat (void *argument);
static float MAX6675_ReadTemp  (void);
static void  UART_Log          (const char *msg);
static void  DTC_Transmit      (uint8_t b1, uint8_t b2, uint8_t b3);
static void  Buzzer_Beep       (uint8_t times);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

static void UART_Log(const char *msg) {
    HAL_UART_Transmit(&huart2, (const uint8_t *)msg, strlen(msg), 200);
}

static void Buzzer_Beep(uint8_t times) {
    for (uint8_t i = 0; i < times; i++) {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);
        osDelay(200);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);
        osDelay(150);
    }
}

static void DTC_Transmit(uint8_t b1, uint8_t b2, uint8_t b3) {
    CAN_TxHeaderTypeDef hdr;
    uint8_t d[3];
    uint32_t mb;
    hdr.StdId = 0x1FF;
    hdr.IDE = CAN_ID_STD;
    hdr.RTR = CAN_RTR_DATA;
    hdr.DLC = 3;
    hdr.TransmitGlobalTime = DISABLE;
    d[0] = b1; d[1] = b2; d[2] = b3;
    HAL_CAN_AddTxMessage(&hcan1, &hdr, d, &mb);
}

/* ── MAX6675 Temperature Read ── */
static float MAX6675_ReadTemp(void) {
    uint8_t rx[2] = {0, 0};
    uint16_t raw;

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET); /* CS LOW */
    osDelay(1);
    if (HAL_SPI_Receive(&hspi2, rx, 2, 100) != HAL_OK) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
        return -2.0f; /* SPI Error */
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);   /* CS HIGH */

    raw = ((uint16_t)rx[0] << 8) | rx[1];
    if (raw & 0x0004) return -1.0f; /* Open circuit */

    raw >>= 3;
    return (float)raw * 0.25f;
}

/* USER CODE END 0 */

int main(void) {
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET); // SPI CS high
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET); // Buzzer off

  CAN_FilterTypeDef flt = {0};
  flt.FilterBank = 0;
  flt.FilterMode = CAN_FILTERMODE_IDMASK;
  flt.FilterScale = CAN_FILTERSCALE_32BIT;
  flt.FilterActivation = ENABLE;
  HAL_CAN_ConfigFilter(&hcan1, &flt);
  HAL_CAN_Start(&hcan1);

  g_txHeader.IDE = CAN_ID_STD;
  g_txHeader.RTR = CAN_RTR_DATA;

  osThreadNew(Task_SensorRead,   NULL, &(const osThreadAttr_t){ .name="Sensor", .stack_size=2048, .priority=osPriorityNormal });
  osThreadNew(Task_CANTransmit,  NULL, &(const osThreadAttr_t){ .name="CAN",    .stack_size=2048, .priority=osPriorityNormal });
  osThreadNew(Task_FaultDetect,  NULL, &(const osThreadAttr_t){ .name="Fault",  .stack_size=2048, .priority=osPriorityAboveNormal });
  osThreadNew(Task_LEDHeartbeat, NULL, &(const osThreadAttr_t){ .name="LED",    .stack_size=1024, .priority=osPriorityBelowNormal });

  /* USER CODE END 2 */

  osKernelInitialize();
  MX_FREERTOS_Init();
  osKernelStart();
  while (1) {}
}

/* USER CODE BEGIN 4 */

/* ── Task_SensorRead ── */
static void Task_SensorRead(void *argument) {
    for (;;) {
        HAL_ADC_Start(&hadc1);
        if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
            g_adcRaw = HAL_ADC_GetValue(&hadc1);
        }
        HAL_ADC_Stop(&hadc1);

        uint16_t joyRPM = (uint16_t)(RPM_MIN + ((g_adcRaw * (uint32_t)(RPM_MAX - RPM_MIN)) / 4095UL));

        switch (g_sysState) {
            case SYS_NORMAL:  g_rpm = joyRPM; break;
            case SYS_LIMP:    g_rpm = 1500U; break;
            case SYS_FAULT:   g_rpm = 0xFFFFU; break;
            case SYS_RECOVER: g_rpm = joyRPM; break;
            default:          g_rpm = RPM_MIN; break;
        }

        if (g_rpm == 0xFFFFU) g_speed = 0;
        else {
            uint32_t s = (uint32_t)g_rpm / 40U;
            g_speed = (s > 250) ? 250 : (uint8_t)s;
        }

        float t = MAX6675_ReadTemp();
        if (t >= 0.0f) g_engTemp = t;

        if (g_buzzerFlag) {
            Buzzer_Beep(g_buzzerBeeps);
            g_buzzerFlag = 0;
        }
        osDelay(50);
    }
}

/* ── Task_CANTransmit ── */
static void Task_CANTransmit(void *argument) {
    uint32_t lastPrint = 0;
    static const char *labels[] = {"NORMAL", "FAULT ", "LIMP  ", "RECOV "};
    for (;;) {
        // CAN: 0x100 (RPM)
        g_txHeader.StdId = 0x100; g_txHeader.DLC = 2;
        g_txData[0] = (g_rpm >> 8) & 0xFF; g_txData[1] = g_rpm & 0xFF;
        HAL_CAN_AddTxMessage(&hcan1, &g_txHeader, g_txData, &g_txMailbox);

        // CAN: 0x102 (Temp x10)
        g_txHeader.StdId = 0x102; g_txHeader.DLC = 2;
        uint16_t tx10 = (uint16_t)(g_engTemp * 10.0f);
        g_txData[0] = (tx10 >> 8) & 0xFF; g_txData[1] = tx10 & 0xFF;
        HAL_CAN_AddTxMessage(&hcan1, &g_txHeader, g_txData, &g_txMailbox);

        uint32_t now = HAL_GetTick();
        if (now - lastPrint >= 500) {
            lastPrint = now;
            char line[150];
            snprintf(line, sizeof(line),
                "[ECU] State:%s | RPM:%5u | Temp:%5.1f C | ADC:%4lu\r\n",
                labels[g_sysState], (g_rpm == 0xFFFF) ? 0 : g_rpm, (double)g_engTemp, g_adcRaw);
            UART_Log(line);
        }
        osDelay(10);
    }
}

/* ── Button Debounce ── */
static uint8_t Button_PressDetected(void) {
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) {
        osDelay(50);
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) {
            while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) osDelay(10);
            return 1;
        }
    }
    return 0;
}

/* ── Task_FaultDetect ── */
static void Task_FaultDetect(void *argument) {
    osDelay(1000);
    for (;;) {
        uint32_t now = HAL_GetTick();
        switch (g_sysState) {
            case SYS_NORMAL:
                g_faultActive = 0;
                if (Button_PressDetected()) {
                    g_sysState = SYS_FAULT; g_faultTimer = now; g_faultActive = 1;
                    g_rpm = 0xFFFF; DTC_Transmit(0x50, 0x03, 0x35);
                    g_buzzerBeeps = 3; g_buzzerFlag = 1;
                }
                if (g_engTemp > OVERTEMP_THRESHOLD_C) {
                    g_sysState = SYS_FAULT; g_faultTimer = now; g_faultActive = 1;
                    DTC_Transmit(0x50, 0x02, 0x17);
                    g_buzzerBeeps = 2; g_buzzerFlag = 1;
                }
                break;
            case SYS_FAULT:
                if (now - g_faultTimer >= 3000) {
                    g_sysState = SYS_LIMP; g_limpTimer = now;
                    g_buzzerBeeps = 1; g_buzzerFlag = 1;
                }
                break;
            case SYS_LIMP:
                if (now - g_limpTimer >= 5000 && g_engTemp < OVERTEMP_THRESHOLD_C) {
                    g_sysState = SYS_RECOVER; g_recoverTimer = now;
                }
                break;
            case SYS_RECOVER:
                if (now - g_recoverTimer >= 2000) {
                    if (g_engTemp < OVERTEMP_THRESHOLD_C) {
                        g_sysState = SYS_NORMAL; g_buzzerBeeps = 2; g_buzzerFlag = 1;
                    } else {
                        g_sysState = SYS_FAULT; g_faultTimer = now;
                    }
                }
                break;
        }
        osDelay(5);
    }
}

/* ── Task_LEDHeartbeat ── */
static void Task_LEDHeartbeat(void *argument) {
    for (;;) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);
        switch (g_sysState) {
            case SYS_NORMAL:  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET); break;
            case SYS_FAULT:   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET); break;
            case SYS_LIMP:    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13); break;
            case SYS_RECOVER: HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12); break;
        }
        osDelay(500);
    }
}

/* USER CODE END 4 */

void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

void Error_Handler(void) {
  __disable_irq();
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
  while (1) {}
}
