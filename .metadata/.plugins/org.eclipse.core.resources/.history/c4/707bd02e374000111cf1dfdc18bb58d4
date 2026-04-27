/* ============================================================
   Engine ECU — Board 1 — STM32F407 Discovery
   CAN Bus ECU Simulator — Final v3.0

   Features:
   - Auto fault detection at 40°C (no button needed)
   - Auto limp mode: RPM locked at 1500 when overtemp
   - Auto recovery when temp drops below 35°C
   - MAX6675 thermocouple temperature
   - Joystick RPM control (800–15000)
   - FreeRTOS 4-task architecture
   - CAN frames: 0x100 RPM, 0x101 Speed, 0x102 Temp,
                  0x103 Relay, 0x104 State, 0x1FF DTC
   ============================================================ */

#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "can.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

#include <stdio.h>
#include <string.h>

/* ── Thresholds ── */
#define OVERTEMP_FAULT_C     40.0f   /* above this → FAULT */
#define OVERTEMP_RECOVER_C   35.0f   /* below this → can recover */
#define RPM_MIN              800U
#define RPM_MAX              15000U
#define LIMP_RPM             1500U

/* ── System state machine ── */
typedef enum {
    SYS_NORMAL  = 0,
    SYS_FAULT   = 1,
    SYS_LIMP    = 2,
    SYS_RECOVER = 3
} SystemState_t;

/* ── Global shared variables ── */
volatile uint16_t      g_rpm          = RPM_MIN;
volatile uint8_t       g_speed        = 0;
volatile float         g_engTemp      = 25.0f;
volatile uint32_t      g_adcRaw       = 2048;   /* mid joystick default */
volatile uint8_t       g_faultActive  = 0;
volatile SystemState_t g_sysState     = SYS_NORMAL;
volatile uint32_t      g_faultTimer   = 0;
volatile uint32_t      g_limpTimer    = 0;
volatile uint32_t      g_recoverTimer = 0;
volatile uint8_t       g_buzzerFlag   = 0;
volatile uint8_t       g_buzzerBeeps  = 0;

/* ── CAN TX ── */
CAN_TxHeaderTypeDef    g_txHeader;
uint8_t                g_txData[8];
uint32_t               g_txMailbox;

/* ── Function prototypes ── */
static void  Task_SensorRead   (void *argument);
static void  Task_CANTransmit  (void *argument);
static void  Task_FaultDetect  (void *argument);
static void  Task_LEDHeartbeat (void *argument);
static float MAX6675_ReadTemp  (void);
static void  UART_Log          (const char *msg);
static void  DTC_Transmit      (uint8_t b1, uint8_t b2, uint8_t b3);
static void  Buzzer_Beep       (uint8_t times);

/* ================================================================
   HELPER FUNCTIONS
   ================================================================ */

static void UART_Log(const char *msg) {
    HAL_UART_Transmit(&huart2, (const uint8_t *)msg, strlen(msg), 200);
}

static void Buzzer_Beep(uint8_t times) {
    for (uint8_t i = 0; i < times; i++) {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);
        osDelay(300);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);
        osDelay(200);
    }
}

static void DTC_Transmit(uint8_t b1, uint8_t b2, uint8_t b3) {
    CAN_TxHeaderTypeDef hdr;
    uint8_t  d[3] = {b1, b2, b3};
    uint32_t mb;
    hdr.StdId              = 0x1FF;
    hdr.IDE                = CAN_ID_STD;
    hdr.RTR                = CAN_RTR_DATA;
    hdr.DLC                = 3;
    hdr.TransmitGlobalTime = DISABLE;
    HAL_CAN_AddTxMessage(&hcan1, &hdr, d, &mb);
}

static float MAX6675_ReadTemp(void) {
    uint8_t  rx[2] = {0, 0};
    uint16_t raw;

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET); /* CS LOW */
    osDelay(1);
    if (HAL_SPI_Receive(&hspi2, rx, 2, 100) != HAL_OK) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
        return -2.0f; /* SPI error */
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET); /* CS HIGH */

    raw = ((uint16_t)rx[0] << 8) | rx[1];

    /* Debug raw SPI bytes */
    char dbg[40];
    snprintf(dbg, sizeof(dbg), "[SPI] Raw: %02X %02X\r\n", rx[0], rx[1]);
    UART_Log(dbg);

    if (raw & 0x0004) {
        UART_Log("[TEMP] Open circuit — check thermocouple\r\n");
        return -1.0f;
    }

    raw >>= 3;  /* bits [14:3] */
    return (float)raw * 0.25f;
}

/* ================================================================
   MAIN
   ================================================================ */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_CAN1_Init();
    MX_SPI2_Init();
    MX_USART2_UART_Init();

    /* Peripherals init */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);   /* MAX6675 CS high */
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0,  GPIO_PIN_RESET); /* Buzzer off */

    /* CAN filter — accept all */
    CAN_FilterTypeDef flt = {0};
    flt.FilterBank           = 0;
    flt.FilterMode           = CAN_FILTERMODE_IDMASK;
    flt.FilterScale          = CAN_FILTERSCALE_32BIT;
    flt.FilterActivation     = ENABLE;
    flt.FilterFIFOAssignment = CAN_RX_FIFO0;
    flt.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan1, &flt);
    HAL_CAN_Start(&hcan1);

    /* TX header defaults */
    g_txHeader.IDE                = CAN_ID_STD;
    g_txHeader.RTR                = CAN_RTR_DATA;
    g_txHeader.TransmitGlobalTime = DISABLE;

    /* Boot messages */
    UART_Log("\r\n================================================\r\n");
    UART_Log("  ENGINE ECU  |  STM32F407  |  v3.0 FINAL\r\n");
    UART_Log("  Auto fault at 40C | Limp at 1500 RPM\r\n");
    UART_Log("  Recovery at 35C   | No button needed\r\n");
    UART_Log("================================================\r\n\r\n");

    /* Startup beep */
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_Delay(300);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);

    /* Create FreeRTOS tasks */
    osThreadNew(Task_SensorRead,   NULL, &(const osThreadAttr_t){
        .name="Sensor", .stack_size=2048, .priority=osPriorityNormal });

    osThreadNew(Task_CANTransmit,  NULL, &(const osThreadAttr_t){
        .name="CAN", .stack_size=2048, .priority=osPriorityNormal });

    osThreadNew(Task_FaultDetect,  NULL, &(const osThreadAttr_t){
        .name="Fault", .stack_size=2048, .priority=osPriorityAboveNormal });

    osThreadNew(Task_LEDHeartbeat, NULL, &(const osThreadAttr_t){
        .name="LED", .stack_size=1024, .priority=osPriorityBelowNormal });

    osKernelInitialize();
    MX_FREERTOS_Init();
    osKernelStart();
    while (1) {}
}

/* ================================================================
   TASK 1 — Sensor Read (50ms)
   Reads joystick ADC + MAX6675 temp
   ================================================================ */
static void Task_SensorRead(void *argument)
{
    for (;;)
    {
        /* Read joystick ADC */
        HAL_ADC_Start(&hadc1);
        if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
            g_adcRaw = HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);

        /* Map ADC to RPM */
        uint16_t joyRPM = (uint16_t)(RPM_MIN +
            ((g_adcRaw * (uint32_t)(RPM_MAX - RPM_MIN)) / 4095UL));

        /* State selects RPM */
        switch (g_sysState) {
            case SYS_NORMAL:  g_rpm = joyRPM;    break;
            case SYS_LIMP:    g_rpm = LIMP_RPM;  break;
            case SYS_FAULT:   g_rpm = 0xFFFFU;   break;
            case SYS_RECOVER: g_rpm = LIMP_RPM;  break;
            default:          g_rpm = RPM_MIN;    break;
        }

        /* Speed: rpm/40, capped at 250 */
        if (g_rpm == 0xFFFFU) {
            g_speed = 0;
        } else {
            uint32_t s = (uint32_t)g_rpm / 40U;
            g_speed = (s > 250U) ? 250U : (uint8_t)s;
        }

        /* Read temperature */
        float t = MAX6675_ReadTemp();
        if (t >= 0.0f) g_engTemp = t;

        /* Handle buzzer from this task (safe osDelay context) */
        if (g_buzzerFlag) {
            uint8_t n    = g_buzzerBeeps;
            g_buzzerFlag = 0;
            Buzzer_Beep(n);
        }

        osDelay(50);
    }
}

/* ================================================================
   TASK 2 — CAN Transmit (10ms)
   Sends all CAN frames + UART dashboard every 500ms
   ================================================================ */
static void Task_CANTransmit(void *argument)
{
    uint32_t lastPrint = 0;
    static const char *labels[] = {"NORMAL", "FAULT ", "LIMP  ", "RECOV "};

    for (;;)
    {
        /* 0x100 — RPM (2 bytes) */
        g_txHeader.StdId = 0x100;
        g_txHeader.DLC   = 2;
        g_txData[0] = (g_rpm >> 8) & 0xFF;
        g_txData[1] =  g_rpm & 0xFF;
        HAL_CAN_AddTxMessage(&hcan1, &g_txHeader, g_txData, &g_txMailbox);

        /* 0x101 — Speed (1 byte) */
        g_txHeader.StdId = 0x101;
        g_txHeader.DLC   = 1;
        g_txData[0]      = g_speed;
        HAL_CAN_AddTxMessage(&hcan1, &g_txHeader, g_txData, &g_txMailbox);

        /* 0x102 — Temperature × 10 (2 bytes) */
        g_txHeader.StdId = 0x102;
        g_txHeader.DLC   = 2;
        {
            uint16_t tx10 = (uint16_t)(g_engTemp * 10.0f);
            g_txData[0] = (tx10 >> 8) & 0xFF;
            g_txData[1] =  tx10 & 0xFF;
        }
        HAL_CAN_AddTxMessage(&hcan1, &g_txHeader, g_txData, &g_txMailbox);

        /* 0x103 — Relay (ON when overtemp) */
        g_txHeader.StdId = 0x103;
        g_txHeader.DLC   = 1;
        g_txData[0]      = (g_engTemp > OVERTEMP_FAULT_C) ? 0x01U : 0x00U;
        HAL_CAN_AddTxMessage(&hcan1, &g_txHeader, g_txData, &g_txMailbox);

        /* 0x104 — System state */
        g_txHeader.StdId = 0x104;
        g_txHeader.DLC   = 1;
        g_txData[0]      = (uint8_t)g_sysState;
        HAL_CAN_AddTxMessage(&hcan1, &g_txHeader, g_txData, &g_txMailbox);

        /* 0x1FF — DTC when fault active */
        if (g_faultActive)
            DTC_Transmit(0x50, 0x02, 0x17); /* P0217 overtemp */

        /* UART dashboard every 500ms */
        uint32_t now = HAL_GetTick();
        if ((now - lastPrint) >= 500UL) {
            lastPrint = now;
            uint16_t dispRPM = (g_rpm == 0xFFFFU) ? 0U : g_rpm;

            /* Temp as integers (nano.specs float fix) */
            int16_t tw = (int16_t)g_engTemp;
            uint8_t tf = (uint8_t)((g_engTemp - (float)tw) * 10.0f);

            char line[160];
            snprintf(line, sizeof(line),
                "+-------------------------------------------------+\r\n"
                "| State: %-6s | RPM: %5u | Spd: %3u km/h |\r\n"
                "| Temp:  %3d.%uC   | ADC: %4lu             |\r\n"
                "+-------------------------------------------------+\r\n",
                labels[g_sysState & 0x03],
                (unsigned)dispRPM,
                (unsigned)g_speed,
                tw, tf,
                (unsigned long)g_adcRaw);
            UART_Log(line);
        }

        osDelay(10);
    }
}

/* ================================================================
   TASK 3 — Fault Detection (5ms)
   Pure automatic — temperature driven state machine
   No button interaction
   ================================================================ */
static void Task_FaultDetect(void *argument)
{
    /* 1 second startup settle — prevents boot false triggers */
    osDelay(1000);
    UART_Log("[FSM] Fault detection active\r\n");

    for (;;)
    {
        uint32_t now = HAL_GetTick();

        switch (g_sysState)
        {
            /* ══ NORMAL: monitor temperature ══ */
            case SYS_NORMAL:
                g_faultActive = 0;

                if (g_engTemp > OVERTEMP_FAULT_C)
                {
                    char msg[80];
                    int16_t tw = (int16_t)g_engTemp;
                    uint8_t tf = (uint8_t)((g_engTemp - tw) * 10.0f);
                    snprintf(msg, sizeof(msg),
                        "[FSM] NORMAL → FAULT (Temp %d.%uC > 40C)\r\n", tw, tf);
                    UART_Log(msg);

                    g_sysState    = SYS_FAULT;
                    g_faultTimer  = now;
                    g_faultActive = 1;
                    DTC_Transmit(0x50, 0x02, 0x17); /* P0217 */
                    g_buzzerBeeps = 2;   /* 2 beeps for overtemp */
                    g_buzzerFlag  = 1;
                }
                break;

            /* ══ FAULT: wait 3s then enter limp ══ */
            case SYS_FAULT:
                if ((now - g_faultTimer) >= 3000UL)
                {
                    UART_Log("[FSM] FAULT → LIMP (3s timeout)\r\n");
                    g_sysState    = SYS_LIMP;
                    g_limpTimer   = now;
                    g_faultActive = 0;
                    g_buzzerBeeps = 1;
                    g_buzzerFlag  = 1;
                }
                break;

            /* ══ LIMP: RPM locked, wait for temp to drop ══ */
            case SYS_LIMP:
                /* Check if temp has recovered to safe level */
                if (g_engTemp < OVERTEMP_RECOVER_C)
                {
                    /* Must be below threshold for 5 seconds */
                    if ((now - g_limpTimer) >= 5000UL)
                    {
                        UART_Log("[FSM] LIMP → RECOVER (temp safe for 5s)\r\n");
                        g_sysState     = SYS_RECOVER;
                        g_recoverTimer = now;
                    }
                }
                else
                {
                    /* Temp still high — reset the 5s timer */
                    g_limpTimer = now;
                }
                break;

            /* ══ RECOVER: 2s settle then back to normal ══ */
            case SYS_RECOVER:
                if ((now - g_recoverTimer) >= 2000UL)
                {
                    if (g_engTemp < OVERTEMP_RECOVER_C)
                    {
                        UART_Log("[FSM] RECOVER → NORMAL (all clear)\r\n");
                        DTC_Transmit(0x50, 0x0A, 0x01); /* recovery OK */
                        g_sysState    = SYS_NORMAL;
                        g_faultActive = 0;
                        g_rpm         = RPM_MIN;
                        g_buzzerBeeps = 2;
                        g_buzzerFlag  = 1;
                    }
                    else
                    {
                        UART_Log("[FSM] RECOVER → FAULT (still hot)\r\n");
                        g_sysState    = SYS_FAULT;
                        g_faultTimer  = now;
                        g_faultActive = 1;
                    }
                }
                break;
        }

        osDelay(5);
    }
}

/* ================================================================
   TASK 4 — LED Heartbeat (500ms)
   Green solid  = NORMAL
   Red solid    = FAULT
   Orange blink = LIMP
   Green blink  = RECOVER
   ================================================================ */
static void Task_LEDHeartbeat(void *argument)
{
    for (;;)
    {
        HAL_GPIO_WritePin(GPIOD,
            GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14, GPIO_PIN_RESET);

        switch (g_sysState) {
            case SYS_NORMAL:
                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
                break;
            case SYS_FAULT:
                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
                break;
            case SYS_LIMP:
                HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
                break;
            case SYS_RECOVER:
                HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
                break;
        }

        osDelay(500);
    }
}

/* ================================================================
   SYSTEM CLOCK — 168MHz from 8MHz HSE
   ================================================================ */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = 8;
    RCC_OscInitStruct.PLL.PLLN       = 336;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ       = 7;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

void Error_Handler(void)
{
    __disable_irq();
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
    while (1) {}
}
