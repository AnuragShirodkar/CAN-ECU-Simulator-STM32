/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Engine ECU — Board 1 — STM32F407 Discovery
  *                   CAN Bus ECU Simulator — Production Firmware v2.0
  *
  * FIX LOG (v1 → v2):
  *   1. Removed duplicate osKernelStart() from USER CODE END 2
  *   2. g_adcRaw promoted to global volatile (was local, printed as 0)
  *   3. Max RPM changed to 15000 (multiplier 800+14200)
  *   4. Overtemp threshold: 800°C → 40°C (demo-friendly, room temp)
  *   5. UART output reformatted — clear, readable dashboard every 500ms
  *   6. Flame sensor status shown every cycle in UART
  *   7. RECOVER state: 2-second settle timer before NORMAL/re-FAULT decision
  *   8. HAL_Delay inside FreeRTOS task → osDelay
  *   9. Speed capped at 250 km/h
  *  10. Task periods tuned: SensorRead=50ms, CANTransmit=10ms, FaultDetect=5ms
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
/* Overtemperature threshold — set to 40.0°C for room-temp demo.
   Change to 800.0f for real automotive use. */
#define OVERTEMP_THRESHOLD_C   40.0f

/* Max RPM joystick can reach */
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

/* ── Shared sensor data — volatile, written by SensorRead, read by all ── */
volatile uint16_t      g_rpm         = RPM_MIN;
volatile uint8_t       g_speed       = 0;
volatile float         g_engTemp     = 25.0f;
volatile uint8_t       g_flameSeen   = 0;   /* 1 = flame active right now */

/* ── State machine variables ── */
volatile uint8_t       g_faultActive = 0;
volatile SystemState_t g_sysState    = SYS_NORMAL;
volatile uint32_t      g_faultTimer  = 0;
volatile uint32_t      g_limpTimer   = 0;
volatile uint32_t      g_recoverTimer= 0;   /* FIX #7: settle timer */

/* ── Raw ADC value — global so CANTransmit task can print it ── */
volatile uint32_t      g_adcRaw      = 0;  /* FIX #2 */

/* ── Buzzer flags — set by FaultDetect, consumed by SensorRead ── */
volatile uint8_t       g_buzzerFlag  = 0;
volatile uint8_t       g_buzzerBeeps = 0;

/* ── CAN TX shared resources ── */
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

/* ── UART_Log ─────────────────────────────────────────────────────────────── */
static void UART_Log(const char *msg)
{
    HAL_UART_Transmit(&huart2, (const uint8_t *)msg, strlen(msg), 200);
}

/* ── Buzzer_Beep ──────────────────────────────────────────────────────────── */
static void Buzzer_Beep(uint8_t times)
{
    for (uint8_t i = 0; i < times; i++) {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);
        osDelay(200);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);
        osDelay(150);
    }
}

/* ── DTC_Transmit ─────────────────────────────────────────────────────────── */
/* Sends a 3-byte DTC on CAN ID 0x1FF.
   b1=0x50 (Powertrain), b2/b3 = code bytes.
   Examples: P0335 → 0x50,0x03,0x35 | P0217 → 0x50,0x02,0x17            */
static void DTC_Transmit(uint8_t b1, uint8_t b2, uint8_t b3)
{
    CAN_TxHeaderTypeDef hdr;
    uint8_t  d[3];
    uint32_t mb;

    hdr.StdId              = 0x1FF;
    hdr.IDE                = CAN_ID_STD;
    hdr.RTR                = CAN_RTR_DATA;
    hdr.DLC                = 3;
    hdr.TransmitGlobalTime = DISABLE;

    d[0] = b1;  d[1] = b2;  d[2] = b3;
    HAL_CAN_AddTxMessage(&hcan1, &hdr, d, &mb);
}

/* ── MAX6675_ReadTemp ─────────────────────────────────────────────────────── */
/* Reads K-type thermocouple via SPI2. Returns °C (0.25°C resolution).
   Returns -1.0f if thermocouple is open-circuit.                             */
static float MAX6675_ReadTemp(void)
{
    uint8_t  rx[2] = {0, 0};
    uint16_t raw;

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET); /* CS LOW  */
    osDelay(1);                          /* FIX #8: osDelay not HAL_Delay */
    HAL_SPI_Receive(&hspi2, rx, 2, 100);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);   /* CS HIGH */

    raw = ((uint16_t)rx[0] << 8) | rx[1];

    if (raw & 0x0004) {
        UART_Log("[TEMP] Open-circuit — check thermocouple wiring\r\n");
        return -1.0f;
    }

    raw >>= 3;
    return (float)raw * 0.25f;
}

/* USER CODE END 0 */

/* ── main ────────────────────────────────────────────────────────────────── */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  UART_Log("\r\n");
  UART_Log("================================================\r\n");
  UART_Log("  ENGINE ECU  |  STM32F407  |  Firmware v2.0  \r\n");
  UART_Log("  CAN Bus ECU Simulator — Automotive Demo      \r\n");
  UART_Log("  Overtemp threshold : 40.0 C (room demo)      \r\n");
  UART_Log("  Max RPM            : 15000                   \r\n");
  UART_Log("================================================\r\n\r\n");

  /* MAX6675 CS idle-high */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);

  /* Buzzer off */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);

  /* CAN1 receive filter — pass all frames */
  {
      CAN_FilterTypeDef flt;
      flt.FilterBank           = 0;
      flt.FilterMode           = CAN_FILTERMODE_IDMASK;
      flt.FilterScale          = CAN_FILTERSCALE_32BIT;
      flt.FilterIdHigh         = 0x0000;
      flt.FilterIdLow          = 0x0000;
      flt.FilterMaskIdHigh     = 0x0000;
      flt.FilterMaskIdLow      = 0x0000;
      flt.FilterFIFOAssignment = CAN_RX_FIFO0;
      flt.FilterActivation     = ENABLE;
      flt.SlaveStartFilterBank = 14;

      if (HAL_CAN_ConfigFilter(&hcan1, &flt) != HAL_OK) {
          UART_Log("[INIT] ERROR: CAN1 filter config FAILED\r\n");
          Error_Handler();
      }
  }
  UART_Log("[INIT] CAN1 filter     : OK\r\n");

  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
      UART_Log("[INIT] ERROR: CAN1 start FAILED\r\n");
      Error_Handler();
  }
  UART_Log("[INIT] CAN1 bus        : Running @ 500 kbps\r\n");

  /* TX header static fields */
  g_txHeader.IDE                = CAN_ID_STD;
  g_txHeader.RTR                = CAN_RTR_DATA;
  g_txHeader.TransmitGlobalTime = DISABLE;

  /* Startup beep — 1 short = system ready */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);
  HAL_Delay(300);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);

  UART_Log("[INIT] Peripherals     : OK\r\n");
  UART_Log("[INIT] Starting FreeRTOS...\r\n\r\n");

  /* ── Create FreeRTOS tasks ── */
  osThreadNew(Task_SensorRead,   NULL,
      &(const osThreadAttr_t){ .name="SensorRead",
          .stack_size = 512 * 4,
          .priority   = osPriorityNormal });

  osThreadNew(Task_CANTransmit,  NULL,
      &(const osThreadAttr_t){ .name="CANTransmit",
          .stack_size = 512 * 4,
          .priority   = osPriorityNormal });

  osThreadNew(Task_FaultDetect,  NULL,
      &(const osThreadAttr_t){ .name="FaultDetect",
          .stack_size = 512 * 4,
          .priority   = osPriorityAboveNormal });

  osThreadNew(Task_LEDHeartbeat, NULL,
      &(const osThreadAttr_t){ .name="LEDHeart",
          .stack_size = 256 * 4,
          .priority   = osPriorityBelowNormal });

  /* !! FIX #1: DO NOT call osKernelStart() here.
     CubeMX-generated code below calls osKernelInitialize(),
     MX_FREERTOS_Init(), and osKernelStart() in the correct order.
     Calling it here causes a double-start hard fault.            !! */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler — does not return */
  osKernelStart();

  /* We should never get here */
  while (1) {}
}

/* USER CODE BEGIN 4 */

/* ══════════════════════════════════════════════════════════════════
   Task_SensorRead  |  Priority: Normal  |  Period: 50 ms
   ─────────────────────────────────────────────────────────────────
   - Reads joystick ADC → maps to 800–15000 RPM
   - State machine selects which RPM value is broadcast
   - Reads MAX6675 temperature every cycle
   - Reads flame sensor digital pin
   - Drives buzzer on request from FaultDetect
   ══════════════════════════════════════════════════════════════════ */
static void Task_SensorRead(void *argument)
{
    for (;;)
    {
        /* ── Joystick ADC read — always, regardless of state ── */
        HAL_ADC_Start(&hadc1);
        if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
            g_adcRaw = HAL_ADC_GetValue(&hadc1);   /* FIX #2: global */
        HAL_ADC_Stop(&hadc1);

        /* Map 0–4095  →  800–15000 RPM  (FIX #3: was 10000) */
        uint16_t joystickRPM = (uint16_t)(RPM_MIN +
                               ((g_adcRaw * (uint32_t)(RPM_MAX - RPM_MIN)) / 4095UL));

        /* ── State machine selects active RPM ── */
        switch (g_sysState)
        {
            case SYS_NORMAL:
                g_rpm = joystickRPM;          /* full joystick control */
                break;
            case SYS_LIMP:
                g_rpm = 1500;                 /* locked — joystick ignored */
                break;
            case SYS_FAULT:
                g_rpm = 0xFFFF;              /* fault sentinel */
                break;
            case SYS_RECOVER:
                g_rpm = joystickRPM;          /* allow control during recovery */
                break;
        }

        /* Speed: rpm/40, capped at 250 km/h (FIX #9) */
        if (g_rpm == 0xFFFF)
            g_speed = 0;
        else {
            uint32_t spd = (uint32_t)g_rpm / 40U;
            g_speed = (spd > 250U) ? 250U : (uint8_t)spd;
        }

        /* ── Temperature read every cycle (FIX #5: real-time) ── */
        float t = MAX6675_ReadTemp();
        if (t >= 0.0f)
            g_engTemp = t;

        /* ── Flame sensor — digital read (FIX #6) ── */
        /* LOW = flame detected (active-low module output) */
        g_flameSeen = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_RESET) ? 1U : 0U;

        /* ── Buzzer: driven here (task context, safe for osDelay) ── */
        if (g_buzzerFlag)
        {
            uint8_t n    = g_buzzerBeeps;
            g_buzzerFlag = 0;
            Buzzer_Beep(n);
        }

        osDelay(50);   /* 50 ms = 20 Hz sensor update rate */
    }
}

/* ══════════════════════════════════════════════════════════════════
   Task_CANTransmit  |  Priority: Normal  |  Period: 10 ms
   ─────────────────────────────────────────────────────────────────
   - Sends 5 regular CAN frames every 10 ms
   - Sends DTC frame on 0x1FF when fault is active
   - Prints clean UART dashboard every 500 ms
   ══════════════════════════════════════════════════════════════════ */
static const char *stateLabel[] = {"NORMAL", "FAULT ", "LIMP  ", "RECOV "};

static void Task_CANTransmit(void *argument)
{
    uint32_t lastPrint = 0;

    for (;;)
    {
        /* ── 0x100 — RPM (2 bytes, big-endian) ── */
        g_txHeader.StdId = 0x100;
        g_txHeader.DLC   = 2;
        g_txData[0] = (g_rpm >> 8) & 0xFF;
        g_txData[1] =  g_rpm & 0xFF;
        HAL_CAN_AddTxMessage(&hcan1, &g_txHeader, g_txData, &g_txMailbox);

        /* ── 0x101 — Speed (1 byte, km/h) ── */
        g_txHeader.StdId = 0x101;
        g_txHeader.DLC   = 1;
        g_txData[0]      = g_speed;
        HAL_CAN_AddTxMessage(&hcan1, &g_txHeader, g_txData, &g_txMailbox);

        /* ── 0x102 — Temperature × 10 (2 bytes, big-endian) ── */
        g_txHeader.StdId = 0x102;
        g_txHeader.DLC   = 2;
        {
            uint16_t tx10 = (uint16_t)(g_engTemp * 10.0f);
            g_txData[0] = (tx10 >> 8) & 0xFF;
            g_txData[1] =  tx10 & 0xFF;
        }
        HAL_CAN_AddTxMessage(&hcan1, &g_txHeader, g_txData, &g_txMailbox);

        /* ── 0x103 — Relay (1 = overheat or flame) ── */
        g_txHeader.StdId = 0x103;
        g_txHeader.DLC   = 1;
        g_txData[0] = ((g_engTemp > OVERTEMP_THRESHOLD_C) || g_flameSeen) ? 0x01U : 0x00U;
        HAL_CAN_AddTxMessage(&hcan1, &g_txHeader, g_txData, &g_txMailbox);

        /* ── 0x104 — System state ── */
        g_txHeader.StdId = 0x104;
        g_txHeader.DLC   = 1;
        g_txData[0]      = (uint8_t)g_sysState;
        HAL_CAN_AddTxMessage(&hcan1, &g_txHeader, g_txData, &g_txMailbox);

        /* ── 0x1FF — DTC if fault active ── */
        if (g_faultActive)
            DTC_Transmit(0x50, 0x03, 0x35); /* P0335 crankshaft sensor */

        /* ── UART dashboard — printed every 500 ms (FIX #4/#5/#6) ── */
        uint32_t now = HAL_GetTick();
        if ((now - lastPrint) >= 500UL)
        {
            lastPrint = now;

            /* RPM display: show 0 when fault sentinel */
            uint16_t displayRPM = (g_rpm == 0xFFFF) ? 0U : g_rpm;

            char line[120];
            snprintf(line, sizeof(line),
                "[ ECU ] State:%-6s | RPM:%5u | Spd:%3u km/h | "
                "Tmp:%5.1f C | Flame:%s | ADC:%4lu\r\n",
                stateLabel[g_sysState],
                (unsigned)displayRPM,
                (unsigned)g_speed,
                (double)g_engTemp,
                g_flameSeen ? "YES(!)" : "no   ",
                (unsigned long)g_adcRaw);
            UART_Log(line);
        }

        osDelay(10);
    }
}

/* ══════════════════════════════════════════════════════════════════
   Task_FaultDetect  |  Priority: AboveNormal  |  Period: 5 ms
   ─────────────────────────────────────────────────────────────────
   State machine:
     NORMAL  → FAULT   : button press | overtemp | flame
     FAULT   → LIMP    : 3 s elapsed
     LIMP    → RECOVER : 5 s elapsed + sensors safe | manual button
     RECOVER → NORMAL  : 2 s settle + sensors safe
     RECOVER → FAULT   : sensors still unsafe after 2 s settle
   ══════════════════════════════════════════════════════════════════ */
static void Task_FaultDetect(void *argument)
{
    for (;;)
    {
        uint32_t now = HAL_GetTick();

        /* ── Flame sensor: highest priority, immediate state override ── */
        /* g_flameSeen is written by Task_SensorRead every 50 ms         */
        if (g_flameSeen && g_sysState != SYS_FAULT)
        {
            UART_Log("\r\n[FAULT] *** FLAME DETECTED — EMERGENCY SHUTDOWN ***\r\n");

            g_sysState    = SYS_FAULT;
            g_faultTimer  = now;
            g_faultActive = 1;
            g_rpm         = 0xFFFF;

            /* Emergency broadcast frame on 0x1FE */
            {
                CAN_TxHeaderTypeDef h;
                uint8_t  d[1] = {0xFF};
                uint32_t mb;
                h.StdId = 0x1FE;  h.IDE = CAN_ID_STD;
                h.RTR   = CAN_RTR_DATA;  h.DLC = 1;
                h.TransmitGlobalTime = DISABLE;
                HAL_CAN_AddTxMessage(&hcan1, &h, d, &mb);
            }
            DTC_Transmit(0x50, 0x02, 0x17); /* P0217 overheat/fire */
            g_buzzerBeeps = 5;
            g_buzzerFlag  = 1;
        }

        /* ── 4-State FSM ── */
        switch (g_sysState)
        {
            /* ══ NORMAL ══ */
            case SYS_NORMAL:
                g_faultActive = 0;

                /* USER button (PA0, active-low) → inject crankshaft fault P0335 */
                if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET)
                {
                    UART_Log("\r\n[FSM] NORMAL → FAULT  (USER button — P0335 injected)\r\n");
                    g_sysState    = SYS_FAULT;
                    g_faultTimer  = now;
                    g_faultActive = 1;
                    g_rpm         = 0xFFFF;
                    DTC_Transmit(0x50, 0x03, 0x35);
                    g_buzzerBeeps = 3;
                    g_buzzerFlag  = 1;
                    break;
                }

                /* Overtemperature → P0217 */
                if (g_engTemp > OVERTEMP_THRESHOLD_C)
                {
                    char msg[80];
                    snprintf(msg, sizeof(msg),
                        "\r\n[FSM] NORMAL → FAULT  (Overtemp %.1fC > %.1fC — P0217)\r\n",
                        (double)g_engTemp, (double)OVERTEMP_THRESHOLD_C);
                    UART_Log(msg);
                    g_sysState    = SYS_FAULT;
                    g_faultTimer  = now;
                    g_faultActive = 1;
                    DTC_Transmit(0x50, 0x02, 0x17);
                    g_buzzerBeeps = 2;
                    g_buzzerFlag  = 1;
                }
                break;

            /* ══ FAULT ══ */
            case SYS_FAULT:
                /* After 3 seconds → enter LIMP mode */
                if ((now - g_faultTimer) >= 3000UL)
                {
                    UART_Log("[FSM] FAULT  → LIMP   (3s timeout — engine limited)\r\n");
                    g_sysState    = SYS_LIMP;
                    g_limpTimer   = now;
                    g_rpm         = 1500;
                    g_faultActive = 0;
                    g_buzzerBeeps = 1;
                    g_buzzerFlag  = 1;
                }
                break;

            /* ══ LIMP ══ */
            case SYS_LIMP:
                g_rpm = 1500; /* enforce lock every cycle */

                /* Auto-recovery: 5 s in LIMP + sensors safe */
                if ((now - g_limpTimer) >= 5000UL)
                {
                    if (g_engTemp < OVERTEMP_THRESHOLD_C &&
                        !g_flameSeen &&
                        HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET)
                    {
                        UART_Log("[FSM] LIMP   → RECOVER (auto — sensors safe)\r\n");
                        g_sysState    = SYS_RECOVER;
                        g_recoverTimer = now;  /* FIX #7 */
                    }
                }

                /* Manual recovery: button press + release */
                if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET)
                {
                    while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET)
                        osDelay(10);
                    UART_Log("[FSM] LIMP   → RECOVER (manual button)\r\n");
                    g_sysState     = SYS_RECOVER;
                    g_recoverTimer = now;  /* FIX #7 */
                }
                break;

            /* ══ RECOVER ══ */
            case SYS_RECOVER:
                /* FIX #7: wait 2 s settle window before making the decision */
                if ((now - g_recoverTimer) >= 2000UL)
                {
                    if (g_engTemp < OVERTEMP_THRESHOLD_C && !g_flameSeen)
                    {
                        UART_Log("[FSM] RECOVER → NORMAL  (all sensors safe)\r\n");
                        DTC_Transmit(0x50, 0x0A, 0x01); /* recovery complete */
                        g_sysState    = SYS_NORMAL;
                        g_faultActive = 0;
                        g_rpm         = RPM_MIN;
                        g_buzzerBeeps = 2;
                        g_buzzerFlag  = 1;
                    }
                    else
                    {
                        UART_Log("[FSM] RECOVER → FAULT   (sensors still unsafe)\r\n");
                        g_sysState    = SYS_FAULT;
                        g_faultTimer  = now;
                        g_faultActive = 1;
                        g_recoverTimer = now; /* reset so next attempt waits again */
                    }
                }
                break;
        }

        osDelay(5);
    }
}

/* ══════════════════════════════════════════════════════════════════
   Task_LEDHeartbeat  |  Priority: BelowNormal  |  Period: 500 ms
   ─────────────────────────────────────────────────────────────────
   Green  PD12 solid  = NORMAL
   Red    PD14 solid  = FAULT
   Orange PD13 blink  = LIMP
   Green  PD12 blink  = RECOVER
   ══════════════════════════════════════════════════════════════════ */
static void Task_LEDHeartbeat(void *argument)
{
    for (;;)
    {
        /* Clear all LEDs first */
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14, GPIO_PIN_RESET);

        switch (g_sysState)
        {
            case SYS_NORMAL:
                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);   /* Green solid */
                break;
            case SYS_FAULT:
                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);   /* Red solid */
                break;
            case SYS_LIMP:
                HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);                 /* Orange blink */
                break;
            case SYS_RECOVER:
                HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);                 /* Green blink */
                break;
        }

        osDelay(500);
    }
}

/* USER CODE END 4 */

/* ── SystemClock_Config ─────────────────────────────────────────────────── */
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    Error_Handler();

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                   | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    Error_Handler();
}

/* ── Error_Handler ──────────────────────────────────────────────────────── */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET); /* Red LED on */
  while (1) {}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif
