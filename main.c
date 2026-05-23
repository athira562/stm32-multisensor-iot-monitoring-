/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  *
  *  Sensors:
  *    1. HC-SR04  Ultrasonic  – TRIG: PB0 (GPIO OUT), ECHO: PB1 (TIM3_CH4 IC)
  *    2. MQ  Gas Sensor       – AO  : PC0 (ADC1 CH10, 12-bit)
  *    3. MH  IR  Sensor       – DO  : PF0 (GPIO Input, active-LOW)
  *
  *  Feeds on Adafruit IO:
  *    "parking"   → OCCUPIED / EMPTY   (ultrasonic, threshold 20 cm)
  *    "gas"       → 0–4095 raw ADC     (MQ sensor)
  *    "ir-sensor" → DETECTED / CLEAR   (MH IR sensor)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "lwip/tcp.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern struct netif gnetif;

/**
 * @brief  Carries feed name + value string through the async TCP callback.
 *         Must remain valid until http_connected_callback fires (~few ms).
 *         A single static instance is safe because uploads are serialised
 *         with a 1500 ms LwIP pump between each call.
 */
typedef struct {
    char feed[32];
    char value[32];
} UploadCtx;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ── MH IR Sensor (flying-fish module) ── */
#define IR_SENSOR_PIN   GPIO_PIN_0      /* PF0 → DO, active-LOW            */
#define IR_SENSOR_PORT  GPIOF

/* ── MQ Gas Sensor ── */
/* PC0 → ADC1_IN10  (configured in MX_ADC1_Init)                          */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef  htim3;
ADC_HandleTypeDef  hadc1;       /* NEW – MQ gas sensor                      */
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint32_t last_upload_time = 0;

/* ── Adafruit IO credentials ── */
const char    *ADAFRUIT_IP   = "52.7.84.197";  /* io.adafruit.com          */
const uint16_t ADAFRUIT_PORT = 80;
const char    *AIO_USERNAME  = "abhijth_chndra";
const char    *AIO_KEY       = "aio_qVmu06cYnrXFFDXsHElH32LukXJf";

/* ── Feed names ── */
const char *AIO_FEED_US  = "ultrasonic";      /* Ultrasonic – slot occupancy  */
const char *AIO_FEED_GAS = "mq-sensor";          /* MQ gas sensor – ADC raw      */
const char *AIO_FEED_IR  = "ir-sensor";   /* MH IR sensor – presence      */

/* ── HC-SR04 input-capture state ── */
volatile uint32_t ic_rising  = 0;
volatile uint32_t ic_falling = 0;
volatile uint8_t  ic_state   = 0;  /* 0 = waiting rising, 1 = waiting falling */
volatile uint8_t  ic_done    = 0;  /* 1 = measurement ready                   */
float distance_cm = 0.0f;

/* ── New sensor readings ── */
uint16_t gas_raw_value = 0;    /* MQ:  0–4095 ADC count                    */
uint8_t  ir_detected   = 0;    /* MH:  1 = object present, 0 = clear       */

/* ── Shared TCP upload context ── */
static UploadCtx upload_ctx;

/* ── HTTP request scratch buffer ── */
char request_buffer[512];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);             /* NEW */

/* USER CODE BEGIN PFP */
/* Sensor helpers */
void     HCSR04_Trigger(void);
float    HCSR04_GetDistance(void);
uint16_t MQ_ReadRaw(void);                  /* NEW */
uint8_t  MH_Read(void);                    /* NEW */

/* Network helpers */
void upload_sensor_data(const char *feed, const char *value); /* NEW */
static err_t http_connected_callback(void *arg, struct tcp_pcb *tpcb, err_t err);
static void  http_error_callback(void *arg, err_t err);
static err_t http_recv_callback(void *arg, struct tcp_pcb *tpcb,
                                 struct pbuf *p, err_t err);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char dbg[160];   /* enlarged to fit three sensor values                     */
/* USER CODE END 0 */

/* ─────────────────────────────── main ──────────────────────────────────── */
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
    MX_LWIP_Init();
    MX_TIM3_Init();
    MX_USART3_UART_Init();
    MX_ADC1_Init();         /* NEW – must come after GPIO clocks are enabled */

    /* USER CODE BEGIN 2 */
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */
        /* USER CODE BEGIN 3 */

        MX_LWIP_Process();

        /* ── Debug: print all three sensor readings every 500 ms ── */
        // Extract the whole number part
        int dist_whole = (int)distance_cm;

        // Extract the fractional part (2 decimal places)
        // We use absolute value to avoid issues if distance_cm is ever negative
        int dist_frac = (int)(fabs(distance_cm - dist_whole) * 100);

        snprintf(dbg, sizeof(dbg),
                 "[US] %d.%02d cm | [GAS] %4u | [IR] %s | ic_done=%d\r\n",
                 dist_whole,
                 dist_frac,
                 gas_raw_value,
                 ir_detected ? "DETECTED" : "CLEAR",
                 (int)ic_done);
        HAL_UART_Transmit(&huart3, (uint8_t*)dbg, strlen(dbg), 100);
        HAL_Delay(500);

        /* ── Upload every 5 seconds ── */
        if (HAL_GetTick() - last_upload_time > 7000)
        {
            if (netif_is_up(&gnetif) && netif_is_link_up(&gnetif))
            {
                uint32_t t;

                /* ── STEP 1: Read all sensors ─────────────────────────── */

                /* 1a. Ultrasonic (HC-SR04) */
                HCSR04_Trigger();
                t = HAL_GetTick();
                while (HAL_GetTick() - t < 50) MX_LWIP_Process(); /* 50 ms echo window */
                distance_cm = HCSR04_GetDistance();

                /* 1b. MQ Gas Sensor */
                gas_raw_value = MQ_ReadRaw();

                /* 1c. MH IR Sensor */
                ir_detected = MH_Read();

                /* ── STEP 2: Upload – Ultrasonic ──────────────────────── */
                if (distance_cm > 0.0f)
                {
                    const char *slot = (distance_cm < 20.0f) ? "OCCUPIED" : "EMPTY";
                    upload_sensor_data(AIO_FEED_US, slot);
                    t = HAL_GetTick();
                    while (HAL_GetTick() - t < 1500) MX_LWIP_Process();
                }

                /* ── STEP 3: Upload – Gas ─────────────────────────────── */
                {
                    char gv[8];
                    snprintf(gv, sizeof(gv), "%u", gas_raw_value);
                    upload_sensor_data(AIO_FEED_GAS, gv);
                    t = HAL_GetTick();
                    while (HAL_GetTick() - t < 1500) MX_LWIP_Process();
                }

                /* ── STEP 4: Upload – IR ──────────────────────────────── */
                upload_sensor_data(AIO_FEED_IR,
                                   ir_detected ? "DETECTED" : "CLEAR");
                t = HAL_GetTick();
                while (HAL_GetTick() - t < 1500) MX_LWIP_Process();

                last_upload_time = HAL_GetTick();
            }
        }
    }
    /* USER CODE END 3 */
}

/* ──────────────────────── Clock configuration ───────────────────────────── */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_BYPASS;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM      = 4;
    RCC_OscInitStruct.PLL.PLLN      = 50;
    RCC_OscInitStruct.PLL.PLLP      = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ      = 2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    if (HAL_PWREx_EnableOverDrive() != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
        Error_Handler();
}

/* ──────────────────────── ADC1 Init (NEW) ───────────────────────────────── */
/**
  * @brief  ADC1 Initialisation for MQ gas sensor
  *         PC0 → ADC1_IN10, 12-bit, software-triggered, single conversion
  */
static void MX_ADC1_Init(void)
{
    /* USER CODE BEGIN ADC1_Init 0 */
    /* USER CODE END ADC1_Init 0 */

    ADC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN ADC1_Init 1 */
    /* USER CODE END ADC1_Init 1 */

    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode          = DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();

    /* Channel 10 = PC0 */
    sConfig.Channel      = ADC_CHANNEL_10;
    sConfig.Rank         = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES; /* max stability for resistive sensor */
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();

    /* USER CODE BEGIN ADC1_Init 2 */
    /* USER CODE END ADC1_Init 2 */
}

/* ──────────────────────── TIM3 Init (unchanged) ────────────────────────── */
static void MX_TIM3_Init(void)
{
    /* USER CODE BEGIN TIM3_Init 0 */
    /* USER CODE END TIM3_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig     = {0};
    TIM_IC_InitTypeDef sConfigIC              = {0};

    /* USER CODE BEGIN TIM3_Init 1 */
    /* USER CODE END TIM3_Init 1 */

    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 50 - 1;   /* APB1 50 MHz → timer 1 MHz (1 µs) */
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 65535;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK) Error_Handler();

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
        Error_Handler();

    if (HAL_TIM_IC_Init(&htim3) != HAL_OK) Error_Handler();

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
        Error_Handler();

    sConfigIC.ICPolarity  = TIM_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter    = 0;
    if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
        Error_Handler();

    /* USER CODE BEGIN TIM3_Init 2 */
    /* USER CODE END TIM3_Init 2 */
}

/* ──────────────────────── USART3 Init (unchanged) ──────────────────────── */
static void MX_USART3_UART_Init(void)
{
    /* USER CODE BEGIN USART3_Init 0 */
    /* USER CODE END USART3_Init 0 */

    /* USER CODE BEGIN USART3_Init 1 */
    /* USER CODE END USART3_Init 1 */

    huart3.Instance                    = USART3;
    huart3.Init.BaudRate               = 115200;
    huart3.Init.WordLength             = UART_WORDLENGTH_8B;
    huart3.Init.StopBits               = UART_STOPBITS_1;
    huart3.Init.Parity                 = UART_PARITY_NONE;
    huart3.Init.Mode                   = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling           = UART_OVERSAMPLING_16;
    huart3.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart3) != HAL_OK) Error_Handler();

    /* USER CODE BEGIN USART3_Init 2 */
    /* USER CODE END USART3_Init 2 */
}

/* ──────────────────────── GPIO Init ────────────────────────────────────── */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* USER CODE BEGIN MX_GPIO_Init_1 */
    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();   /* NEW – needed for PF0 (IR sensor DO)  */

    /* ── PB0: HC-SR04 TRIG (existing) ── */
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin   = TRIG_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(TRIG_GPIO_Port, &GPIO_InitStruct);

    /* USER CODE BEGIN MX_GPIO_Init_2 */

    /* ── PF0: MH IR Sensor DO (NEW) ──
     *  The MH module's DO pin is open-collector active-LOW:
     *  LOW  = IR reflected → object detected
     *  HIGH = no reflection → clear
     *  Enable internal pull-up so the line is HIGH when no object is present. */
    GPIO_InitStruct.Pin   = IR_SENSOR_PIN;   /* GPIO_PIN_0                   */
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    HAL_GPIO_Init(IR_SENSOR_PORT, &GPIO_InitStruct);   /* GPIOF               */

    /* USER CODE END MX_GPIO_Init_2 */
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  USER CODE BEGIN 4
 * ═══════════════════════════════════════════════════════════════════════════ */

/* ──────────────────────── LwIP TCP callbacks ────────────────────────────── */

static err_t http_recv_callback(void *arg, struct tcp_pcb *tpcb,
                                 struct pbuf *p, err_t err)
{
    (void)arg; (void)err;
    if (p != NULL) {
        tcp_recved(tpcb, p->tot_len);
        pbuf_free(p);
    } else {
        /* p == NULL: server closed connection gracefully */
        tcp_close(tpcb);
    }
    return ERR_OK;
}

static void http_error_callback(void *arg, err_t err)
{
    (void)arg;
    /* PCB already freed by LwIP on error – do not call tcp_close() here */
    char msg[48];
    snprintf(msg, sizeof(msg), "[ERR] TCP error: %d\r\n", (int)err);
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 50);
}

/**
  * @brief  Called when TCP connection to Adafruit IO is established.
  *         Reads feed/value from the UploadCtx passed via tcp_arg().
  */
static err_t http_connected_callback(void *arg, struct tcp_pcb *tpcb, err_t err)
{
    if (err != ERR_OK) return err;

    UploadCtx *ctx = (UploadCtx *)arg;

    char body[48];
    snprintf(body, sizeof(body), "{\"value\":\"%s\"}", ctx->value);

    snprintf(request_buffer, sizeof(request_buffer),
        "POST /api/v2/%s/feeds/%s/data HTTP/1.1\r\n"
        "Host: io.adafruit.com\r\n"
        "X-AIO-Key: %s\r\n"
        "Content-Type: application/json\r\n"
        "Content-Length: %d\r\n"
        "Connection: close\r\n\r\n"
        "%s",
        AIO_USERNAME, ctx->feed, AIO_KEY, (int)strlen(body), body);

    tcp_recv(tpcb, http_recv_callback);
    tcp_write(tpcb, request_buffer, strlen(request_buffer), TCP_WRITE_FLAG_COPY);
    tcp_output(tpcb);
    return ERR_OK;
}

/**
  * @brief  Open a TCP connection and POST one sensor value to Adafruit IO.
  * @param  feed   Adafruit IO feed name (e.g. "gas", "ir-sensor", "parking")
  * @param  value  String value to publish (e.g. "OCCUPIED", "1234")
  *
  * @note   upload_ctx is a static struct; the 1500 ms LwIP pump after each
  *         call ensures the callback fires before the struct is overwritten.
  */
void upload_sensor_data(const char *feed, const char *value)
{
    struct tcp_pcb *pcb;
    ip_addr_t dest_ip;

    /* Populate shared context */
    strncpy(upload_ctx.feed,  feed,  sizeof(upload_ctx.feed)  - 1);
    upload_ctx.feed [sizeof(upload_ctx.feed)  - 1] = '\0';
    strncpy(upload_ctx.value, value, sizeof(upload_ctx.value) - 1);
    upload_ctx.value[sizeof(upload_ctx.value) - 1] = '\0';

    ipaddr_aton(ADAFRUIT_IP, &dest_ip);
    pcb = tcp_new();
    if (pcb != NULL) {
        tcp_arg(pcb, &upload_ctx);          /* pass context to callbacks    */
        tcp_err(pcb, http_error_callback);
        tcp_connect(pcb, &dest_ip, ADAFRUIT_PORT, http_connected_callback);
    }
}

/* ──────────────────────── HC-SR04 Ultrasonic ───────────────────────────── */

/**
  * @brief  Send a >10 µs trigger pulse on PB0 (trig_Pin).
  *         Resets ic_done so HCSR04_GetDistance() can detect a fresh result.
  */
void HCSR04_Trigger(void)
{
    ic_done  = 0;
    ic_state = 0;
    /* __HAL_TIM_SET_CAPTUREPOLARITY ensures we start on rising edge */
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_4,
                                  TIM_INPUTCHANNELPOLARITY_RISING);

    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
    HAL_Delay(1);   /* 1 ms >> 10 µs minimum pulse width */
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
}

/**
  * @brief  Compute distance from captured timer counts.
  * @retval distance in cm, or -1.0f if echo not yet ready.
  *
  *  TIM3 clock = APB1 / prescaler = 50 MHz / 50 = 1 MHz → 1 µs per count
  *  distance   = pulse_µs × 0.0343 / 2   (speed of sound, round-trip)
  */
float HCSR04_GetDistance(void)
{
    if (!ic_done) return -1.0f;

    uint32_t pulse_us = (ic_falling >= ic_rising)
        ? (ic_falling  - ic_rising)
        : (65536UL - ic_rising + ic_falling);   /* handle 16-bit counter wrap */

    return (pulse_us * 0.0343f) / 2.0f;
}

/**
  * @brief  TIM3 CH4 input-capture ISR – toggles polarity to catch both edges.
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
    {
        if (ic_state == 0)  /* ── Rising edge captured ── */
        {
            ic_rising = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4,
                                          TIM_INPUTCHANNELPOLARITY_FALLING);
            ic_state = 1;
        }
        else                /* ── Falling edge captured ── */
        {
            ic_falling = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4,
                                          TIM_INPUTCHANNELPOLARITY_RISING);
            ic_state = 0;
            ic_done  = 1;   /* Signal: measurement ready */
        }
    }
}

/* ──────────────────────── MQ Gas Sensor ────────────────────────────────── */
/**
  * @brief  Read MQ sensor analogue output via ADC1 CH10 (PC0).
  * @retval 12-bit raw ADC value (0 = 0 V, 4095 = VCC = 3.3 V on STM32F7).
  *
  *  Voltage on AO rises with gas concentration.
  *  To convert to ppm: apply sensor-specific Rs/R0 curve from datasheet.
  *  (MQ-2 ≈ LPG/Smoke, MQ-3 ≈ Alcohol, MQ-7 ≈ CO – choose per module.)
  */
uint16_t MQ_ReadRaw(void)
{
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10 /* ms timeout */) == HAL_OK)
        return (uint16_t)HAL_ADC_GetValue(&hadc1);
    return 0;   /* return 0 on timeout/error */
}

/* ──────────────────────── MH IR Sensor (Flying Fish) ───────────────────── */
/**
  * @brief  Read MH-series IR obstacle sensor digital output on PF0.
  * @retval 1 = object detected (DO LOW), 0 = clear (DO HIGH).
  *
  *  Sensitivity is adjustable via the onboard potentiometer.
  *  Typical detection range: 2–40 cm (varies with reflectivity).
  *  DO pin is open-collector; internal pull-up enabled in MX_GPIO_Init.
  */
uint8_t MH_Read(void)
{
    /* Active LOW: pin pulled to GND by comparator when object is present */
    return (HAL_GPIO_ReadPin(IR_SENSOR_PORT, IR_SENSOR_PIN) == GPIO_PIN_RESET)
           ? 1u : 0u;
}

/* USER CODE END 4 */

/* ──────────────────────── Error handler ────────────────────────────────── */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    __disable_irq();
    while (1) {}
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    (void)file; (void)line;
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
