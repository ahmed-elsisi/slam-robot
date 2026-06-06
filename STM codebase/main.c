/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
osThreadId_t controlTaskHandle;
const osThreadAttr_t controlTask_attributes = {
  .name = "controlTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
osThreadId_t encoderTaskHandle;
const osThreadAttr_t encoderTask_attributes = {
  .name = "encoderTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* USER CODE BEGIN PV */
volatile uint8_t uart4_rx;
volatile uint8_t uart2_rx;
osMessageQueueId_t velQueueHandle;
volatile uint32_t last_cmd_time_ms = 0;

typedef struct
{
  int16_t left_pwm;
  int16_t right_pwm;
} VelMsg_t;

static char rx_line_buf[32];
static uint8_t rx_line_idx = 0;

int32_t last_ticks1 = 0;
int32_t last_ticks2 = 0;

float distance1 = 0;
float distance2 = 0;

#define TICKS_PER_REV 3960.0f
#define WHEEL_DIAMETER 6.7f
#define WHEEL_CIRCUMFERENCE (3.1416f * WHEEL_DIAMETER)
#define CM_PER_TICK (WHEEL_CIRCUMFERENCE / TICKS_PER_REV)

volatile int32_t prev_enc1 = 0;
volatile int32_t prev_enc2 = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
static void MX_UART4_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void StartControlTask(void *argument);
void StartEncoderTask(void *argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static inline void set_speed(uint16_t duty)
{
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, duty);
}

static inline int16_t encoder_delta_16bit(uint16_t now, uint16_t previous)
{
  return (int16_t)(now - previous);
}

static inline void proto_uart_start_rx(UART_HandleTypeDef *huart, volatile uint8_t *rx_byte)
{
  HAL_UART_Receive_IT(huart, (uint8_t *)rx_byte, 1);
}

static inline void proto_uart_rearm(UART_HandleTypeDef *huart)
{
  if (huart == &huart4)
  {
    HAL_UART_Receive_IT(&huart4, (uint8_t *)&uart4_rx, 1);
  }
  else if (huart == &huart2)
  {
    HAL_UART_Receive_IT(&huart2, (uint8_t *)&uart2_rx, 1);
  }
}

#define MAX_PWM 49

static int16_t clamp_pwm(int value)
{
  if (value > MAX_PWM) return MAX_PWM;
  if (value < -MAX_PWM) return -MAX_PWM;
  return (int16_t)value;
}

static void parse_uart_byte(uint8_t byte)
{
  if (byte == '\r')
  {
    return;
  }

  if (byte == '\n')
  {
    rx_line_buf[rx_line_idx] = '\0';

    if (strncmp(rx_line_buf, "VEL,", 4) == 0)
    {
      int lp = 0;
      int rp = 0;

      if (sscanf(rx_line_buf + 4, "%d,%d", &lp, &rp) == 2)
      {
        VelMsg_t msg;
        msg.left_pwm = clamp_pwm(lp);
        msg.right_pwm = clamp_pwm(rp);

        osMessageQueuePut(velQueueHandle, &msg, 0, 0);
        last_cmd_time_ms = HAL_GetTick();
      }
    }

    rx_line_idx = 0;
  }
  else
  {
    if (rx_line_idx < sizeof(rx_line_buf) - 1)
    {
      rx_line_buf[rx_line_idx++] = (char)byte;
    }
    else
    {
      rx_line_idx = 0;  // discard too-long or corrupted packet
    }
  }
}

static inline void proto_uart_tx(const uint8_t *buf, uint16_t len)
{
  HAL_UART_Transmit(&huart4, (uint8_t *)buf, len, 20);
  HAL_UART_Transmit(&huart2, (uint8_t *)buf, len, 20);
}

/* ------------------------------------------------------
   D2  = PA10 -> Motor1 IN1  (front-left)
   D4  = PB5  -> Motor1 IN2  (front-left)
   D5  = PB4  -> Motor2 IN1  (front-right)
   D6  = PB10 -> Motor2 IN2  (front-right)
   28  = PB14 -> Motor3 IN1  (rear-left)
   26  = PB15 -> Motor3 IN2  (rear-left)
   D12 = PA6  -> Motor4 IN1  (rear-right)
   D13 = PA5  -> Motor4 IN2  (rear-right)
   ------------------------------------------------------ */

static inline void left_forward(void)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
}

static inline void left_reverse(void)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
}

static inline void right_forward(void)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
}

static inline void right_reverse(void)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
}

static inline void left_brake(void)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
}

static inline void right_brake(void)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
}

static void apply_velocity(int16_t left_pwm, int16_t right_pwm)
{
  // Left side direction
  if (left_pwm > 0)
  {
    left_forward();
  }
  else if (left_pwm < 0)
  {
    left_reverse();
  }
  else
  {
    left_brake();
  }

  // Right side direction
  if (right_pwm > 0)
  {
    right_forward();
  }
  else if (right_pwm < 0)
  {
    right_reverse();
  }
  else
  {
    right_brake();
  }

  /*
   * Current hardware limitation:
   * This code still uses ONE shared PWM channel: TIM4 CH1.
   * Therefore, the direction is independent per side, but speed is averaged.
   * For true curved motion, later add one PWM channel for the left side
   * and one PWM channel for the right side.
   */
  uint16_t speed = (uint16_t)((abs((int)left_pwm) + abs((int)right_pwm)) / 2);
  set_speed(speed);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
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
  MX_USART2_UART_Init();
  MX_TIM8_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_UART4_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);

  __HAL_TIM_SET_COUNTER(&htim8, 0);
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  /* USER CODE END 2 */

  last_cmd_time_ms = HAL_GetTick();

  osKernelInitialize();
  velQueueHandle = osMessageQueueNew(8, sizeof(VelMsg_t), NULL);
  if (velQueueHandle == NULL) Error_Handler();

  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  if (defaultTaskHandle == NULL) Error_Handler();

  controlTaskHandle = osThreadNew(StartControlTask, NULL, &controlTask_attributes);
  if (controlTaskHandle == NULL) Error_Handler();

  encoderTaskHandle = osThreadNew(StartEncoderTask, NULL, &encoderTask_attributes);
  if (encoderTaskHandle == NULL) Error_Handler();

  proto_uart_start_rx(&huart4, &uart4_rx);
  proto_uart_start_rx(&huart2, &uart2_rx);

  osKernelStart();

  while (1)
  {
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 83;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 49;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim4);
}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, D13_Pin | D12_Pin | D2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, D6_Pin | IN1_2_Pin | IN2_2_Pin | D5_Pin | D4_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = D13_Pin | D12_Pin | D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = D6_Pin | IN1_2_Pin | IN2_2_Pin | D5_Pin | D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart4)
  {
    parse_uart_byte(uart4_rx);
    proto_uart_rearm(&huart4);
  }
  else if (huart == &huart2)
  {
    // USART2 is debug only. Do not allow it to control motion.
    proto_uart_rearm(&huart2);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart4)
  {
    __HAL_UART_CLEAR_PEFLAG(&huart4);
    __HAL_UART_CLEAR_FEFLAG(&huart4);
    __HAL_UART_CLEAR_NEFLAG(&huart4);
    __HAL_UART_CLEAR_OREFLAG(&huart4);
    proto_uart_rearm(&huart4);
  }
  else if (huart == &huart2)
  {
    __HAL_UART_CLEAR_PEFLAG(&huart2);
    __HAL_UART_CLEAR_FEFLAG(&huart2);
    __HAL_UART_CLEAR_NEFLAG(&huart2);
    __HAL_UART_CLEAR_OREFLAG(&huart2);
    proto_uart_rearm(&huart2);
  }
}
/* USER CODE END 4 */

void StartDefaultTask(void *argument)
{
  for (;;)
  {
    // Safety watchdog: if valid VEL packets stop, force motor stop.
    if ((HAL_GetTick() - last_cmd_time_ms) > 500U)
    {
      apply_velocity(0, 0);
    }

    osDelay(20);
  }
}

void StartControlTask(void *argument)
{
  VelMsg_t msg;

  for (;;)
  {
    if (osMessageQueueGet(velQueueHandle, &msg, NULL, 20) == osOK)
    {
      apply_velocity(msg.left_pwm, msg.right_pwm);
    }

    osDelay(20);
  }
}

void StartEncoderTask(void *argument)
{
  uint16_t last_left_ticks = 0;
  uint16_t last_right_ticks = 0;

  int32_t left_total_ticks = 0;
  int32_t right_total_ticks = 0;

  float left_distance_cm = 0.0f;
  float right_distance_cm = 0.0f;

  uint32_t enc_seq = 0;
  char tx_buf[128];

  last_left_ticks = (uint16_t)__HAL_TIM_GET_COUNTER(&htim1);
  last_right_ticks = (uint16_t)__HAL_TIM_GET_COUNTER(&htim8);

  for (;;)
  {
    uint16_t current_left_ticks = (uint16_t)__HAL_TIM_GET_COUNTER(&htim1);
    uint16_t current_right_ticks = (uint16_t)__HAL_TIM_GET_COUNTER(&htim8);

    int16_t left_delta = -encoder_delta_16bit(current_left_ticks, last_left_ticks);
    int16_t right_delta = -encoder_delta_16bit(current_right_ticks, last_right_ticks);

    last_left_ticks = current_left_ticks;
    last_right_ticks = current_right_ticks;

    left_total_ticks += left_delta;
    right_total_ticks += right_delta;

    left_distance_cm += ((float)left_delta * CM_PER_TICK);
    right_distance_cm += ((float)right_delta * CM_PER_TICK);

    int tx_len = snprintf(
        tx_buf,
        sizeof(tx_buf),
        "ENC,%lu,%lu,%ld,%ld,%ld,%ld\r\n",
        (unsigned long)enc_seq++,
        (unsigned long)HAL_GetTick(),
        (long)left_delta,
        (long)right_delta,
        (long)left_total_ticks,
        (long)right_total_ticks
    );

    if (tx_len > 0)
    {
      proto_uart_tx((const uint8_t *)tx_buf, (uint16_t)tx_len);
    }

    osDelay(50);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
