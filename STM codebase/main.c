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
  *****************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

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
volatile uint8_t uart4_rx;          // already have this, keep as volatile
osMessageQueueId_t cmdQueueHandle;
volatile uint32_t last_cmd_time_ms = 0;  // watchdog: updated on every valid UART4 command

typedef struct
{
  char cmd;
} CmdMsg_t;

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
// ===== Shared PWM (Speed control for all motors) =====
static inline void set_speed(uint16_t duty)
{
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, duty);  // TIM4 CH2
}

static inline int16_t encoder_delta_16bit(uint16_t now, uint16_t previous)
{
  return (int16_t)(now - previous);
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

// ----- LEFT side = M1 (front-left: PA10/PB5) + M3 (rear-left: PB14/PB15)
static inline void left_forward(void) {
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);    // M1 IN1
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,  GPIO_PIN_RESET);  // M1 IN2
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);  // M3 IN1
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);    // M3 IN2
}

static inline void left_reverse(void) {
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);  // M1 IN1
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,  GPIO_PIN_SET);    // M1 IN2
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);    // M3 IN1
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);  // M3 IN2
}

// ----- RIGHT side = M2 (front-right: PB4/PB10) + M4 (rear-right: PA6/PA5)
static inline void right_forward(void) {
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,  GPIO_PIN_SET);    // M2 IN1
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);  // M2 IN2
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6,  GPIO_PIN_RESET);  // M4 IN1
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,  GPIO_PIN_SET);    // M4 IN2
}

static inline void right_reverse(void) {
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,  GPIO_PIN_RESET);  // M2 IN1
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);    // M2 IN2
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6,  GPIO_PIN_SET);    // M4 IN1
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,  GPIO_PIN_RESET);  // M4 IN2
}

// ----- BRAKE: all INx = LOW
static inline void left_brake(void) {
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);  // M1 IN1
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,  GPIO_PIN_RESET);  // M1 IN2
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);  // M3 IN1
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);  // M3 IN2
}

static inline void right_brake(void) {
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,  GPIO_PIN_RESET);  // M2 IN1
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);  // M2 IN2
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6,  GPIO_PIN_RESET);  // M4 IN1
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,  GPIO_PIN_RESET);  // M4 IN2
}



#define SPEED_DRIVE     49   // 0..49 (your PWM period=49)
#define SPEED_TURN      32
#define SPEED_BRAKE     20   // ~41% duty — counter-inertia pulse, not strong enough to reverse
#define BRAKE_PULSE_MS  60   // duration of counter-brake pulse in ms; tune if robot creeps backward

static volatile char current_cmd = 'S';
static char applied_cmd = '?';

static inline void apply_cmd(char cmd)
{
  switch (cmd)
  {
    case 'F': // forward both pairs
		// Forward
    	left_forward(); right_forward();
    	set_speed(SPEED_DRIVE);
      break;

    case 'B': // backward both pairs
    	left_reverse(); right_reverse();
    	set_speed(SPEED_DRIVE);
      break;

    case 'R': // rotate right
  	  // printf("MOVING RIGHT%c\r\n");
       	right_reverse(); left_forward();
    	set_speed(SPEED_TURN);
      break;

    case 'L': // rotate left
    	right_forward(); left_reverse();
    	set_speed(SPEED_TURN);

      break;

    case 'S': // stop
    default:
    	set_speed(0);
    	left_brake(); right_brake();
      break;
  }
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

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM8_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim4,  TIM_CHANNEL_1);   // ENB on PC7 (D9)
  HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);

  // Zero encoders
  __HAL_TIM_SET_COUNTER(&htim8, 0);
  __HAL_TIM_SET_COUNTER(&htim1, 0);

  // Optional hello over Debugging for RTOS
  // Startup debug messages
  const char hello_uart4[] = "DBG,UART4 command link ready\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)hello_uart4, strlen(hello_uart4), HAL_MAX_DELAY);
  HAL_UART_Transmit(&huart4, (uint8_t*)hello_uart4, strlen(hello_uart4), HAL_MAX_DELAY);

  /* USER CODE END 2 */

  last_cmd_time_ms = HAL_GetTick();  // seed watchdog so robot doesn't stop on boot

  /* Init scheduler */
  osKernelInitialize();
  cmdQueueHandle = osMessageQueueNew(8, sizeof(CmdMsg_t), NULL);
  if (cmdQueueHandle == NULL) Error_Handler();
  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  if (defaultTaskHandle == NULL) Error_Handler();

  controlTaskHandle = osThreadNew(StartControlTask, NULL, &controlTask_attributes);
  if (controlTaskHandle == NULL) Error_Handler();

  encoderTaskHandle = osThreadNew(StartEncoderTask, NULL, &encoderTask_attributes);
  if (encoderTaskHandle == NULL) Error_Handler();

  // Start RTOS RX interrupt (UART4 on PA0/PA1)
  HAL_UART_Receive_IT(&huart4, (uint8_t*)&uart4_rx, 1);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
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

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
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

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
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
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
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
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
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
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
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
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
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
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, D13_Pin|D12_Pin|D2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, D6_Pin|IN1_2_Pin|IN2_2_Pin|D5_Pin
                          |D4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : D13_Pin D12_Pin D2_Pin */
  GPIO_InitStruct.Pin = D13_Pin|D12_Pin|D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : D6_Pin IN1_2_Pin IN2_2_Pin D5_Pin
                           D4_Pin */
  GPIO_InitStruct.Pin = D6_Pin|IN1_2_Pin|IN2_2_Pin|D5_Pin
                          |D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart4)
  {
	  // RAW DEBUG — bypass queue entirely
	  char dbg[24];
	    int n = snprintf(dbg, sizeof(dbg), "RX:0x%02X ('%c')\r\n", uart4_rx, uart4_rx);
	    HAL_UART_Transmit(&huart2, (uint8_t*)dbg, n, 500);
	  // ----
    char c = (uart4_rx >= 'a' && uart4_rx <= 'z') ? (uart4_rx - 'a' + 'A') : uart4_rx;

    if (c == 'F' || c == 'B' || c == 'L' || c == 'R' || c == 'S')
    {
      last_cmd_time_ms = HAL_GetTick();  // reset watchdog on every valid command
      CmdMsg_t msg;
      msg.cmd = c;
      osMessageQueuePut(cmdQueueHandle, &msg, 0, 0);
    }

    HAL_UART_Receive_IT(&huart4, (uint8_t*)&uart4_rx, 1);
  }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart4)
    {
        // Clear error flags and re-arm reception unconditionally
        __HAL_UART_CLEAR_PEFLAG(&huart4);
        __HAL_UART_CLEAR_FEFLAG(&huart4);
        __HAL_UART_CLEAR_NEFLAG(&huart4);
        __HAL_UART_CLEAR_OREFLAG(&huart4);

        HAL_UART_Receive_IT(&huart4, (uint8_t*)&uart4_rx, 1);
    }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  char last_cmd = 0;
  char last_motion = 'S';  // last executed F or B — used to pick counter-brake direction

  for (;;)
  {
    // Safety watchdog: if no valid command received from Pi in 500 ms, force stop.
    // Protects against Pi crash, UART disconnect, or bridge process dying mid-motion.
    if ((HAL_GetTick() - last_cmd_time_ms) > 500)
    {
      current_cmd = 'S';
    }

    if (current_cmd != last_cmd)
    {
      char cmd = current_cmd;  // snapshot — current_cmd may change during the brake delay

      if (cmd == 'S' && (last_motion == 'F' || last_motion == 'B'))
      {
        // Active counter-brake: momentary reverse of travel direction, then hard stop.
        // Snapshot direction so we act on what the robot was doing, not what arrives next.
        if (last_motion == 'F') { left_reverse();  right_reverse(); }
        else                    { left_forward(); right_forward(); }
        set_speed(SPEED_BRAKE);
        osDelay(BRAKE_PULSE_MS);
        set_speed(0);
        left_brake(); right_brake();
      }
      else
      {
        apply_cmd(cmd);
      }

      if (cmd == 'F' || cmd == 'B') last_motion = cmd;
      last_cmd = cmd;
    }

    osDelay(10);
  }
}

void StartControlTask(void *argument)
{
  CmdMsg_t msg;
  char logbuf[32];

  for (;;)
  {
    if (osMessageQueueGet(cmdQueueHandle, &msg, NULL, osWaitForever) == osOK)
    {
      current_cmd = msg.cmd;
      int len = snprintf(logbuf, sizeof(logbuf), "CMD: %c\r\n", current_cmd);
      HAL_UART_Transmit(&huart2, (uint8_t*)logbuf, len, HAL_MAX_DELAY);
    }
    osDelay(50);
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
  char dbg_buf[128];

  // IMPORTANT:
  // Current assumption:
  // TIM1 = left encoder
  // TIM8 = right encoder
  // If your physical robot shows the opposite, swap htim1 and htim8 below.

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

    uint32_t time_ms = HAL_GetTick();

    /*
      Structured message to Raspberry Pi over UART4:

      ENC,seq,time_ms,left_delta,right_delta,left_total,right_total

      Example:
      ENC,15,2300,4,5,120,118
    */
    int tx_len = snprintf(
        tx_buf,
        sizeof(tx_buf),
        "ENC,%lu,%lu,%ld,%ld,%ld,%ld\r\n",
        (unsigned long)enc_seq++,
        (unsigned long)time_ms,
        (long)left_delta,
        (long)right_delta,
        (long)left_total_ticks,
        (long)right_total_ticks
    );

    HAL_UART_Transmit(&huart4, (uint8_t*)tx_buf, tx_len, 20);

    // Human-readable debug copy to USART2
    int dbg_len = snprintf(
        dbg_buf,
        sizeof(dbg_buf),
        "DBG,Ld:%ld Rd:%ld Ltot:%ld Rtot:%ld Lcm:%.2f Rcm:%.2f\r\n",
        (long)left_delta,
        (long)right_delta,
        (long)left_total_ticks,
        (long)right_total_ticks,
        left_distance_cm,
        right_distance_cm
    );

    HAL_UART_Transmit(&huart2, (uint8_t*)dbg_buf, dbg_len, 50);

    osDelay(50);
  }
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
