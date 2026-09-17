/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define ALIVE_SYSTEM_TASK   (1U << 0)
#define ALIVE_CAN_TX_TASK   (1U << 1)

volatile uint32_t task_alive_flags = 0; // 任务健康标志

#define PARAM_MAGIC          0x12345678U // 人为约定一个特殊标志
#define PARAM_FLASH_PAGE     63U
#define PARAM_FLASH_ADDRESS  0x0801F800U

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for CAN_Rx_Task */
osThreadId_t CAN_Rx_TaskHandle;
const osThreadAttr_t CAN_Rx_Task_attributes = {
  .name = "CAN_Rx_Task",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};
/* Definitions for CAN_Tx_Task */
osThreadId_t CAN_Tx_TaskHandle;
const osThreadAttr_t CAN_Tx_Task_attributes = {
  .name = "CAN_Tx_Task",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};
/* Definitions for System_Task */
osThreadId_t System_TaskHandle;
const osThreadAttr_t System_Task_attributes = {
  .name = "System_Task",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};
/* Definitions for Watchdog_Task */
osThreadId_t Watchdog_TaskHandle;
const osThreadAttr_t Watchdog_Task_attributes = {
  .name = "Watchdog_Task",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for ControlCommandQueue */
osMessageQueueId_t ControlCommandQueueHandle;
const osMessageQueueAttr_t ControlCommandQueue_attributes = {
  .name = "ControlCommandQueue"
};
/* USER CODE BEGIN PV */

int cnt1 = 0, cnt2 = 0;

volatile uint16_t count_now = 0;
volatile uint16_t count_last = 0;
volatile int16_t delta_count = 0;
volatile float rpm = 0.0f;

volatile float target_rpm = 80.0f;   // 目标转速
volatile float error = 0.0f;          // 转速误差
volatile float kp = 4.0f;             // P控制比例系数

volatile float integral = 0.0f;  // I控制
volatile float ki = 0.2f;

volatile int32_t pwm_value = 480;

FDCAN_TxHeaderTypeDef txHeader;	// 发送头
uint8_t txData[4]; 				// 发送缓冲区

FDCAN_TxHeaderTypeDef txHeader202;
uint8_t txData202[4];

FDCAN_TxHeaderTypeDef txHeader203;
uint8_t txData203[4];

FDCAN_RxHeaderTypeDef rxHeader;	// 接受头
uint8_t rxData[8];				// 接受缓冲区

uint32_t last_report_tick = 0; 	// 用于发送电机数据

uint32_t last_local_cmd_tick = 0; 	// system_task断线超时检测
uint32_t last_remote_cmd_tick = 0;	// system_task断线超时检测

uint32_t stall_start_tick = 0; 	// 堵转开始时间
uint8_t stall_detecting = 0;	// 堵转检查
uint8_t stall_fault = 0;		// 堵转故障
uint8_t encoder_fault = 0;		// 编码器错误

// 简单状态机：PC控制-本地控制切换
typedef enum
{
    CONTROL_LOCAL = 0,
    CONTROL_REMOTE
} ControlMode;

volatile ControlMode control_mode = CONTROL_LOCAL;

volatile uint16_t local_target_rpm = 0;
volatile uint16_t remote_target_rpm = 0;

// 用于解耦的控制命令消息
typedef enum
{
    CMD_LOCAL_TARGET,
    CMD_REMOTE_CONTROL,
	CMD_CLEAR_FAULT, 	// fault状态机引入
	CMD_SET_PI_PARAMS, 	// flash修改引入
	CMD_SAVE_PI_PARAMS	// flash保存
} CommandType; 		// 控制类型

typedef struct
{
    CommandType type;

    uint16_t target_rpm;
    uint8_t mode;

    uint16_t kp_raw;	// flash修改引入
    uint16_t ki_raw;
} ControlCommand; 	// 控制指令

typedef enum
{
    SYSTEM_NORMAL = 0,
    SYSTEM_FAULT
} SystemState; 		// 系统状态

SystemState system_state = SYSTEM_NORMAL;

typedef enum
{
    FAULT_NONE = 0,
    FAULT_COMM_TIMEOUT, // 只有这个允许自动恢复
    FAULT_STALL,
    FAULT_ENCODER,
} FaultCode; 		// 错误码

FaultCode fault_code = FAULT_NONE;

// G4 的 Flash 编程单位不是随便按 32 位写、先用一次写8字节比较稳
typedef struct
{
    uint32_t magic; // 判断 Flash 这一块里的数据，是不是我们自己之前保存过的有效参数
    float kp;
    float ki;
    uint32_t reserved;
} PI_Params;

PI_Params *test_params; // 测试是否写入

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
static void MX_IWDG_Init(void);
void StartDefaultTask(void *argument);
void StartCAN_Rx_Task(void *argument);
void StartCAN_Tx_Task(void *argument);
void StartSystem_Task(void *argument);
void StartWatchdog_Task(void *argument);

/* USER CODE BEGIN PFP */

HAL_StatusTypeDef SavePIParams(float kp, float ki);
void LoadPIParams(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_FDCAN1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  LoadPIParams();

  test_params = (PI_Params *)PARAM_FLASH_ADDRESS;

  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST))
  {
      // 说明上一次是IWDG复位
	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	  HAL_Delay(500);
	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
  }

  __HAL_RCC_CLEAR_RESET_FLAGS();


  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); 		// 启动PWM
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); 	// 启动编码器
  HAL_TIM_Base_Start_IT(&htim6); 					// 启动TIM6中断

  FDCAN_FilterTypeDef sFilterConfig;

  // 配置过滤器
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;

  sFilterConfig.FilterIndex = 0;	// 过滤槽 Filter0
  sFilterConfig.FilterID1 = 0x123;	// 只接收 ID = 0x123
  sFilterConfig.FilterID2 = 0x7FF;	// 11 位标准 ID 的全掩码

  HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);

  sFilterConfig.FilterIndex = 1;	// 过滤槽 Filter1
  sFilterConfig.FilterID1 = 0x301;	// 只接收 ID = 0x301
  sFilterConfig.FilterID2 = 0x7FF;

  HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);

  sFilterConfig.FilterIndex = 2;	// 过滤槽 Filter2
  sFilterConfig.FilterID1 = 0x302;	// 只接收 ID = 0x302
  sFilterConfig.FilterID2 = 0x7FF;

  HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);

  sFilterConfig.FilterIndex = 3;	// 过滤槽 Filter3
  sFilterConfig.FilterID1 = 0x303;	// 只接收 ID = 0x303
  sFilterConfig.FilterID2 = 0x7FF;

  HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);

  HAL_FDCAN_ConfigGlobalFilter(
      &hfdcan1,
      FDCAN_REJECT, 		// 未匹配的 Standard ID → Reject
      FDCAN_REJECT,			// 未匹配的 Extended ID → Reject
      FDCAN_REJECT_REMOTE,	// Standard Remote Frame → Reject
      FDCAN_REJECT_REMOTE	// Extended Remote Frame → Reject
  );

  HAL_FDCAN_Start(&hfdcan1);
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0); // 启动FDCAN中断

  // 初始化发送头
  txHeader.Identifier = 0x201;
  txHeader.IdType = FDCAN_STANDARD_ID;
  txHeader.TxFrameType = FDCAN_DATA_FRAME;
  txHeader.DataLength = FDCAN_DLC_BYTES_4;
  txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  txHeader.BitRateSwitch = FDCAN_BRS_OFF;
  txHeader.FDFormat = FDCAN_CLASSIC_CAN;
  txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  txHeader.MessageMarker = 0;

  txHeader202.Identifier = 0x202;
  txHeader202.IdType = FDCAN_STANDARD_ID;
  txHeader202.TxFrameType = FDCAN_DATA_FRAME;
  txHeader202.DataLength = FDCAN_DLC_BYTES_4;
  txHeader202.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  txHeader202.BitRateSwitch = FDCAN_BRS_OFF;
  txHeader202.FDFormat = FDCAN_CLASSIC_CAN;
  txHeader202.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  txHeader202.MessageMarker = 0;

  txHeader203.Identifier = 0x203;
  txHeader203.IdType = FDCAN_STANDARD_ID;
  txHeader203.TxFrameType = FDCAN_DATA_FRAME;
  txHeader203.DataLength = FDCAN_DLC_BYTES_4;
  txHeader203.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  txHeader203.BitRateSwitch = FDCAN_BRS_OFF;
  txHeader203.FDFormat = FDCAN_CLASSIC_CAN;
  txHeader203.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  txHeader203.MessageMarker = 0;

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of ControlCommandQueue */
  ControlCommandQueueHandle = osMessageQueueNew (8, sizeof(ControlCommand), &ControlCommandQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of CAN_Rx_Task */
  CAN_Rx_TaskHandle = osThreadNew(StartCAN_Rx_Task, NULL, &CAN_Rx_Task_attributes);

  /* creation of CAN_Tx_Task */
  CAN_Tx_TaskHandle = osThreadNew(StartCAN_Tx_Task, NULL, &CAN_Tx_Task_attributes);

  /* creation of System_Task */
  System_TaskHandle = osThreadNew(StartSystem_Task, NULL, &System_Task_attributes);

  /* creation of Watchdog_Task */
  Watchdog_TaskHandle = osThreadNew(StartWatchdog_Task, NULL, &Watchdog_Task_attributes);

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
  }
  /* USER CODE END 3 */
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 2;
  hfdcan1.Init.NominalSyncJumpWidth = 2;
  hfdcan1.Init.NominalTimeSeg1 = 13;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 4;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_16;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 3999;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 799;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 480;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 15999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 99;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

HAL_StatusTypeDef SavePIParams(float kp_value, float ki_value)
{
	PI_Params *old_params = (PI_Params *)PARAM_FLASH_ADDRESS;

	/* Flash里已有有效参数 */
	if (old_params->magic == PARAM_MAGIC)
	{
		/* 新参数与旧参数相同，则不重复擦写 */
		if (fabsf(old_params->kp - kp_value) < 0.0001f && fabsf(old_params->ki - ki_value) < 0.0001f)
		{
			return HAL_OK;
		}
	}

	// /* 下面才是真正的擦除 + 写入 */
    HAL_StatusTypeDef status;
    FLASH_EraseInitTypeDef eraseInit;
    uint32_t pageError = 0;

    PI_Params params;
    params.magic = PARAM_MAGIC;
    params.kp = kp_value;
    params.ki = ki_value;
    params.reserved = 0xFFFFFFFFU;

    HAL_FLASH_Unlock();

    eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseInit.Banks = FLASH_BANK_1;
    eraseInit.Page = PARAM_FLASH_PAGE;
    eraseInit.NbPages = 1;

    status = HAL_FLASHEx_Erase(&eraseInit, &pageError);

    if (status != HAL_OK)
    {
        HAL_FLASH_Lock();
        return status;
    }

    uint64_t *src = (uint64_t *)&params;

    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, PARAM_FLASH_ADDRESS, src[0]);

    if (status != HAL_OK)
    {
        HAL_FLASH_Lock();
        return status;
    }

    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, PARAM_FLASH_ADDRESS + 8, src[1]);

    HAL_FLASH_Lock();

    return status;
}

void LoadPIParams(void)
{
    PI_Params *params = (PI_Params *)PARAM_FLASH_ADDRESS;

    if (params->magic == PARAM_MAGIC)
    {
    	// 如果是之前约定好的参数
        kp = params->kp;
        ki = params->ki;
    }
    else
    {
        kp = 4.0f;
        ki = 0.2f;
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (hfdcan->Instance == FDCAN1)
    {
    	// 通知FDCAN1有消息帧来了
        if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
        {
        	/* RTOS 内核会知道某个 Task 正在等 0x01
        	 * 现在 0x01 被 set，Task 从 Blocked → Ready
        	 * 所以不是 Task 自己一直轮询“有没有被唤醒”，而是 RTOS 内核在管理它的等待条件
        	 * */
            osThreadFlagsSet(CAN_Rx_TaskHandle, 0x01);
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
    	// TIM6中断 -> 100ms触发一次
        count_now = __HAL_TIM_GET_COUNTER(&htim3);

        delta_count = (int16_t)(count_now - count_last);

        rpm = delta_count * 0.4f;

        count_last = count_now;

        // PI 控制
        error = target_rpm - rpm;

        float p_term = kp * error;

		float i_candidate = integral + error * 0.1f; // 抗饱和积分

		float output_candidate = 480.0f + p_term + ki * i_candidate;

		// 只有积分不会把输出继续推向饱和时，才接受这次积分
		if (!((output_candidate >= 799.0f && error > 0.0f) || (output_candidate <= 0.0f && error < 0.0f)))
		{
			integral = i_candidate;
		}

		float output = 480.0f + kp * error + ki * integral;

		if (output > 799.0f)
			output = 799.0f;

		if (output < 0.0f)
			output = 0.0f;

		pwm_value = (int32_t)output;

		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_value);
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
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
//	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	osDelay(500);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartCAN_Rx_Task */
/**
* @brief Function implementing the CAN_Rx_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCAN_Rx_Task */
void StartCAN_Rx_Task(void *argument)
{
  /* USER CODE BEGIN StartCAN_Rx_Task */
  /* Infinite loop */
  for(;;)
  {
	// 当前 Task 等待线程标志 0x01，一直等到有人设置这个标志，若没有则进入 Blocked
	osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);

	while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0)
	{
		// 收帧 → 看ID → 解析 → 丢进Queue → 结束

		HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rxHeader, rxData);

		if (rxHeader.Identifier == 0x123)
		{
			ControlCommand cmd;

			cmd.type = CMD_LOCAL_TARGET;
			cmd.target_rpm = ((uint16_t)rxData[0] << 8) | rxData[1];

			osMessageQueuePut(ControlCommandQueueHandle, &cmd, 0, 0);
		}

		else if (rxHeader.Identifier == 0x301)
		{
			ControlCommand cmd;

			cmd.type = CMD_REMOTE_CONTROL;
			cmd.mode = rxData[0];

			cmd.target_rpm = ((uint16_t)rxData[1] << 8) | rxData[2];

			osMessageQueuePut(ControlCommandQueueHandle, &cmd, 0, 0);
		}

		else if (rxHeader.Identifier == 0x302)
		{
		    if (rxData[0] == 0x01)
		    {
		        ControlCommand cmd = {0};

		        cmd.type = CMD_CLEAR_FAULT;

		        osMessageQueuePut(ControlCommandQueueHandle, &cmd, 0, 0);
		    }
		}

		else if (rxHeader.Identifier == 0x303)
		{
	        ControlCommand cmd = {0};

		    if (rxData[0] == 0x01)
		    {
		        cmd.type = CMD_SET_PI_PARAMS;

		        cmd.kp_raw = ((uint16_t)rxData[1] << 8) | rxData[2];
		        cmd.ki_raw = ((uint16_t)rxData[3] << 8) | rxData[4];

		        osMessageQueuePut(ControlCommandQueueHandle, &cmd, 0, 0);
		    }
		    else if (rxData[0] == 0x02)
			{
				cmd.type = CMD_SAVE_PI_PARAMS;

				osMessageQueuePut(ControlCommandQueueHandle, &cmd, 0, 0);
			}
		}
	}
  }
  /* USER CODE END StartCAN_Rx_Task */
}

/* USER CODE BEGIN Header_StartCAN_Tx_Task */
/**
* @brief Function implementing the CAN_Tx_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCAN_Tx_Task */
void StartCAN_Tx_Task(void *argument)
{
  /* USER CODE BEGIN StartCAN_Tx_Task */
  /* Infinite loop */
  for(;;)
  {
	taskENTER_CRITICAL();
	task_alive_flags |= ALIVE_CAN_TX_TASK; // 看门狗
	taskEXIT_CRITICAL();

	// 向上位机反馈：实际转速，PWM值
	if (HAL_GetTick() - last_report_tick >= 100)
	{
		last_report_tick = HAL_GetTick();

		float rpm_snapshot;
		int32_t pwm_snapshot;

		taskENTER_CRITICAL();
		rpm_snapshot = rpm; // 快照：防止中途被中断从而一个周期内放置不同场景的数据
		pwm_snapshot = pwm_value;
		taskEXIT_CRITICAL();

		uint16_t rpm_u16 = (uint16_t)rpm_snapshot;
		uint16_t pwm_u16 = (uint16_t)pwm_snapshot;

		txData[0] = (rpm_u16 >> 8) & 0xFF;
		txData[1] = rpm_u16 & 0xFF;
		txData[2] = (pwm_u16 >> 8) & 0xFF;
		txData[3] = pwm_u16 & 0xFF;

		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, txData);

		/*
		 * Byte0: system_state	→	0 = SYSTEM_NORMAL	1 = SYSTEM_FAUL
		 * Byte1: fault_code 	→	0 = FAULT_NONE、 	1 = FAULT_COMM_TIMEOUT	 2 = FAULT_STALL   3 = FAULT_ENCODER
		 * Byte2: control_mode 	→ 	0 = LOCAL、 			1 = REMOTE
		 * */

		txData202[0] = (uint8_t)system_state;
		txData202[1] = (uint8_t)fault_code;
		txData202[2] = (uint8_t)control_mode;
		txData202[3] = 0;

		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader202, txData202);

		float kp_snapshot;
		float ki_snapshot;

		taskENTER_CRITICAL();
		kp_snapshot = kp;
		ki_snapshot = ki;
		taskEXIT_CRITICAL();

		uint16_t kp_u16 = (uint16_t)lroundf(kp_snapshot * 100.0f);
		uint16_t ki_u16 = (uint16_t)lroundf(ki_snapshot * 1000.0f);

		txData203[0] = (kp_u16 >> 8) & 0xFF;
		txData203[1] = kp_u16 & 0xFF;
		txData203[2] = (ki_u16 >> 8) & 0xFF;
		txData203[3] = ki_u16 & 0xFF;

		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader203, txData203);

	}
    osDelay(1);
  }
  /* USER CODE END StartCAN_Tx_Task */
}

/* USER CODE BEGIN Header_StartSystem_Task */
/**
* @brief Function implementing the System_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSystem_Task */
void StartSystem_Task(void *argument)
{
  /* USER CODE BEGIN StartSystem_Task */
  /* Infinite loop */

  ControlCommand cmd;

  for(;;)
  {
	/* 最多阻塞50ms
	 * 50ms内有消息就立刻返回 osOK
	 * 50ms内没消息就超时返回
	 * */
	taskENTER_CRITICAL();
	task_alive_flags |= ALIVE_SYSTEM_TASK; // 看门狗
	taskEXIT_CRITICAL();

	if (osMessageQueueGet(ControlCommandQueueHandle, &cmd, NULL, 50) == osOK)
	{
		if (cmd.type == CMD_LOCAL_TARGET)
		{
			local_target_rpm = cmd.target_rpm;
			last_local_cmd_tick = HAL_GetTick(); // 获取时间戳，判断超时用

			if (control_mode == CONTROL_LOCAL)
			{
				// CAN超时可自动恢复，无需PC手动抹除错误
				if (system_state == SYSTEM_FAULT && fault_code == FAULT_COMM_TIMEOUT)
				{
					system_state = SYSTEM_NORMAL;
					fault_code = FAULT_NONE;
				}

				if (system_state == SYSTEM_NORMAL)
				{
					target_rpm = (float)local_target_rpm;
				}
			}
		}

		else if (cmd.type == CMD_REMOTE_CONTROL)
		{
			if (cmd.mode == 0)
			{
				control_mode = CONTROL_LOCAL;
				target_rpm = (float)local_target_rpm;
			}

			else if (cmd.mode == 1)
			{
				control_mode = CONTROL_REMOTE;
				remote_target_rpm = cmd.target_rpm;
				last_remote_cmd_tick = HAL_GetTick(); // 获取时间戳，判断超时用

				// CAN超时可自动恢复，无需PC手动抹除错误
				if (system_state == SYSTEM_FAULT && fault_code == FAULT_COMM_TIMEOUT)
				{
					system_state = SYSTEM_NORMAL;
					fault_code = FAULT_NONE;
				}

				if (system_state == SYSTEM_NORMAL)
				{
					target_rpm = (float)remote_target_rpm;
				}
			}
		}

		else if (cmd.type == CMD_CLEAR_FAULT)
		{
		    if (system_state == SYSTEM_FAULT)
		    {
		        if (fabsf(rpm) < 10.0f)
		        {
		            system_state = SYSTEM_NORMAL; // 清除错误状态
		            fault_code = FAULT_NONE;
		            stall_detecting = 0;
		            target_rpm = 0;
		        }
		    }
		}

		else if (cmd.type == CMD_SET_PI_PARAMS)
		{
		    kp = (float)cmd.kp_raw / 100.0f;
		    ki = (float)cmd.ki_raw / 1000.0f;
		}

		else if (cmd.type == CMD_SAVE_PI_PARAMS)
		{
		    SavePIParams(kp, ki);
		}
	}

	uint32_t now = HAL_GetTick();

	// CAN通信超时检测
	if (control_mode == CONTROL_LOCAL)
	{
		if ((uint32_t)(now - last_local_cmd_tick) > 500)
		{
			// 只有当前没有更严重锁存故障时，COMM_TIMEOUT 才允许写入 fault_code
			if (fault_code == FAULT_NONE || fault_code == FAULT_COMM_TIMEOUT)
			{
				system_state = SYSTEM_FAULT;
				fault_code = FAULT_COMM_TIMEOUT;
			}
		}
	}
	else if (control_mode == CONTROL_REMOTE)
	{
		if ((uint32_t)(now - last_remote_cmd_tick) > 500)
		{
			// 只有当前没有更严重锁存故障时，COMM_TIMEOUT 才允许写入 fault_code
			if (fault_code == FAULT_NONE || fault_code == FAULT_COMM_TIMEOUT)
			{
				system_state = SYSTEM_FAULT;
				fault_code = FAULT_COMM_TIMEOUT;
			}
		}
	}

	// 堵转检测
	if (target_rpm > 100 && pwm_value > 500 && fabsf(rpm) < 10.0f)
	{
		if (!stall_detecting)
		{
			stall_detecting = 1; // 堵转标记
			stall_start_tick = now;
		}
		else
		{
			if ((uint32_t)(now - stall_start_tick) > 1000)
			{
				system_state = SYSTEM_FAULT;
				fault_code = FAULT_STALL;
			}
		}
	}
	else
	{
		stall_detecting = 0;
	}

	// 编码器测速错误检测
	if (fabsf(rpm) > 288.0f)
	{
		system_state = SYSTEM_FAULT;
		fault_code = FAULT_ENCODER;
	}

	/* 故障拥有最高优先级 */
	if (system_state == SYSTEM_FAULT)
	{
	    target_rpm = 0;
	}

  }
  /* USER CODE END StartSystem_Task */
}

/* USER CODE BEGIN Header_StartWatchdog_Task */
/**
* @brief Function implementing the Watchdog_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartWatchdog_Task */
void StartWatchdog_Task(void *argument)
{
  /* USER CODE BEGIN StartWatchdog_Task */
  /* Infinite loop */
  for(;;)
  {
	  osDelay(500);

	  uint32_t flags;

	  taskENTER_CRITICAL();

	  flags = task_alive_flags;
	  task_alive_flags = 0;

	  taskEXIT_CRITICAL();

	  if ((flags & (ALIVE_SYSTEM_TASK | ALIVE_CAN_TX_TASK)) == (ALIVE_SYSTEM_TASK | ALIVE_CAN_TX_TASK))
	  {
		  HAL_IWDG_Refresh(&hiwdg);
	  }
  }
  /* USER CODE END StartWatchdog_Task */
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
