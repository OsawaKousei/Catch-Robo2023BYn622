/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <CAN_Main.h>
#include <UDPController.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_tx;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 512 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for systemCheckTask */
osThreadId_t systemCheckTaskHandle;
uint32_t systemCheckTaskBuffer[ 512 ];
osStaticThreadDef_t systemCheckTaskControlBlock;
const osThreadAttr_t systemCheckTask_attributes = {
  .name = "systemCheckTask",
  .cb_mem = &systemCheckTaskControlBlock,
  .cb_size = sizeof(systemCheckTaskControlBlock),
  .stack_mem = &systemCheckTaskBuffer[0],
  .stack_size = sizeof(systemCheckTaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ControllerTask */
osThreadId_t ControllerTaskHandle;
uint32_t ControllerTaskBuffer[ 512 ];
osStaticThreadDef_t ControllerTaskControlBlock;
const osThreadAttr_t ControllerTask_attributes = {
  .name = "ControllerTask",
  .cb_mem = &ControllerTaskControlBlock,
  .cb_size = sizeof(ControllerTaskControlBlock),
  .stack_mem = &ControllerTaskBuffer[0],
  .stack_size = sizeof(ControllerTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal7,
};
/* USER CODE BEGIN PV */
NUM_OF_DEVICES num_of_devices;
MCMD_HandleTypedef mcmd4_struct;
MCMD_Feedback_Typedef mcmd_fb;//MCMDからのフィードバックを受け取る構造体を定義

// CANモジュール基板の設定
CANServo_Param_Typedef servo_param;
CAN_Device servo_device;

//Air基盤の設定
CAN_Device air_device;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_CAN1_Init(void);
void StartDefaultTask(void *argument);
void StartSystemCheckTask(void *argument);
void StartControllerTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(uint8_t ch)
#else
#define PUTCHAR_PROTYPE int fputc(int ch,FILE *f)
#endif

PUTCHAR_PROTOTYPE {
    HAL_UART_Transmit(&huart3, &ch, 1, 500);
    return ch;
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan){
    WhenTxMailbox0_1_2CompleteCallbackCalled();
}

void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan){
    WhenTxMailbox0_1_2AbortCallbackCalled();
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan){
    WhenTxMailbox0_1_2CompleteCallbackCalled();
}

void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan){
    WhenTxMailbox0_1_2AbortCallbackCalled();
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan){
    WhenTxMailbox0_1_2CompleteCallbackCalled();
}

void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan){
    WhenTxMailbox0_1_2AbortCallbackCalled();
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
    WhenCANRxFifo0MsgPending(hcan, &num_of_devices);
}

void canSetting(){
	printf("Start Initializing CAN System:Begin\n\r");
	HAL_Delay(100);

	CAN_SystemInit(&hcan1); // F7のCAN通信のinit

	// デバイス数の設定 (今回はmcmd4が1枚)
	num_of_devices.mcmd3 = 1;
	num_of_devices.mcmd4 = 0;
	num_of_devices.air = 0;
	num_of_devices.servo = 0;

	printf("Start Initializing CAN System:End\n\r");
	HAL_Delay(100);
	//CAN_WaitConnect(&num_of_devices);  // 設定された全てのCANモジュール基板との接続が確認できるまで待機
}

void mcmdSetting(){
	   // ここからはCANモジュール基板の設定
	    // 接続先のMCMDの設定
	    mcmd4_struct.device.node_type = NODE_MCMD3;  // nodeのタイプ (NODE_MCMD3など)
	    mcmd4_struct.device.node_id = 1;  // 基板の番号 (基板上の半固定抵抗を回す事で設定できる)
	    mcmd4_struct.device.device_num = 0;  // モーターの番号(MCMDなら0と1の2つが選べる)

	    // 制御パラメータの設定
	    mcmd4_struct.ctrl_param.ctrl_type = MCMD_CTRL_POS;  //制御タイプを設定
	    mcmd4_struct.ctrl_param.PID_param.kp = 0.3f;  // Pゲイン 1.0
	    mcmd4_struct.ctrl_param.PID_param.ki = 0.0f;  // Iゲイン 0.0
	    mcmd4_struct.ctrl_param.PID_param.kd = 0.0f;  // Dゲイン 0.0 (Dゲインは使いにくい)
	    mcmd4_struct.ctrl_param.accel_limit = ACCEL_LIMIT_ENABLE;  // PIDの偏差をclipするか
	    mcmd4_struct.ctrl_param.accel_limit_size = 2.0f;  // PIDの偏差をclipする場合の絶対値のmax値
	    mcmd4_struct.ctrl_param.feedback = MCMD_FB_ENABLE;  // MCMDからF7にフィードバックを送信するか否か
	    mcmd4_struct.ctrl_param.timup_monitor = TIMUP_MONITOR_DISABLE;  // timeupは未実装なのでDISABLE。
	    mcmd4_struct.enc_dir = MCMD_DIR_FW;  // Encoderの回転方向設定
	    mcmd4_struct.rot_dir = MCMD_DIR_BC;  // モーターの回転方向設定
	    mcmd4_struct.quant_per_unit = 59.0/6400.0f;  // エンコーダーの分解能に対する制御値の変化量の割合

	    // 原点サーチの設定
	    mcmd4_struct.limit_sw_type = LIMIT_SW_NC;  // 原点サーチにNomaly Closedのスイッチを用いる
	    mcmd4_struct.calib = CALIBRATION_DISABLE;  // 原点サーチを行う。
	    mcmd4_struct.calib_duty = 0.1f;  // 原点サーチ時のduty
	    mcmd4_struct.offset = 0.0f;  // 原点のオフセット
	    mcmd4_struct.fb_type = MCMD_FB_POS;  // 読み取った位置情報をF7にフィードバックする。
}

void activateMcmdControll(){
	// パラメータなどの設定と動作命令をMCMDに送信する
	 MCMD_init(&mcmd4_struct);
	 HAL_Delay(10);
	 MCMD_Calib(&mcmd4_struct);  // キャリブレーションを行う
	 HAL_Delay(2000);  // キャリブレーションが終わるまで待つ
	 MCMD_SetTarget(&mcmd4_struct, 30.0f);  // 目標値(0.0)を設定
	 HAL_Delay(10);
	 MCMD_Control_Enable(&mcmd4_struct);  // 制御開始
	 printf("start");
	 HAL_Delay(10);
}

void servoSetting(){
	// Servo基板のdevice設定
	servo_device.node_type = NODE_SERVO;
	servo_device.node_id = 2;
	servo_device.device_num = 0;//0~3を指定する

	// Servo基板のパラメータ (offset以外はあまり変更しない)
	servo_param.angle_range=270.0f;//サーボの動作範囲
	servo_param.angle_offset=0.0f;//原点の位置
	servo_param.pulse_width_max=2.4f;//サーボの制御のPWM信号のパルス幅の最大値
	servo_param.pulse_width_min=0.5f;//サーボの制御のPWM信号のパルス幅の最小値
	servo_param.pwm_frequency=50;//PWM周波数（この変更は未実装
}

void airSetting(){
	air_device.node_type = NODE_AIR; //エアシリンダ基盤であることを示す
	air_device.node_id = 0; //基板の番号
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
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */

  //記事ではmcmdなどの初期化コードを描くことになっている場所
  canSetting();
  mcmdSetting();
  activateMcmdControll();
  servoSetting();
  airSetting();


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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of systemCheckTask */
  systemCheckTaskHandle = osThreadNew(StartSystemCheckTask, NULL, &systemCheckTask_attributes);

  /* creation of ControllerTask */
  ControllerTaskHandle = osThreadNew(StartControllerTask, NULL, &ControllerTask_attributes);

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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN 5 */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_RESET);  // LED1 消灯
  /* Infinite loop */
  for(;;)
  {
	uint16_t button_data = UDPController_GetControllerButtons();  // buttonの入力を取得
	if((button_data & CONTROLLER_CIRCLE) != 0){  // oボタンが押されている場合
	   HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_SET);  // LED1 点灯
	}else{
	   HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_RESET);  // LED1 消灯
	}
	osDelay(100);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartSystemCheckTask */
/**
* @brief Function implementing the systemCheckTask thread.
* @param argument: Not used
* @retval None
*/
void freeRTOSChecker(){//無限ループの中で実行
	HAL_GPIO_TogglePin(GPIOB, LD2_Pin);  // PINのPin stateを反転
}

void mcmdChecker(){//無限ループの中で実行
	mcmd_fb = Get_MCMD_Feedback(&(mcmd4_struct.device));
	printf("value of tyokudou %d\r\n",(int)(mcmd_fb.value));
}

void servoChecker(){
	ServoDriver_Init(&servo_device, &servo_param);  // Servo基板にパラメータを送信
	HAL_Delay(100);  // 適切なdelayを入れる
	ServoDriver_SendValue(&servo_device, 20.0f);  // サーボが20.0度になるように回転させる
}

void airChecker(){
	for(uint8_t i=PORT_1; i<=PORT_8; i++){  //すべてのポートを初期化しないとAir基板は動かない
	    air_device.device_num = i; // (i番ポートを指定)
	    AirCylinder_Init(&air_device, AIR_OFF);
	    HAL_Delay(10);  // このdelayは必要
	  }
	  air_device.device_num=0; // とりあえず0番ポートのエアシリンダを動かします。
	  AirCylinder_SendOutput(&air_device, AIR_ON);  // 0番ポートの電磁弁がonになる
	  HAL_Delay(1000);
	  AirCylinder_SendOutput(&air_device, AIR_OFF); // 0番ポートの電磁弁がoffになる
	  HAL_Delay(1000);
}
/* USER CODE END Header_StartSystemCheckTask */
void StartSystemCheckTask(void *argument)
{
  /* USER CODE BEGIN StartSystemCheckTask */
	servoChecker();
	airChecker();
  /* Infinite loop */
  for(;;)
  {
	  freeRTOSChecker();
	  mcmdChecker();
	  osDelay(1000);
  }
  /* USER CODE END StartSystemCheckTask */
}

/* USER CODE BEGIN Header_StartControllerTask */
/**
* @brief Function implementing the ControllerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartControllerTask */
void StartControllerTask(void *argument)
{
  /* USER CODE BEGIN StartControllerTask */
  /* Infinite loop */
	UDPControllerReceive(argument);
  /* USER CODE END StartControllerTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14) {
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

#ifdef  USE_FULL_ASSERT
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
