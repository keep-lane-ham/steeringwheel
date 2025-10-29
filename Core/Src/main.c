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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define AS5600_SLAVE_ADDR   0x36
#define AS5600_REG_ANGLE    0x0E
#define CAN_ANGLE_ID        0x100
#define TCA9548A_ADDR 0x70
#define DRV2605L_ADDR 0x5A
#define DRV2605L_REG_STATUS 0x00
#define DRV2605L_REG_MODE 0x01
#define DRV2605L_REG_RTP_INPUT 0x02
#define DRV2605L_REG_LIBRARY_SEL 0x03
#define DRV2605L_REG_WAVEFORM_SEQ_1 0x04
#define DRV2605L_REG_WAVEFORM_SEQ_2 0x05
#define DRV2605L_REG_GO 0x0C
#define DRV2605L_REG_RATED_VOLTAGE 0x16
#define DRV2605L_REG_OD_CLAMP 0x17
#define DRV2605L_REG_FEEDBACK_CONTROL 0x1A
#define DRV2605L_REG_CONTROL1 0x1B
#define DRV2605L_REG_CONTROL3 0x1D
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
CAN_HandleTypeDef hcan;
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;

CAN_TxHeaderTypeDef   TxHeader;
uint8_t               TxData[8];
uint32_t              TxMailbox;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN_Init(void);
static void MX_I2C1_Init(void);

uint16_t as5600_read_angle(void);
HAL_StatusTypeDef select_mux_channel(uint8_t channel);
HAL_StatusTypeDef drv2605l_write_reg(uint8_t channel, uint8_t reg, uint8_t value);
HAL_StatusTypeDef drv2605l_read_reg(uint8_t channel, uint8_t reg, uint8_t *value);
HAL_StatusTypeDef drv2605l_init(uint8_t channel);
HAL_StatusTypeDef drv2605l_play_effect(uint8_t channel, uint8_t effect_id);

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
	if (ch == "\n")
		HAL_UART_Transmit (&huart2, (uint8_t*) "\r", 1, 0xFFFF);
	HAL_UART_Transmit (&huart2, (uint8_t*) &ch, 1, 0xFFFF);

	return ch;
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint8_t b1_pressed_flag = 0;
volatile uint8_t b2_pressed_flag = 0;
uint32_t last_b1_tick = 0;
uint32_t last_b2_tick = 0;
#define DEBOUNCE_TIME_MS 100
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

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
	char uart_buf[100];
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
  MX_CAN_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  if (HAL_I2C_IsDeviceReady(&hi2c1, (AS5600_SLAVE_ADDR << 1), 2, 100) != HAL_OK)
    {
      printf("AS5600 not found!\r\n");
      Error_Handler();
    }
    else
    {
      printf("AS5600 found.\r\n");
    }

    /* --- CAN 통신 시작 --- */
    if (HAL_CAN_Start(&hcan) != HAL_OK)
    {
      printf("CAN Start Failed!\r\n");
      Error_Handler();
    }
    printf("CAN Started Successfully.\r\n");

    TxHeader.StdId = CAN_ANGLE_ID;
    TxHeader.ExtId = 0;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 2;
    TxHeader.TransmitGlobalTime = DISABLE;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		  if (b1_pressed_flag == 1) {
			  printf("B1 Pin enabled - Sending CAN 0x101\r\n");
			  TxHeader.StdId = 0x101; TxHeader.DLC = 1; TxData[0] = 0x01;
			  if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK){ printf("CAN Tx (B1) FAILED!\r\n"); }
			  b1_pressed_flag = 0;
		  }
		  if (b2_pressed_flag == 1) {
			  printf("B2 Pin enabled - Sending CAN 0x101\r\n");
			  TxHeader.StdId = 0x101; TxHeader.DLC = 1; TxData[0] = 0x02;
			  if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK){ printf("CAN Tx (B2) FAILED!\r\n"); }
			  b2_pressed_flag = 0;
		  }

	      uint16_t raw_angle = (as5600_read_angle() + 1282) % 4096; // angle offset
	      TxData[0] = (uint8_t)(raw_angle & 0xFF);
	      TxData[1] = (uint8_t)((raw_angle >> 8) & 0xFF);
	      TxHeader.StdId = 0x100; TxHeader.DLC = 2;
	      if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK) { printf("CAN Tx FAILED!\r\n"); }
	      printf("Transmitting Angle: %u degrees\r\n", raw_angle);
	      HAL_Delay(50);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 8;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_14TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_5TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  CAN_FilterTypeDef sFilterConfig;

    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
    {
      /* Filter configuration Error */
      printf("!!! HAL_CAN_ConfigFilter FAILED !!!\r\n");
      Error_Handler();
    }
  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : B1_Pin B2_Pin */
  GPIO_InitStruct.Pin = B1_Pin|B2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
uint16_t as5600_read_angle(void)
{
  uint8_t i2c_buf[2];
  uint16_t angle_val = 0;

  if (HAL_I2C_Mem_Read(&hi2c1, (AS5600_SLAVE_ADDR << 1), AS5600_REG_ANGLE, I2C_MEMADD_SIZE_8BIT, i2c_buf, 2, 100) == HAL_OK)
  {
    angle_val = (i2c_buf[0] << 8) | i2c_buf[1];
  }
  else
  {
    printf("I2C Read Error! (AS5600)\r\n");
  }

  return angle_val;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
   uint32_t current_tick = HAL_GetTick();

   switch(GPIO_Pin) {
       case (B1_Pin):
          if (current_tick - last_b1_tick > DEBOUNCE_TIME_MS) {
              b1_pressed_flag = 1;
              last_b1_tick = current_tick;
          }
          break;
       case (B2_Pin):
          if (current_tick - last_b2_tick > DEBOUNCE_TIME_MS) {
              b2_pressed_flag = 1;
              last_b2_tick = current_tick;
          }
          break;
       default:
          break;
   }
}


/**
  * @brief 특정 채널의 DRV2605L 레지스터에 값을 씁니다.
  * @param channel: 대상 모터 드라이버의 MUX 채널 (0-7)
  * @param reg: 쓸 레지스터 주소
  * @param value: 쓸 값
  * @retval HAL status
  */
HAL_StatusTypeDef drv2605l_write_reg(uint8_t channel, uint8_t reg, uint8_t value)
{
  HAL_StatusTypeDef status;
  HAL_StatusTypeDef mux_deselect_status; // MUX 비활성화 상태 저장용

  // 1. MUX 채널 선택
  status = select_mux_channel(channel);
  if (status != HAL_OK) {
      select_mux_channel(0xFF); // 오류 시에도 비활성화 시도
      return status;
  }

  status = HAL_I2C_Mem_Write(&hi2c1, (DRV2605L_ADDR << 1), reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
  mux_deselect_status = select_mux_channel(0xFF); // 0xFF 같은 유효하지 않은 채널 번호로 비활성화

  return status;
}

/**
  * @brief 특정 채널의 DRV2605L 레지스터 값을 읽습니다. (MUX Deselect 추가됨)
  * @param channel: 대상 모터 드라이버의 MUX 채널 (0-7)
  * @param reg: 읽을 레지스터 주소
  * @param value: 읽은 값을 저장할 포인터
  * @retval HAL status
  */
HAL_StatusTypeDef drv2605l_read_reg(uint8_t channel, uint8_t reg, uint8_t *value)
{
  HAL_StatusTypeDef status;
  HAL_StatusTypeDef mux_deselect_status;

  status = select_mux_channel(channel);
  if (status != HAL_OK) {
      select_mux_channel(0xFF);
      return status;
  }

  status = HAL_I2C_Mem_Read(&hi2c1, (DRV2605L_ADDR << 1), reg, I2C_MEMADD_SIZE_8BIT, value, 1, 100);

  mux_deselect_status = select_mux_channel(0xFF);

  return status;
}


/**
  * @brief [수정됨] 특정 채널의 DRV2605L을 LRA 폐쇄 루프 모드로 초기화/보정합니다.
  * @brief (VLV101040A 170Hz LRA @ 5.0V V_BAT 기준)
  * @param channel: 초기화할 모터 드라이버의 MUX 채널 (0-7)
  * @retval HAL status
  */
HAL_StatusTypeDef drv2605l_init(uint8_t channel)
{
  HAL_StatusTypeDef status;
  uint8_t temp_reg_val;
  char dbg_buf[80];
  status = drv2605l_read_reg(channel, DRV2605L_REG_STATUS, &temp_reg_val);
  if (status != HAL_OK) {
      snprintf(dbg_buf, sizeof(dbg_buf), "Ch %d Init: Failed read Status\r\n", channel);
      printf(dbg_buf);
      return status;
  }
  if ((temp_reg_val >> 5) != 0x07) { // Device ID for DRV2605L should be 7
      snprintf(dbg_buf, sizeof(dbg_buf), "Ch %d Init: Wrong Device ID: 0x%02X\r\n", channel, temp_reg_val);
      printf(dbg_buf);
      return HAL_ERROR;
  }

  status = drv2605l_write_reg(channel, DRV2605L_REG_MODE, 0x00); // Mode = 0, Standby = 0
  if (status != HAL_OK) { printf("Ch %d Init: Failed exit Standby\r\n", channel); return status; }
  HAL_Delay(5);
  // --- Configure for LRA Closed Loop ---
  status = drv2605l_read_reg(channel, DRV2605L_REG_FEEDBACK_CONTROL, &temp_reg_val);
  if (status != HAL_OK) { printf("Ch %d Init: Failed read FB_CTRL\r\n", channel); return status; }
  temp_reg_val |= (1 << 7);
  status = drv2605l_write_reg(channel, DRV2605L_REG_FEEDBACK_CONTROL, temp_reg_val);
  if (status != HAL_OK) { printf("Ch %d Init: Failed write FB_CTRL\r\n", channel); return status; }

  status = drv2605l_read_reg(channel, DRV2605L_REG_CONTROL3, &temp_reg_val);
  if (status != HAL_OK) { printf("Ch %d Init: Failed read CTRL3\r\n", channel); return status; }
  temp_reg_val &= ~(1 << 0);
  status = drv2605l_write_reg(channel, DRV2605L_REG_CONTROL3, temp_reg_val);
  if (status != HAL_OK) { printf("Ch %d Init: Failed write CTRL3\r\n", channel); return status; }
  status = drv2605l_write_reg(channel, DRV2605L_REG_LIBRARY_SEL, 6);
  if (status != HAL_OK) { printf("Ch %d Init: Failed set Library\r\n", channel); return status; }

  // --- Prepare for Auto-Calibration (LRA 값 수정됨) ---
  uint8_t drive_time_val = 13;
  status = drv2605l_read_reg(channel, DRV2605L_REG_CONTROL1, &temp_reg_val);
  if (status != HAL_OK) { printf("Ch %d Cal: Failed read CTRL1\r\n", channel); return status; }
  temp_reg_val = (temp_reg_val & 0xE0) | (drive_time_val & 0x1F);
  status = drv2605l_write_reg(channel, DRV2605L_REG_CONTROL1, temp_reg_val);
  if (status != HAL_OK) { printf("Ch %d Cal: Failed set Drive Time\r\n", channel); return status; }

  uint8_t rated_voltage_reg_val = 144;
  uint8_t od_clamp_reg_val = 137;

  status = drv2605l_write_reg(channel, DRV2605L_REG_RATED_VOLTAGE, rated_voltage_reg_val);
  if (status != HAL_OK) { printf("Ch %d Cal: Failed set Rated V\r\n", channel); return status; }
  status = drv2605l_write_reg(channel, DRV2605L_REG_OD_CLAMP, od_clamp_reg_val);
  if (status != HAL_OK) { printf("Ch %d Cal: Failed set OD Clamp\r\n", channel); return status; }

  snprintf(dbg_buf, sizeof(dbg_buf), "--> Ch %d: Set LRA, Lib6, DrvT=%d, RateV=0x%02X, ODClp=0x%02X\r\n",
           channel, drive_time_val, rated_voltage_reg_val, od_clamp_reg_val);
  printf(dbg_buf);

  // --- Run Auto-Calibration ---
  status = drv2605l_write_reg(channel, DRV2605L_REG_MODE, 0x07); // Mode = 7
  if (status != HAL_OK) { printf("Ch %d Cal: Failed set Cal Mode\r\n", channel); return status; }
  status = drv2605l_write_reg(channel, DRV2605L_REG_GO, 1); // Set GO bit
  if (status != HAL_OK) { printf("Ch %d Cal: Failed set GO bit\r\n", channel); return status; }
  HAL_Delay(1500);
  uint8_t cal_status_reg;
  status = drv2605l_read_reg(channel, DRV2605L_REG_STATUS, &cal_status_reg);
  if (status != HAL_OK) {
       printf("Ch %d Cal: Failed to read Status after cal\r\n", channel);
       return status;
  } else {
      if (cal_status_reg & (1 << 3)) { 
          printf("--> Ch %d: Auto-Calibration FAILED! (DIAG_RESULT=1)\r\n", channel);
          return HAL_ERROR;
      } else {
          printf("--> Ch %d: Auto-Calibration OK.\r\n", channel);
      }
  }
  status = drv2605l_write_reg(channel, DRV2605L_REG_MODE, 0x00); // 0x00 = Internal Trigger mode
  if (status != HAL_OK) { printf("Ch %d Init: Failed set Mode final (Internal Trigger)\r\n", channel); return status; }
  uint8_t mode_read=0;
  drv2605l_read_reg(channel, DRV2605L_REG_MODE, &mode_read);
  printf("--> Ch %d Final Check: Mode=0x%02X\r\n", channel, mode_read);

  return HAL_OK;
}

/**
  * @brief 특정 채널의 DRV2605L에서 지정된 라이브러리 효과를 재생합니다.
  * @param channel: 재생할 모터 드라이버의 MUX 채널 (0-7)
  * @param effect_id: 재생할 효과 ID (1-123)
  * @retval HAL status
  */
HAL_StatusTypeDef drv2605l_play_effect(uint8_t channel, uint8_t effect_id)
{
  HAL_StatusTypeDef status;
  uint8_t go_status;

  if (effect_id == 0 || effect_id > 123) return HAL_ERROR;
  status = drv2605l_write_reg(channel, DRV2605L_REG_WAVEFORM_SEQ_1, effect_id);
  if (status != HAL_OK) return status;
  status = drv2605l_write_reg(channel, DRV2605L_REG_WAVEFORM_SEQ_2, 0x00);
  if (status != HAL_OK) return status;
  status = drv2605l_write_reg(channel, DRV2605L_REG_GO, 0x01);
  return status;
}


/**
  * @brief  CAN RX FIFO 0에 메시지가 수신되면 호출되는 콜백 함수
  * @brief  [수정됨] 수신된 데이터에 따라 16번 이펙트를 재생합니다.
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef   RxHeader;
    uint8_t               RxData[8];
    char uart_buf[100];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
    {
        return;
    }

    // ID 0x102 = "모터 8개 이펙트 트리거" 명령
    if (RxHeader.StdId == 0x102)
    {
        // 8바이트 데이터(DLC=8)가 왔는지 확인
        if (RxHeader.DLC == 8)
        {
            snprintf(uart_buf, sizeof(uart_buf), ">>> CAN RX [Effect Trigger]: %u %u %u %u %u %u %u %u\r\n",
                     RxData[0], RxData[1], RxData[2], RxData[3],
                     RxData[4], RxData[5], RxData[6], RxData[7]);
            printf("%s", uart_buf);

            // 8개 모터에 대해 이펙트 재생
            for (int i = 0; i < 8; i++)
            {
                // RxData[i] 값이 0이 아니면, 해당 채널(i)에 16번 이펙트를 재생합니다.
                if (RxData[i] != 0)
                {
                    drv2605l_play_effect(i, 16);
                }
            }
        }
    }
  }

HAL_StatusTypeDef read_mux_control_register(uint8_t *control_reg_value) {
    return HAL_I2C_Master_Receive(&hi2c1, (TCA9548A_ADDR << 1), control_reg_value, 1, 100);
}

/**
  * @brief TCA9548A MUX의 특정 채널을 선택하거나 모두 비활성화합니다.
  * @param channel: 선택할 채널 번호 (0-7). 0xFF 또는 다른 값 입력 시 모든 채널 비활성화.
  * @retval HAL status
  */
HAL_StatusTypeDef select_mux_channel(uint8_t channel)
{
  uint8_t mux_cmd = 0; // 기본값: 모든 채널 비활성화
  if (channel < 8)
  {
    mux_cmd = 1 << channel; // 해당 채널 비트만 1로 설정
  }
  HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1, (TCA9548A_ADDR << 1), &mux_cmd, 1, 100);
  return status;
}


/* USER CODE END 4 */

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
