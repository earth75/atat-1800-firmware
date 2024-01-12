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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_hid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
	uint8_t MODIFIER;
	uint8_t RESERVED;
	uint8_t KEYCODE1;
	uint8_t KEYCODE2;
	uint8_t KEYCODE3;
	uint8_t KEYCODE4;
	uint8_t KEYCODE5;
	uint8_t KEYCODE6;
}KeyBoardReport_t;

typedef struct
{
	GPIO_TypeDef* Port;
	uint16_t Pin;
}GPIO_t;

//  +------+-------+
//  |          16
//  +------+-------+
//  |
//  +------+-------+
typedef union{
	unsigned int keyState;
	struct{
	   // LSB
	   unsigned int brightness:16;
	   unsigned int state:8;
	   // MSB
	};

} keystate_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ROWS 6
#define COLS 19



const GPIO_t cols[COLS] = {
		{COL0_GPIO_Port,  COL0_Pin},
		{COL1_GPIO_Port,  COL1_Pin},
		{COL2_GPIO_Port,  COL2_Pin},
		{COL3_GPIO_Port,  COL3_Pin},
		{COL4_GPIO_Port,  COL4_Pin},
		{COL5_GPIO_Port,  COL5_Pin},
		{COL6_GPIO_Port,  COL6_Pin},
		{COL7_GPIO_Port,  COL7_Pin},
		{COL8_GPIO_Port,  COL8_Pin},
		{COL9_GPIO_Port,  COL9_Pin},
		{COL10_GPIO_Port, COL10_Pin},
		{COL11_GPIO_Port, COL11_Pin},
		{COL12_GPIO_Port, COL12_Pin},
		{COL13_GPIO_Port, COL13_Pin},
		{COL14_GPIO_Port, COL14_Pin},
		{COL15_GPIO_Port, COL15_Pin},
		{COL16_GPIO_Port, COL16_Pin},
		{COL17_GPIO_Port, COL17_Pin},
		{COL18_GPIO_Port, COL18_Pin},
	};

const GPIO_t rows[ROWS] = {
		{ROW0_GPIO_Port, ROW0_Pin},
		{ROW1_GPIO_Port, ROW1_Pin},
		{ROW2_GPIO_Port, ROW2_Pin},
		{ROW3_GPIO_Port, ROW3_Pin},
		{ROW4_GPIO_Port, ROW4_Pin},
		{ROW5_GPIO_Port, ROW5_Pin},
	};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

/* USER CODE BEGIN PV */

extern USBD_HandleTypeDef hUsbDeviceFS;
KeyBoardReport_t keyBoardHIDsub = {0,0,0,0,0,0,0,0};

keystate_t Keyboard[ROWS][COLS];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


//initialize backlight drivers
void initBL(void){
  uint8_t data;
  uint8_t ledState[3] = {0x0F, 0xFF, 0xFF};

  //chip addresses are in the range 0x30 0x39
  for (uint16_t i = 0x30; i<=0x39; i++){

	  //enable chip FRFSH=0
	  data = 0x00;
	  if(HAL_I2C_Mem_Write (&hi2c1, i << 1, 0x02, I2C_MEMADD_SIZE_8BIT, &data, 1, 100) == HAL_ERROR) continue;

	  //set current to half for all leds
	  for (uint16_t j = 0x0A; j<0x40; j+=3){
		HAL_I2C_Mem_Write (&hi2c1, i << 1, j, I2C_MEMADD_SIZE_8BIT, ledState, 3, 100);
	  }

	  //enable chip FRFSH=0 FLTEN=1 STH=3 LATCH=0
	  data = 0xBD;
	  HAL_I2C_Mem_Write (&hi2c1, i << 1, 0x01, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
	  //activate channel 1
	  data = 0x01;
	  HAL_I2C_Mem_Write (&hi2c1, i << 1, 0x05, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

  }
}

//blink all led1 once
void blink_BL(int times){
 uint8_t ledState[3] = {0x0F, 0xFF, 0xFF};

 for (int i=0; i< times; i++){
	  //ON
	  ledState[0] = 0x0F;
	  for (uint16_t i = 0x30; i<=0x39; i++){
		  HAL_I2C_Mem_Write (&hi2c1, i << 1, 0x0A, I2C_MEMADD_SIZE_8BIT, ledState, 3, 100);
	  }
	  HAL_Delay(150);

	  //OFF
	  ledState[0] = 0x00;
	  for (uint16_t i = 0x30; i<=0x39; i++){
		  HAL_I2C_Mem_Write (&hi2c1, i << 1, 0x0A, I2C_MEMADD_SIZE_8BIT, ledState, 3, 100);
	  }
	  HAL_Delay(100);
 }
 HAL_Delay(200);

}

//refresh backlight levels and update drivers
void refreshBL(void){

}


//update key status
void refreshKeys(void){
	for (int row=0; row < ROWS; row++){
		HAL_GPIO_WritePin(rows[row].Port, rows[row].Pin, GPIO_PIN_RESET); // set this pin  to open drain
		HAL_Delay(1); //settle time (might not be needed)
		for (int col=0; col < COLS; col++) 	Keyboard[row][col].state = HAL_GPIO_ReadPin(cols[col].Port, cols[col].Pin);
		HAL_GPIO_WritePin(rows[row].Port, rows[row].Pin, GPIO_PIN_SET); // set it back to hi z
	}
}


//send HID report to Host
void sendReport(void){

	  keyBoardHIDsub.MODIFIER=0x02;  // To press shift key<br>keyBoardHIDsub.KEYCODE1=0x04;  // Press A key
	  keyBoardHIDsub.KEYCODE2=0x05;  // Press B key
	  keyBoardHIDsub.KEYCODE3=0x06;  // Press C key
	  USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t *)&keyBoardHIDsub,sizeof(keyBoardHIDsub));
	  HAL_Delay(50); 		       // Press all key for 50 milliseconds
	  keyBoardHIDsub.MODIFIER=0x00;  // To release shift key
	  keyBoardHIDsub.KEYCODE2=0x00;  // Release B key
	  keyBoardHIDsub.KEYCODE3=0x00;  // Release C key
	  USBD_HID_SendReport(&hUsbDeviceFS,(uint8_t *)&keyBoardHIDsub,sizeof(keyBoardHIDsub));
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
  MX_I2C1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  //init all led drivers


  HAL_Delay(20);
  initBL();






  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  refreshKeys();


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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM14_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM14_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM14_IRQn);
  /* TIM16_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM16_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM16_IRQn);
  /* TIM17_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM17_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM17_IRQn);
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
  hi2c1.Init.Timing = 0x0010020A;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 0;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 0;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 65535;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ROW0_Pin|ROW1_Pin|ROW2_Pin|ROW4_Pin
                          |ROW5_Pin|ROW3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : COL18_Pin COL17_Pin COL16_Pin */
  GPIO_InitStruct.Pin = COL18_Pin|COL17_Pin|COL16_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : COL15_Pin */
  GPIO_InitStruct.Pin = COL15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(COL15_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ROW0_Pin ROW1_Pin ROW2_Pin ROW4_Pin
                           ROW5_Pin ROW3_Pin */
  GPIO_InitStruct.Pin = ROW0_Pin|ROW1_Pin|ROW2_Pin|ROW4_Pin
                          |ROW5_Pin|ROW3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : FAULT_Pin */
  GPIO_InitStruct.Pin = FAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(FAULT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : COL7_Pin COL6_Pin COL5_Pin COL4_Pin
                           COL9_Pin COL8_Pin COL3_Pin COL2_Pin
                           COL13_Pin COL12_Pin COL11_Pin COL10_Pin
                           COL14_Pin */
  GPIO_InitStruct.Pin = COL7_Pin|COL6_Pin|COL5_Pin|COL4_Pin
                          |COL9_Pin|COL8_Pin|COL3_Pin|COL2_Pin
                          |COL13_Pin|COL12_Pin|COL11_Pin|COL10_Pin
                          |COL14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : COL1_Pin COL0_Pin */
  GPIO_InitStruct.Pin = COL1_Pin|COL0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PWR_Pin */
  GPIO_InitStruct.Pin = USB_PWR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_PWR_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
