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
#include "mpq3326.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define ROLLOVER 6
typedef struct
{
	uint8_t MODIFIER;
	uint8_t RESERVED;
	uint8_t KEYCODE[ROLLOVER];
}KeyBoardReport_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
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

const backlight_t Backlight_map[ROWS][COLS] = {
//COL0				1			2			3			4			5			6			7			8			9			10			11			12			13				14			15					16							17					18
//ROW 0
{{1, 6},			{0,1},		{0,1},		{0,1},		{0,1},		{0,1},		{0,1},		{0,1},		{0,1},		{0,1},		{0,1},		{0,1},		{0,1},		{0,1},			{0,1},		{0,1},				{0,1},						{0,1},				{0,1}		},
//ROW 1
{{0, 6},			{0,1},		{0,1},		{0,1},		{0,1},		{0,1},		{0,1},		{0,1},		{0,1},		{0,1},		{0,1},		{0,1},		{0,1},		{0,1},			{0,1},		{0,1},				{0,1},						{0,1},				{0,1}		},
//ROW 2
{{0, 6},			{0,1},		{0,1},		{0,1},		{0,1},		{0,1},		{0,1},		{0,1},		{0,1},		{0,1},		{0,1},		{0,1},		{0,1},		{0,1},			{0,1},		{0,1},				{0,1},						{0,1},				{0,1}		},
//ROW 3
{{0, 6},			{0,1},		{0,1},		{0,1},		{0,1},		{0,1},		{0,1},		{0,1},		{0,1},		{0,1},		{0,1},		{0,1},		{0,1},		{0,1},			{0,1},		{0,1},				{0,1},						{0,1},				{0,1}		},
//ROW 4
{{0, 6},			{0,1},		{0,1},		{0,1},		{0,1},		{0,1},		{0,1},		{0,1},		{0,1},		{0,1},		{0,1},		{0,1},		{0,1},		{0,1},			{0,1},		{0,1},				{0,1},						{0,1},				{0,1}		},
//ROW 5
{{0, 6},			{0,1},		{0,1},		{0,1},		{0,1},		{0,1},		{0,1},		{0,1},		{0,1},		{0,1},		{0,1},		{0,1},		{0,1},		{0,1},			{0,1},		{0,1},				{0,1},						{0,1},				{0,1}		},

};

const char Keycode_map[ROWS][COLS] = {
//COL0				1			2			3			4			5			6			7			8			9			10			11			12			13				14			15					16							17					18
//ROW 0
{Key_ESCAPE, 		0, 			Key_F1, 	Key_F2, 	Key_F3, 	Key_F4, 	Key_F5, 	Key_F6, 	Key_F7, 	Key_F8, 	Key_F9, 	Key_F10, 	Key_F11, 	Key_F12, 		Key_DELETE, Key_INSERT, 		0, 							0, 					0},
//ROW 1
{Key_GRAVE_ACCENT, 	Key_ONE, 	Key_TWO, 	Key_THREE, 	Key_FOUR, 	Key_FIVE, 	Key_SIX, 	Key_SEVEN, 	Key_EIGHT, 	Key_NINE, 	Key_ZERO, 	Key_MINUS, 	Key_EQUALS, Key_BACKSPACE, 	Key_MACRO, 	Key_KEYPAD_NUMLOCK, Key_KEYPAD_FORWARD_SLASH, 	Key_KEYPAD_ASTERISK, Key_KEYPAD_MINUS},
//ROW 2
{Key_TAB, 			Key_Q, 		Key_W, 		Key_E, 		Key_R, 		Key_T, 		Key_Y, 		Key_U, 		Key_I, 		Key_O, 		Key_P, 	Key_L_BRACKET, Key_R_BRACKET, Key_BACKSLASH, Key_MACRO, Key_KEYPAD_SEVEN, Key_KEYPAD_EIGHT, 		Key_KEYPAD_NINE, 	Key_KEYPAD_PLUS},
//ROW 3
{Key_CAPS_LOCK, 	Key_A,		Key_S,		Key_D,		Key_F,		Key_G,		Key_H,		Key_J,		Key_K,		Key_L,		Key_SEMICOLON, Key_QUOTE, Key_ENTER, 0, 			0, 			Key_KEYPAD_FOUR, 	Key_KEYPAD_FIVE, 			Key_KEYPAD_SIX, 	0},
//ROW 4
{Key_L_SHIFT,		Key_Z,		Key_X,		Key_C,		Key_V,		Key_B,		Key_N,		Key_M,		Key_COMMA,	Key_PERIOD,	Key_FORWARD_SLASH,Key_R_SHIFT, 0, 	0, 			Key_UP_ARROW, 	Key_KEYPAD_ONE, 	Key_KEYPAD_TWO, 			Key_KEYPAD_THREE,	Key_KEYPAD_ENTER},
//ROW 5
{Key_L_CTL,			Key_WIN,	Key_L_ALT,	0, 			0, 			Key_SPACEBAR, 0, 		0,			0,			Key_R_ALT,	Key_MACRO,	Key_R_CTL,	0,			Key_L_ARROW,	Key_DOWN_ARROW, Key_R_ARROW,	Key_KEYPAD_ZERO,			Key_KEYPAD_PERIOD,	0}
};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim14;

/* USER CODE BEGIN PV */

extern USBD_HandleTypeDef hUsbDeviceFS;
static KeyBoardReport_t keyBoardHIDsub = {0,0,{0,0,0,0,0,0}};
volatile uint8_t keyBoardLEDState = 0x0;
volatile keystate_t Keyboard[ROWS][COLS] = {};
static mpq3326_t ledDriver[10];
int driver_present[10]; //some drivers are not used
volatile int KEY_CHANGE = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//initialize backlight drivers
void initBL(void){
  //Parse the keyboard descriptor to enable the necessary channels on the drivers


  //chip addresses are in the range 0x30 0x39
  for (uint16_t i = 0; i<10; i++){

	//set FRFSH=0 and see if the chip ACKs to test if present
	ledDriver[i].refresh = 0x0000;
	if(HAL_I2C_Mem_Write (&hi2c1, (0x30 + i) << 1, 0x02, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&(ledDriver[i].refresh), 1, 100) == HAL_ERROR) {
		driver_present[i] = 0; //this chip does not exist
		continue; //try the next one
	}

	driver_present[i] = 1; //communication with the driver is OK

	//set f_pwm to 250Hz (not used anyway)
	ledDriver[i].fpwm = 0x01;

	//enable chip FLTEN=1 STH=3 LATCH=0
	ledDriver[i].control = 0xBD;

	//TODO dynamic enable
	ledDriver[i].enable = 0xFFFF;

	//set current to half for all leds and duty cycle to 100%
	for (uint16_t j = 0; j<16; j++){
		ledDriver[i].channels[j].current  = 0x02;
		ledDriver[i].channels[j].duty_msb = 0xFF;
		ledDriver[i].channels[j].duty_lsb = 0xFF;
	}

	 //apply all the settings
	for(int i = 0; i < 10; i++){
		HAL_I2C_Mem_Write (&hi2c1, (0x30 + i) << 1, 0x00, I2C_MEMADD_SIZE_8BIT, (uint8_t *)(&ledDriver[i]), sizeof(mpq3326_t), 100);
	}

  }
}

//refresh backlight levels and update drivers
void refreshBL(void){
	for(int i = 0; i < 10; i++){
		if(driver_present[i]) HAL_I2C_Mem_Write (&hi2c1, (0x30 + i) << 1, 0x0A, I2C_MEMADD_SIZE_8BIT, (uint8_t *)(&(ledDriver[i].channels)), 16*sizeof(mpq3326_channel_t), 100);
	}
}

//blink all led1 once
void blink_BL(int times){

 for (int j=0; j<times; j++){
	  //ON
	  for (uint16_t i = 0; i<10; i++){
		  ledDriver[i].channels[1].current = 0x0F; //led 1 of each driver ON
	  }
	  refreshBL();
	  HAL_Delay(150);

	  //OFF
	  for (uint16_t i = 0; i<10; i++){
		  ledDriver[i].channels[1].current = 0x00; //led 1 of each driver OFF
	  }
	  refreshBL();
	  HAL_Delay(150);
 }
 HAL_Delay(200);

}


//update HID report content
//return 0 if OK
//return 1 if NOK (unknown keycode or more than 6 keys already present)
//return 2 if WTF
int updateReport(int keycode, int pressed){
	uint16_t i = 0;
	if(keycode&0x80){ //is this a modifier key?
		if(pressed) keyBoardHIDsub.MODIFIER |= 1<<(keycode&0x0F); //if pressed set the corresponding bit to 1
		else keyBoardHIDsub.MODIFIER &= ~(1<<(keycode&0x0F)); //if released set the corresponding bit to  0
		return 0; //this never fails as each modifier key has its own bit
	}
	if (pressed){
		uint16_t free_slot = ROLLOVER;
		for (i = 0; i<ROLLOVER; i++){
			if(keyBoardHIDsub.KEYCODE[i] == keycode) return 2;//WTF this key is pressed already bail out
			if(keyBoardHIDsub.KEYCODE[i] == 0 && free_slot == ROLLOVER) free_slot = i; //yay there is space to add the keycode
		}
		if(free_slot >= ROLLOVER) return 1; //more than 6 keys are pressed at the same time. put your other hand back on the mouse (﻿ ͡° ͜ʖ ͡°)
		keyBoardHIDsub.KEYCODE[free_slot] = keycode; //add the keycode to the lowest free spot
		return 0; //done
	}

	for (i = 0; i<ROLLOVER; i++){
		if(keyBoardHIDsub.KEYCODE[i] == keycode){//we found the key
			keyBoardHIDsub.KEYCODE[i] = 0; //boom
			return 0; //done
		}
	}
	if(i>=ROLLOVER) return 2; //WTF you released a key that wasnt pressed


	return 3; //get outta here!
}


//update key status, deprecated
//scans all keys and sends a report if there was a change
void refreshKeys(void){
	for (int row=0; row < ROWS; row++){
		HAL_GPIO_WritePin(rows[row].Port, rows[row].Pin, GPIO_PIN_RESET); // set this pin to LOW
		HAL_Delay(1); //settle time

		for (int col=0; col < COLS; col++){
			int k = HAL_GPIO_ReadPin(cols[col].Port, cols[col].Pin);
			if (Keyboard[row][col].isPressed == k){
				Keyboard[row][col].hasChanged = 1;
				Keyboard[row][col].isPressed = !k;
				updateReport(Keycode_map[row][col], Keyboard[row][col].isPressed);
				KEY_CHANGE = 1;
			}
		}
		HAL_GPIO_WritePin(rows[row].Port, rows[row].Pin, GPIO_PIN_SET); // set it back to hi z
	}
	//if any key was pressed or released, send the damn thing
	if(KEY_CHANGE) {
		KEY_CHANGE = USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t *)&keyBoardHIDsub,sizeof(keyBoardHIDsub));
		HAL_Delay(10); //USB rate limiting
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
  MX_I2C1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM14_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(20);
  //init all led drivers
  initBL();
  //start keyboard refresh timer
  HAL_TIM_Base_Start_IT(&htim14);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t old_state = 0;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	if(keyBoardLEDState!=old_state){
		//blink_BL(keyBoardLEDState);
		old_state = keyBoardLEDState;
	}


	if(KEY_CHANGE) {
		while(USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t *)&keyBoardHIDsub,sizeof(keyBoardHIDsub))) HAL_Delay(10); //USB rate limiting
		KEY_CHANGE--;
	}


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
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 48000;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 20;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  htim14.Init.Prescaler = 48;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 1000;
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
