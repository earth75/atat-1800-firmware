/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "keycodes.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct
{
	GPIO_TypeDef* Port;
	uint16_t Pin;
}GPIO_t;

typedef	struct{
	   unsigned int DriverID;
	   unsigned int ChannelNbr;
} backlight_t;

typedef	struct{
	   unsigned int isPressed;
	   unsigned int hasChanged;
	   unsigned int value;
} keystate_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define ROWS 6
#define COLS 19
extern const GPIO_t rows[ROWS];
extern const GPIO_t cols[COLS];
extern const backlight_t Backlight_map[ROWS][COLS];
extern const char Keycode_map[ROWS][COLS];


/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
int updateReport(int keycode, int pressed);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define COL18_Pin GPIO_PIN_13
#define COL18_GPIO_Port GPIOC
#define COL17_Pin GPIO_PIN_14
#define COL17_GPIO_Port GPIOC
#define COL16_Pin GPIO_PIN_15
#define COL16_GPIO_Port GPIOC
#define COL15_Pin GPIO_PIN_0
#define COL15_GPIO_Port GPIOF
#define ROW0_Pin GPIO_PIN_0
#define ROW0_GPIO_Port GPIOA
#define ROW1_Pin GPIO_PIN_2
#define ROW1_GPIO_Port GPIOA
#define ROW2_Pin GPIO_PIN_3
#define ROW2_GPIO_Port GPIOA
#define ROW4_Pin GPIO_PIN_5
#define ROW4_GPIO_Port GPIOA
#define ROW5_Pin GPIO_PIN_6
#define ROW5_GPIO_Port GPIOA
#define ROW3_Pin GPIO_PIN_7
#define ROW3_GPIO_Port GPIOA
#define FAULT_Pin GPIO_PIN_0
#define FAULT_GPIO_Port GPIOB
#define COL7_Pin GPIO_PIN_1
#define COL7_GPIO_Port GPIOB
#define COL6_Pin GPIO_PIN_2
#define COL6_GPIO_Port GPIOB
#define COL5_Pin GPIO_PIN_10
#define COL5_GPIO_Port GPIOB
#define COL4_Pin GPIO_PIN_11
#define COL4_GPIO_Port GPIOB
#define COL9_Pin GPIO_PIN_12
#define COL9_GPIO_Port GPIOB
#define COL8_Pin GPIO_PIN_13
#define COL8_GPIO_Port GPIOB
#define COL3_Pin GPIO_PIN_14
#define COL3_GPIO_Port GPIOB
#define COL2_Pin GPIO_PIN_15
#define COL2_GPIO_Port GPIOB
#define COL1_Pin GPIO_PIN_8
#define COL1_GPIO_Port GPIOA
#define COL0_Pin GPIO_PIN_9
#define COL0_GPIO_Port GPIOA
#define USB_PWR_Pin GPIO_PIN_10
#define USB_PWR_GPIO_Port GPIOA
#define COL13_Pin GPIO_PIN_3
#define COL13_GPIO_Port GPIOB
#define COL12_Pin GPIO_PIN_4
#define COL12_GPIO_Port GPIOB
#define COL11_Pin GPIO_PIN_5
#define COL11_GPIO_Port GPIOB
#define COL10_Pin GPIO_PIN_6
#define COL10_GPIO_Port GPIOB
#define COL14_Pin GPIO_PIN_7
#define COL14_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
