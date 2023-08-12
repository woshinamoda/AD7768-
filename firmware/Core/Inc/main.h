/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

	
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RESET_Pin GPIO_PIN_14
#define RESET_GPIO_Port GPIOG
#define START_Pin GPIO_PIN_13
#define START_GPIO_Port GPIOG
#define NOTIFY_Pin GPIO_PIN_7
#define NOTIFY_GPIO_Port GPIOB
#define MCLR_Pin GPIO_PIN_6
#define MCLR_GPIO_Port GPIOB
#define SPI1_CS_Pin GPIO_PIN_10
#define SPI1_CS_GPIO_Port GPIOG
#define DRDY_Pin GPIO_PIN_6
#define DRDY_GPIO_Port GPIOD
#define GD0_Pin GPIO_PIN_11
#define GD0_GPIO_Port GPIOA
#define FILTER_Pin GPIO_PIN_10
#define FILTER_GPIO_Port GPIOA
#define MODE3_Pin GPIO_PIN_9
#define MODE3_GPIO_Port GPIOA
#define MODE2_Pin GPIO_PIN_8
#define MODE2_GPIO_Port GPIOA
#define MODE1_Pin GPIO_PIN_7
#define MODE1_GPIO_Port GPIOC
#define MODE0_Pin GPIO_PIN_6
#define MODE0_GPIO_Port GPIOC
#define KEY_PA0_Pin GPIO_PIN_0
#define KEY_PA0_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_3
#define LED_GPIO_Port GPIOA
#define D7_Pin GPIO_PIN_7
#define D7_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
