/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

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
#define LED_B_Pin GPIO_PIN_5
#define LED_B_GPIO_Port GPIOC
#define Relay_1_Pin GPIO_PIN_7
#define Relay_1_GPIO_Port GPIOE
#define Relay_2_Pin GPIO_PIN_8
#define Relay_2_GPIO_Port GPIOE
#define Relay_3_CT1_Pin GPIO_PIN_9
#define Relay_3_CT1_GPIO_Port GPIOE
#define Relay_3_CT2_Pin GPIO_PIN_10
#define Relay_3_CT2_GPIO_Port GPIOE
#define Relay_4_Pin GPIO_PIN_11
#define Relay_4_GPIO_Port GPIOE
#define TM1367_DIO_Pin GPIO_PIN_14
#define TM1367_DIO_GPIO_Port GPIOD
#define TM1367_CLK_Pin GPIO_PIN_15
#define TM1367_CLK_GPIO_Port GPIOD
#define ADF4002_LE_Pin GPIO_PIN_0
#define ADF4002_LE_GPIO_Port GPIOD
#define ADF4002_DAT_Pin GPIO_PIN_1
#define ADF4002_DAT_GPIO_Port GPIOD
#define ADF4002_CLK_Pin GPIO_PIN_2
#define ADF4002_CLK_GPIO_Port GPIOD
#define SDIO3_9959_Pin GPIO_PIN_3
#define SDIO3_9959_GPIO_Port GPIOD
#define SDIO2_9959_Pin GPIO_PIN_4
#define SDIO2_9959_GPIO_Port GPIOD
#define SCLK_9959_Pin GPIO_PIN_5
#define SCLK_9959_GPIO_Port GPIOD
#define SDIO1_9959_Pin GPIO_PIN_6
#define SDIO1_9959_GPIO_Port GPIOD
#define CS_9959_Pin GPIO_PIN_7
#define CS_9959_GPIO_Port GPIOD
#define SDIO0_9959_Pin GPIO_PIN_3
#define SDIO0_9959_GPIO_Port GPIOB
#define UPDATE_9959_Pin GPIO_PIN_4
#define UPDATE_9959_GPIO_Port GPIOB
#define PS3_9959_Pin GPIO_PIN_5
#define PS3_9959_GPIO_Port GPIOB
#define RST_9959_Pin GPIO_PIN_6
#define RST_9959_GPIO_Port GPIOB
#define PS2_9959_Pin GPIO_PIN_7
#define PS2_9959_GPIO_Port GPIOB
#define PDC_9959_Pin GPIO_PIN_8
#define PDC_9959_GPIO_Port GPIOB
#define PS1_9959_Pin GPIO_PIN_9
#define PS1_9959_GPIO_Port GPIOB
#define PS0_9959_Pin GPIO_PIN_1
#define PS0_9959_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
