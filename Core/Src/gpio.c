/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, Relay_1_Pin|Relay_2_Pin|Relay_3_CT1_Pin|Relay_3_CT2_Pin
                          |Relay_4_Pin|PS0_9959_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, TM1367_DIO_Pin|TM1367_CLK_Pin|ADF4002_LE_Pin|ADF4002_DAT_Pin
                          |ADF4002_CLK_Pin|SDIO3_9959_Pin|SDIO2_9959_Pin|SCLK_9959_Pin
                          |SDIO1_9959_Pin|CS_9959_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SDIO0_9959_Pin|UPDATE_9959_Pin|PS3_9959_Pin|RST_9959_Pin
                          |PS2_9959_Pin|PDC_9959_Pin|PS1_9959_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_B_Pin */
  GPIO_InitStruct.Pin = LED_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_B_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Relay_1_Pin Relay_2_Pin Relay_3_CT1_Pin Relay_3_CT2_Pin
                           Relay_4_Pin PS0_9959_Pin */
  GPIO_InitStruct.Pin = Relay_1_Pin|Relay_2_Pin|Relay_3_CT1_Pin|Relay_3_CT2_Pin
                          |Relay_4_Pin|PS0_9959_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : TM1367_DIO_Pin TM1367_CLK_Pin ADF4002_LE_Pin ADF4002_DAT_Pin
                           ADF4002_CLK_Pin SDIO3_9959_Pin SDIO2_9959_Pin SCLK_9959_Pin
                           SDIO1_9959_Pin CS_9959_Pin */
  GPIO_InitStruct.Pin = TM1367_DIO_Pin|TM1367_CLK_Pin|ADF4002_LE_Pin|ADF4002_DAT_Pin
                          |ADF4002_CLK_Pin|SDIO3_9959_Pin|SDIO2_9959_Pin|SCLK_9959_Pin
                          |SDIO1_9959_Pin|CS_9959_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : SDIO0_9959_Pin UPDATE_9959_Pin PS3_9959_Pin RST_9959_Pin
                           PS2_9959_Pin PDC_9959_Pin PS1_9959_Pin */
  GPIO_InitStruct.Pin = SDIO0_9959_Pin|UPDATE_9959_Pin|PS3_9959_Pin|RST_9959_Pin
                          |PS2_9959_Pin|PDC_9959_Pin|PS1_9959_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 (Key input) */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 (LED output) */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
