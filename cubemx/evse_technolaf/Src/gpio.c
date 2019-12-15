/**
  ******************************************************************************
  * File Name          : gpio.c
  * Description        : This file provides code for the configuration
  *                      of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_D8_Pin|LED_D9_Pin|LED_D10_Pin|CONTACTOR_Pin 
                          |LED_D11_Pin|LED_D12_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LD4_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin PBPin PBPin 
                           PBPin PBPin */
  GPIO_InitStruct.Pin = LED_D8_Pin|LED_D9_Pin|LED_D10_Pin|CONTACTOR_Pin 
                          |LED_D11_Pin|LED_D12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PCPin PCPin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */
void set_contactor(BOOLEAN state){
	if(state){
		HAL_GPIO_WritePin(CONTACTOR_GPIO_Port, CONTACTOR_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(CONTACTOR_GPIO_Port, CONTACTOR_Pin, GPIO_PIN_RESET);
	}

}

void set_indicator(INDICATOR led, BOOLEAN state){
	state = state ? GPIO_PIN_SET : GPIO_PIN_RESET;
	switch(led){
	case LED_DISCO_BLUE:
		HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, state);
		break;
	case LED_DISCO_GREEN:
		HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, state);
		break;
	case LED_POWER:
		HAL_GPIO_WritePin(LED_D8_GPIO_Port, LED_D8_Pin, state);
		break;
	case LED_READY:
		HAL_GPIO_WritePin(LED_D9_GPIO_Port, LED_D9_Pin, state);
		break;
	case LED_CONNECTED:
		HAL_GPIO_WritePin(LED_D10_GPIO_Port, LED_D10_Pin, state);
		break;
	case LED_CHARGING:
		HAL_GPIO_WritePin(LED_D11_GPIO_Port, LED_D11_Pin, state);
		break;
	case LED_FAULT:
		HAL_GPIO_WritePin(LED_D12_GPIO_Port, LED_D12_Pin, state);
		break;
	}
}

/* USER CODE END 2 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
