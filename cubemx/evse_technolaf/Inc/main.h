/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
typedef uint8_t BOOLEAN;
#define FALSE 	0
#define TRUE  	!FALSE

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
#define BUTTON_Pin GPIO_PIN_0
#define BUTTON_GPIO_Port GPIOA
#define HV1_Pin GPIO_PIN_1
#define HV1_GPIO_Port GPIOA
#define HV2_Pin GPIO_PIN_2
#define HV2_GPIO_Port GPIOA
#define PILOT_ADC_Pin GPIO_PIN_3
#define PILOT_ADC_GPIO_Port GPIOA
#define IMEAS_L1_Pin GPIO_PIN_4
#define IMEAS_L1_GPIO_Port GPIOA
#define IMEAS_L2_Pin GPIO_PIN_5
#define IMEAS_L2_GPIO_Port GPIOA
#define LED_D8_Pin GPIO_PIN_0
#define LED_D8_GPIO_Port GPIOB
#define LED_D9_Pin GPIO_PIN_1
#define LED_D9_GPIO_Port GPIOB
#define LED_D10_Pin GPIO_PIN_2
#define LED_D10_GPIO_Port GPIOB
#define CONTACTOR_Pin GPIO_PIN_15
#define CONTACTOR_GPIO_Port GPIOB
#define LD4_Pin GPIO_PIN_8
#define LD4_GPIO_Port GPIOC
#define LD3_Pin GPIO_PIN_9
#define LD3_GPIO_Port GPIOC
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define PILOT_Pin GPIO_PIN_15
#define PILOT_GPIO_Port GPIOA
#define LED_D11_Pin GPIO_PIN_4
#define LED_D11_GPIO_Port GPIOB
#define LED_D12_Pin GPIO_PIN_5
#define LED_D12_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
