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
#include "stm32f1xx_hal.h"
#include <stdio.h>
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SENZOR_ADC_Pin GPIO_PIN_0
#define SENZOR_ADC_GPIO_Port GPIOA
#define SENZOR_LED_Pin GPIO_PIN_1
#define SENZOR_LED_GPIO_Port GPIOA
#define VAGA_SCK_Pin GPIO_PIN_4
#define VAGA_SCK_GPIO_Port GPIOA
#define VAGA_DT_Pin GPIO_PIN_5
#define VAGA_DT_GPIO_Port GPIOA
#define STEPPER_DIR_Pin GPIO_PIN_6
#define STEPPER_DIR_GPIO_Port GPIOA
#define STEPPER_STEP_Pin GPIO_PIN_7
#define STEPPER_STEP_GPIO_Port GPIOA
#define SERVO2_Pin GPIO_PIN_9
#define SERVO2_GPIO_Port GPIOA
#define RPI_GPIO_Pin GPIO_PIN_11
#define RPI_GPIO_GPIO_Port GPIOA
#define STEPPER_EN_Pin GPIO_PIN_12
#define STEPPER_EN_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
#define RX_BUFF_SIZE (10)
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
