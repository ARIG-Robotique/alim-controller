/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

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
#define FAULT_1_Pin GPIO_PIN_3
#define FAULT_1_GPIO_Port GPIOC
#define ANA_VOLT_1_Pin GPIO_PIN_0
#define ANA_VOLT_1_GPIO_Port GPIOA
#define ANA_CURRENT_1_Pin GPIO_PIN_1
#define ANA_CURRENT_1_GPIO_Port GPIOA
#define ANA_VOLT_2_Pin GPIO_PIN_2
#define ANA_VOLT_2_GPIO_Port GPIOA
#define ANA_CURRENT_2_Pin GPIO_PIN_3
#define ANA_CURRENT_2_GPIO_Port GPIOA
#define FAULT_2_Pin GPIO_PIN_4
#define FAULT_2_GPIO_Port GPIOA
#define FAULT_3_Pin GPIO_PIN_5
#define FAULT_3_GPIO_Port GPIOA
#define ANA_VOLT_3_Pin GPIO_PIN_6
#define ANA_VOLT_3_GPIO_Port GPIOA
#define ANA_CURRENT_3_Pin GPIO_PIN_7
#define ANA_CURRENT_3_GPIO_Port GPIOA
#define PIL_ALIM_2_Pin GPIO_PIN_0
#define PIL_ALIM_2_GPIO_Port GPIOB
#define PIL_ALIM_3_Pin GPIO_PIN_1
#define PIL_ALIM_3_GPIO_Port GPIOB
#define AU_KO_Pin GPIO_PIN_11
#define AU_KO_GPIO_Port GPIOB
#define AU_OK_Pin GPIO_PIN_13
#define AU_OK_GPIO_Port GPIOB
#define AU_Pin GPIO_PIN_15
#define AU_GPIO_Port GPIOB
#define HEART_BEAT_Pin GPIO_PIN_7
#define HEART_BEAT_GPIO_Port GPIOC
#define ADD_2_Pin GPIO_PIN_8
#define ADD_2_GPIO_Port GPIOC
#define ADD_1_Pin GPIO_PIN_9
#define ADD_1_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
