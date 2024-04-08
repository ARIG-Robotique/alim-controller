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
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct {
    double tension;
    double current;
    bool fault;
} Alimentation;

typedef struct {
    double cell1Volt;
    double cell1Percent;
    double cell2Volt;
    double cell2Percent;
    double cell3Volt;
    double cell3Percent;
    double cell4Volt;
    double cell4Percent;
    double batteryVolt;
    double batteryPercent;
} Battery;

extern Alimentation internalAlim;
extern Alimentation externalAlim;

extern Battery battery;

extern bool au;

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
#define WARN_BATT_Pin GPIO_PIN_15
#define WARN_BATT_GPIO_Port GPIOC
#define CELL_4_Pin GPIO_PIN_0
#define CELL_4_GPIO_Port GPIOC
#define CELL_3_Pin GPIO_PIN_1
#define CELL_3_GPIO_Port GPIOC
#define CELL_2_Pin GPIO_PIN_2
#define CELL_2_GPIO_Port GPIOC
#define CELL_1_Pin GPIO_PIN_3
#define CELL_1_GPIO_Port GPIOC
#define ANA_VOLT_3_Pin GPIO_PIN_0
#define ANA_VOLT_3_GPIO_Port GPIOA
#define ANA_CURRENT_3_Pin GPIO_PIN_1
#define ANA_CURRENT_3_GPIO_Port GPIOA
#define ANA_VOLT_2_Pin GPIO_PIN_2
#define ANA_VOLT_2_GPIO_Port GPIOA
#define ANA_CURRENT_2_Pin GPIO_PIN_3
#define ANA_CURRENT_2_GPIO_Port GPIOA
#define FAULT_3_Pin GPIO_PIN_4
#define FAULT_3_GPIO_Port GPIOA
#define FAULT_2_Pin GPIO_PIN_5
#define FAULT_2_GPIO_Port GPIOA
#define PIL_ALIM_2_Pin GPIO_PIN_0
#define PIL_ALIM_2_GPIO_Port GPIOB
#define PIL_ALIM_3_Pin GPIO_PIN_1
#define PIL_ALIM_3_GPIO_Port GPIOB
#define HEART_BEAT_Pin GPIO_PIN_7
#define HEART_BEAT_GPIO_Port GPIOC
#define ADD_2_Pin GPIO_PIN_8
#define ADD_2_GPIO_Port GPIOC
#define ADD_1_Pin GPIO_PIN_9
#define ADD_1_GPIO_Port GPIOC
#define BUZZER_Pin GPIO_PIN_15
#define BUZZER_GPIO_Port GPIOA
#define AU_KO_Pin GPIO_PIN_4
#define AU_KO_GPIO_Port GPIOB
#define AU_OK_Pin GPIO_PIN_5
#define AU_OK_GPIO_Port GPIOB
#define AU_Pin GPIO_PIN_6
#define AU_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

// Tension d'alimentation de référence
#define V_REF 3.3

// Résolution des convertisseurs ADC de la STM32G474RET6
#define ADC_RESOLUTION 4096.0

// Valeur de convertion du pont diviseur de tension (13.6 V => 3.3 V)
// R1 = 1.8K ; R2 = 5.6K
// Vo = Vi * R1 / (R1 + R2) = Vi * 0.243243243
#define DIVISEUR_TENSION 0.243243243

// Pourcentage des céllule LIPO
#define CELL_100 4.20
#define CELL_90 4.10
#define CELL_80 3.97
#define CELL_70 3.92
#define CELL_60 3.87
#define CELL_50 3.83
#define CELL_40 3.79
#define CELL_30 3.75
#define CELL_20 3.7
#define CELL_10 3.6
#define CELL_5 3.3
#define CELL_0 3.0

// Résolution pour le courant de l'ACS711 15A alimenté en 3.3V, 90mV / A
#define ACS_RESOLUTION 90.0/1000.0

// CAN Message ID
#define SET_ALIM_2 1
#define SET_ALIM_3 2
#define GET_VERSION 3
#define GET_AU_STATE 4
#define GET_ALIMS_STATE 5
#define GET_BATTERY_STATE 6
#define GET_SOUND 10

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
