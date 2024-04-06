/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "itm_log.h"
#include "adc.h"
#include "tim.h"
#include "fdcan.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for heartBeatTimer */
osTimerId_t heartBeatTimerHandle;
const osTimerAttr_t heartBeatTimer_attributes = {
  .name = "heartBeatTimer"
};
/* Definitions for adcTimer */
osTimerId_t adcTimerHandle;
const osTimerAttr_t adcTimer_attributes = {
  .name = "adcTimer"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void notifyAU(FDCAN_TxHeaderTypeDef txHeader);
void notifyAlims(FDCAN_TxHeaderTypeDef txHeader);
void notifyBattery(FDCAN_TxHeaderTypeDef txHeader);
void notifyVersion(FDCAN_TxHeaderTypeDef txHeader);

double lipoCellPercent(double tension);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void heartBeatCallback(void *argument);
void adcCallback(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  LOG_INFO("freertos: Init FreeRTOS");
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of heartBeatTimer */
  heartBeatTimerHandle = osTimerNew(heartBeatCallback, osTimerPeriodic, NULL, &heartBeatTimer_attributes);

  /* creation of adcTimer */
  adcTimerHandle = osTimerNew(adcCallback, osTimerPeriodic, NULL, &adcTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  LOG_INFO("mainTask: Start");

  // Buzzer sound works
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 27450);
  for (int i = 0 ; i < 10 ; i++) {
    if (i % 2) {
      HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    } else {
      HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    }
    osDelay(100);
  }
  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);

  LOG_INFO("mainTask: Init variables");
  internalAlim.tension = 0.0;
  internalAlim.current = 0.0;
  internalAlim.fault = false;

  externalAlim.tension = 0.0;
  externalAlim.current = 0.0;
  externalAlim.fault = false;

  battery.cell1Volt = 4.2;
  battery.cell1Percent = 100.0;
  battery.cell2Volt = 4.2;
  battery.cell2Percent = 100.0;
  battery.cell3Volt = 4.2;
  battery.cell3Percent = 100.0;
  battery.cell4Volt = 4.2;
  battery.cell4Percent = 100.0;
  battery.batteryVolt = 16.8;
  battery.batteryPercent = 100.0;

  LOG_INFO("mainTask: Start FDCan listener");
  HAL_FDCAN_Start(&hfdcan1);

  // Start timers after boot init
  osTimerStart(heartBeatTimerHandle, 1000);
  osTimerStart(adcTimerHandle, 2000);

  /* Prepare FDCAN Tx */
  FDCAN_TxHeaderTypeDef TxHeader;

  TxHeader.IdType = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;

  bool auPrec = -1;

  /* Infinite loop */
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
  while(true) {
    LOG_INFO("mainTask: Read AU");
    au = HAL_GPIO_ReadPin(AU_GPIO_Port, AU_Pin) == GPIO_PIN_RESET;
    if (au != auPrec) {
      LOG_INFO("mainTask: AU changed");
      auPrec = au;
      notifyAU(TxHeader);
    }

    // Print AU state on LED
    if (au) {
      HAL_GPIO_WritePin(AU_OK_GPIO_Port, AU_OK_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(AU_KO_GPIO_Port, AU_KO_Pin, GPIO_PIN_RESET);
    } else {
      HAL_GPIO_WritePin(AU_OK_GPIO_Port, AU_OK_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(AU_KO_GPIO_Port, AU_KO_Pin, GPIO_PIN_SET);
    }

    /* Check if a message is come */
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t               RxData[1];
    HAL_StatusTypeDef canRequest = HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData);
    if (canRequest == HAL_OK) {
      TxHeader.Identifier = RxHeader.Identifier;
      if (RxHeader.Identifier == GET_AU_STATE) {
        notifyAU(TxHeader);

      } else if (RxHeader.Identifier == GET_ALIMS_STATE) {
        notifyAlims(TxHeader);

      } else if (RxHeader.Identifier == GET_BATTERY_STATE) {
        notifyBattery(TxHeader);

      } else if (RxHeader.Identifier == GET_VERSION) {
        notifyVersion(TxHeader);

      } else if (RxHeader.Identifier == SET_ALIM_2) {
        bool enable = RxData[0] & 0x01;
        if (enable) {
          HAL_GPIO_WritePin(PIL_ALIM_2_GPIO_Port, PIL_ALIM_2_Pin, GPIO_PIN_RESET);
        } else {
          HAL_GPIO_WritePin(PIL_ALIM_2_GPIO_Port, PIL_ALIM_2_Pin, GPIO_PIN_SET);
        }

      } else if (RxHeader.Identifier == SET_ALIM_3) {
        bool enable = RxData[0] & 0x01;
        if (enable) {
          HAL_GPIO_WritePin(PIL_ALIM_3_GPIO_Port, PIL_ALIM_3_Pin, GPIO_PIN_RESET);
        } else {
          HAL_GPIO_WritePin(PIL_ALIM_3_GPIO_Port, PIL_ALIM_3_Pin, GPIO_PIN_SET);
        }
      } else {
        LOG_WARN("mainTask: Unknown message");
      }
    }
    osDelay(50);
  }
#pragma clang diagnostic pop
  /* USER CODE END StartDefaultTask */
}

/* heartBeatCallback function */
void heartBeatCallback(void *argument)
{
  /* USER CODE BEGIN heartBeatCallback */
  if (!internalAlim.fault && !externalAlim.fault) {
    HAL_GPIO_TogglePin(HEART_BEAT_GPIO_Port, HEART_BEAT_Pin);
  } else {
    // Alim en erreur
    HAL_GPIO_WritePin(HEART_BEAT_GPIO_Port, HEART_BEAT_Pin, GPIO_PIN_SET);
  }
  /* USER CODE END heartBeatCallback */
}

/* adcCallback function */
void adcCallback(void *argument)
{
  /* USER CODE BEGIN adcCallback */

  FDCAN_TxHeaderTypeDef TxHeader;

  TxHeader.IdType = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;

  // Read fault
  internalAlim.fault = HAL_GPIO_ReadPin(FAULT_2_GPIO_Port, FAULT_2_Pin) == GPIO_PIN_RESET;
  externalAlim.fault = HAL_GPIO_ReadPin(FAULT_3_GPIO_Port, FAULT_3_Pin) == GPIO_PIN_RESET;

  uint32_t rawAdc;

  // Read ADC Alims
  LOG_INFO("adcCallback: Read ADC Internal Alim Volt");
  adcSelectInternalAlimVolt();
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 1000);
  rawAdc = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);
  internalAlim.tension = (rawAdc / ADC_RESOLUTION) * (V_REF / DIVISEUR_TENSION);

  LOG_INFO("adcCallback: Read ADC Internal Alim Current");
  adcSelectInternalAlimCurrent();
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 1000);
  rawAdc = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);
  internalAlim.current = (rawAdc / ADC_RESOLUTION) * V_REF * ACS_RESOLUTION;

  LOG_INFO("adcCallback: Read ADC External Alim Volt");
  adcSelectExternalAlimVolt();
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 1000);
  rawAdc = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);
  externalAlim.tension = (rawAdc / ADC_RESOLUTION) * (V_REF / DIVISEUR_TENSION);

  LOG_INFO("adcCallback: Read ADC External Alim Current");
  adcSelectExternalAlimCurrent();
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 1000);
  rawAdc = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);
  externalAlim.current = (rawAdc / ADC_RESOLUTION) * V_REF * ACS_RESOLUTION;

  LOG_INFO("adcCallback: Notify Alims");
  notifyAlims(TxHeader);

  // Read battery
  LOG_INFO("adcCallback: Read Battery cell 1");
  adcSelectCell1Volt();
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 1000);
  rawAdc = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);
  battery.cell1Volt = (rawAdc / ADC_RESOLUTION) * (V_REF / DIVISEUR_TENSION);
  battery.cell1Percent = lipoCellPercent(battery.cell1Volt);

  LOG_INFO("adcCallback: Read Battery cell 2");
  adcSelectCell2Volt();
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 1000);
  rawAdc = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);
  battery.cell2Volt = (rawAdc / ADC_RESOLUTION) * (V_REF / DIVISEUR_TENSION);
  battery.cell2Percent = lipoCellPercent(battery.cell2Volt);

  LOG_INFO("adcCallback: Read Battery cell 3");
  adcSelectCell3Volt();
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 1000);
  rawAdc = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);
  battery.cell3Volt = (rawAdc / ADC_RESOLUTION) * (V_REF / DIVISEUR_TENSION);
  battery.cell3Percent = lipoCellPercent(battery.cell3Volt);

  LOG_INFO("adcCallback: Read Battery cell 4");
  adcSelectCell4Volt();
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 1000);
  rawAdc = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);
  battery.cell4Volt = (rawAdc / ADC_RESOLUTION) * (V_REF / DIVISEUR_TENSION);
  battery.cell4Percent = lipoCellPercent(battery.cell4Volt);

  battery.batteryVolt = battery.cell1Volt + battery.cell2Volt + battery.cell3Volt + battery.cell4Volt;
  battery.batteryPercent = (battery.cell1Percent + battery.cell2Percent + battery.cell3Percent + battery.cell4Percent) / 4.0;

  LOG_INFO("Notify Alims");
  notifyBattery(TxHeader);

  /* USER CODE END adcCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void notifyAU(FDCAN_TxHeaderTypeDef txHeader) {
  LOG_INFO("notifyAU");

  // Notify
  txHeader.Identifier = GET_AU_STATE;
  txHeader.DataLength = FDCAN_DLC_BYTES_1;
  uint8_t data = au ? 0x01 : 0x00;

  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, &data) != HAL_OK) {
    /* Transmission request Error */
    Error_Handler();
  }
}

void notifyAlims(FDCAN_TxHeaderTypeDef txHeader) {
  LOG_INFO("notifyAlims");

  // Notify
  txHeader.Identifier = GET_ALIMS_STATE;
  txHeader.DataLength = 9;

  uint8_t txBuffer[9];
  uint16_t tension, current;

  // 0-1   : Alim 1 tension
  tension = internalAlim.tension * 100;
  txBuffer[0] = (tension >> 8) & 0xFF;
  txBuffer[1] = tension & 0xFF;
  // 2-3   : Alim 1 current
  current = internalAlim.current * 100;
  txBuffer[2] = (current >> 8) & 0xFF;
  txBuffer[3] = current & 0xFF;

  // 4-5  : Alim 2 tension
  tension = externalAlim.tension * 100;
  txBuffer[4] = (tension >> 8) & 0xFF;
  txBuffer[5] = tension & 0xFF;
  // 6-7 : Alim 2 current
  current = externalAlim.current * 100;
  txBuffer[6] = (current >> 8) & 0xFF;
  txBuffer[7] = current & 0xFF;

  // 8    : 0 0 0 0 0 0 'ExternalAlim fault' 'Internal Alim fault'
  txBuffer[8] = (externalAlim.fault << 1) + internalAlim.fault;

  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, &txBuffer) != HAL_OK) {
    /* Transmission request Error */
    Error_Handler();
  }

}

void notifyBattery(FDCAN_TxHeaderTypeDef txHeader) {
  LOG_INFO("notifyBattery");

  // Notify
  txHeader.Identifier = GET_BATTERY_STATE;
  txHeader.DataLength = 20;

  uint8_t txBuffer[20];
  uint16_t tension, percent;

  // 0-1   : Cell 1 tension
  tension = battery.cell1Volt * 100;
  txBuffer[0] = (tension >> 8) & 0xFF;
  txBuffer[1] = tension & 0xFF;
  // 2-3   : Cell 1 percent
  percent = battery.cell1Percent * 100;
  txBuffer[2] = (percent >> 8) & 0xFF;
  txBuffer[3] = percent & 0xFF;

  // 4-5   : Cell 2 tension
  tension = battery.cell2Volt * 100;
  txBuffer[4] = (tension >> 8) & 0xFF;
  txBuffer[5] = tension & 0xFF;
  // 6-7   : Cell 2 percent
  percent = battery.cell2Percent * 100;
  txBuffer[6] = (percent >> 8) & 0xFF;
  txBuffer[7] = percent & 0xFF;

  // 8-9   : Cell 3 tension
  tension = battery.cell3Volt * 100;
  txBuffer[8] = (tension >> 8) & 0xFF;
  txBuffer[9] = tension & 0xFF;
  // 10-11 : Cell 3 percent
  percent = battery.cell3Percent * 100;
  txBuffer[10] = (percent >> 8) & 0xFF;
  txBuffer[11] = percent & 0xFF;

  // 12-13 : Cell 4 tension
  tension = battery.cell4Volt * 100;
  txBuffer[12] = (tension >> 8) & 0xFF;
  txBuffer[13] = tension & 0xFF;
  // 14-15 : Cell 4 percent
  percent = battery.cell4Percent * 100;
  txBuffer[14] = (percent >> 8) & 0xFF;
  txBuffer[15] = percent & 0xFF;

  // 16-17 : Battery tension
  tension = battery.batteryVolt * 100;
  txBuffer[16] = (tension >> 8) & 0xFF;
  txBuffer[17] = tension & 0xFF;
  // 18-19 : Battery percent
  percent = battery.batteryPercent * 100;
  txBuffer[18] = (percent >> 8) & 0xFF;
  txBuffer[19] = percent & 0xFF;

  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, &txBuffer) != HAL_OK) {
    /* Transmission request Error */
    Error_Handler();
  }
}

void notifyVersion(FDCAN_TxHeaderTypeDef txHeader) {
  LOG_INFO("notifyVersion");

  // Notify
  txHeader.Identifier = GET_VERSION;
  txHeader.DataLength = 19;

  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, FIRMWARE_VERSION) != HAL_OK) {
    /* Transmission request Error */
    Error_Handler();
  }
}

double lipoCellPercent(double tension){
  // TODO : Bad formula
  return (tension - 3.0) / 1.2 * 100;
}
/* USER CODE END Application */

