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
/* Definitions for soundTimer */
osTimerId_t soundTimerHandle;
const osTimerAttr_t soundTimer_attributes = {
  .name = "soundTimer"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void notifyAU(FDCAN_TxHeaderTypeDef txHeader);
void notifyAlim(FDCAN_TxHeaderTypeDef txHeader, Alimentation alim, uint8_t canId);
void notifyBattery(FDCAN_TxHeaderTypeDef txHeader);
void notifyVersion(FDCAN_TxHeaderTypeDef txHeader);

double lipoCellPercent(double tension);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void heartBeatCallback(void *argument);
void adcCallback(void *argument);
void soundCallback(void *argument);

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

  /* creation of soundTimer */
  soundTimerHandle = osTimerNew(soundCallback, osTimerOnce, NULL, &soundTimer_attributes);

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

  // Buzzer sounds works
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 27450);
  osTimerStart(soundTimerHandle, 0);

  // Warn led works
  for (int i = 0 ; i < 5 ; i++) {
    HAL_GPIO_TogglePin(WARN_BATT_GPIO_Port, WARN_BATT_Pin);
    osDelay(100);
  }
  HAL_GPIO_WritePin(WARN_BATT_GPIO_Port, WARN_BATT_Pin, GPIO_PIN_RESET);

  LOG_INFO("mainTask: Init variables");
  configuration.monitoredInternalAlim = false;
  configuration.monitoredExternalAlim = false;
  configuration.monitoredBattery = false;

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
  TxHeader.Identifier = 0;
  TxHeader.DataLength = 0;

  bool auPrec = -1;

  /* Infinite loop */
  // ReSharper disable once CppDFAEndlessLoop
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

      } else if (RxHeader.Identifier == GET_ALIM2_STATE) {
        notifyAlim(TxHeader, internalAlim, GET_ALIM2_STATE);

      } else if (RxHeader.Identifier == GET_ALIM3_STATE) {
        notifyAlim(TxHeader, externalAlim, GET_ALIM3_STATE);

      } else if (RxHeader.Identifier == GET_BATTERY_STATE) {
        notifyBattery(TxHeader);

      } else if (RxHeader.Identifier == GET_VERSION) {
        notifyVersion(TxHeader);

      } else if (RxHeader.Identifier == GET_SOUND) {
        LOG_INFO("mainTask: Sound");
        osTimerStart(soundTimerHandle, 0);

      } else if (RxHeader.Identifier == SET_CONFIG) {
        configuration.monitoredInternalAlim = RxData[0] & 0x01;
        configuration.monitoredExternalAlim = RxData[0] & 0x02;
        configuration.monitoredBattery = RxData[0] & 0x04;

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
  /* USER CODE END StartDefaultTask */
}

/* heartBeatCallback function */
void heartBeatCallback(void *argument)
{
  /* USER CODE BEGIN heartBeatCallback */

  // Heart Beat LED
  HAL_GPIO_TogglePin(HEART_BEAT_GPIO_Port, HEART_BEAT_Pin);

  // Battery critical
  if (battery.batteryPercent < 20) {
    // Batterie faible
    if (HAL_GPIO_ReadPin(HEART_BEAT_GPIO_Port, HEART_BEAT_Pin) == GPIO_PIN_SET) {
      HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    } else {
      HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    }
  }

  // Alim in fault or battery low
  if (internalAlim.fault || externalAlim.fault || battery.batteryPercent < 25) {
    // Alim en erreur
    for (int i = 0 ; i < 5 ; i++) {
      HAL_GPIO_TogglePin(WARN_BATT_GPIO_Port, WARN_BATT_Pin);
      osDelay(100);
    }
    HAL_GPIO_WritePin(WARN_BATT_GPIO_Port, WARN_BATT_Pin, GPIO_PIN_RESET);
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

  if (configuration.monitoredInternalAlim) {
    double oldTension = internalAlim.tension;
    double oldCurrent = internalAlim.current;

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

    if (internalAlim.current != oldCurrent || internalAlim.tension != oldTension) {
      LOG_INFO("adcCallback: Notify internal Alim");
      notifyAlim(TxHeader, internalAlim, GET_ALIM2_STATE);
    }
  }

  if (configuration.monitoredExternalAlim) {
    double oldTension = externalAlim.tension;
    double oldCurrent = externalAlim.current;

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

    if (externalAlim.current != oldCurrent || externalAlim.tension != oldTension) {
      LOG_INFO("adcCallback: Notify external Alims");
      notifyAlim(TxHeader, externalAlim, GET_ALIM3_STATE);
    }
  }

  if (configuration.monitoredBattery) {
    // Read battery
    LOG_INFO("adcCallback: Read Battery cell 1");
    adcSelectCell1Volt();
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 1000);
    rawAdc = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    battery.cell1Volt = (rawAdc / ADC_RESOLUTION) * (V_REF / GAIN_DIFFERENTIEL);
    battery.cell1Percent = lipoCellPercent(battery.cell1Volt);

    LOG_INFO("adcCallback: Read Battery cell 2");
    adcSelectCell2Volt();
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 1000);
    rawAdc = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    battery.cell2Volt = (rawAdc / ADC_RESOLUTION) * (V_REF / GAIN_DIFFERENTIEL);
    battery.cell2Percent = lipoCellPercent(battery.cell2Volt);

    LOG_INFO("adcCallback: Read Battery cell 3");
    adcSelectCell3Volt();
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 1000);
    rawAdc = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    battery.cell3Volt = (rawAdc / ADC_RESOLUTION) * (V_REF / GAIN_DIFFERENTIEL);
    battery.cell3Percent = lipoCellPercent(battery.cell3Volt);

    LOG_INFO("adcCallback: Read Battery cell 4");
    adcSelectCell4Volt();
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 1000);
    rawAdc = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    battery.cell4Volt = (rawAdc / ADC_RESOLUTION) * (V_REF / GAIN_DIFFERENTIEL);
    battery.cell4Percent = lipoCellPercent(battery.cell4Volt);

    double oldBatteryVolt = battery.batteryVolt;
    double oldBatteryPercent = battery.batteryPercent;
    battery.batteryVolt = battery.cell1Volt + battery.cell2Volt + battery.cell3Volt + battery.cell4Volt;
    battery.batteryPercent =
            (battery.cell1Percent + battery.cell2Percent + battery.cell3Percent + battery.cell4Percent) / 4.0;

    if (oldBatteryVolt != battery.batteryVolt || oldBatteryPercent != battery.batteryPercent) {
      LOG_INFO("adcCallback: Notify Battery");
      notifyBattery(TxHeader);
    }
  }

  /* USER CODE END adcCallback */
}

/* soundCallback function */
void soundCallback(void *argument)
{
  /* USER CODE BEGIN soundCallback */
  for (int i = 0 ; i < 10 ; i++) {
    if (i % 2) {
      HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    } else {
      HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    }
    osDelay(100);
  }
  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
  /* USER CODE END soundCallback */
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

void notifyAlim(FDCAN_TxHeaderTypeDef txHeader, Alimentation alim, uint8_t canId) {
  LOG_INFO("notifyAlim");

  // Notify
  txHeader.Identifier = canId;
  txHeader.DataLength = FDCAN_DLC_BYTES_5;

  uint8_t txBuffer[FDCAN_DLC_BYTES_5];
  uint16_t tension, current;

  // 0-1   : Alim tension
  tension = alim.tension * 100;
  txBuffer[0] = (tension >> 8) & 0xFF;
  txBuffer[1] = tension & 0xFF;
  // 2-3   : Alim current
  current = alim.current * 100;
  txBuffer[2] = (current >> 8) & 0xFF;
  txBuffer[3] = current & 0xFF;

  // 8    : 0 0 0 0 0 0 'ExternalAlim fault' 'Internal Alim fault'
  txBuffer[4] = alim.fault;

  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, txBuffer) != HAL_OK) {
    /* Transmission request Error */
    Error_Handler();
  }
}

void notifyBattery(FDCAN_TxHeaderTypeDef txHeader) {
  LOG_INFO("notifyBattery");

  // Notify
  txHeader.Identifier = GET_BATTERY_STATE;
  txHeader.DataLength = FDCAN_DLC_BYTES_4;

  uint8_t txBuffer[FDCAN_DLC_BYTES_4];
  uint16_t tension, percent;

  // 0-1 : Battery tension
  tension = battery.batteryVolt * 100;
  txBuffer[0] = (tension >> 8) & 0xFF;
  txBuffer[1] = tension & 0xFF;
  // 2-3 : Battery percent
  percent = battery.batteryPercent * 100;
  txBuffer[2] = (percent >> 8) & 0xFF;
  txBuffer[3] = percent & 0xFF;

  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, txBuffer) != HAL_OK) {
    /* Transmission request Error */
    Error_Handler();
  }

}

void notifyVersion(FDCAN_TxHeaderTypeDef txHeader) {
  LOG_INFO("notifyVersion");

  // Notify
  txHeader.Identifier = GET_VERSION;
  txHeader.DataLength = sizeof(VERSION_SHORT);

  uint8_t txBuffer[sizeof(VERSION_SHORT)];
  for (int i = 0 ; i < sizeof(VERSION_SHORT) ; i++) {
    txBuffer[i] = VERSION_SHORT[i];
  }

  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, txBuffer) != HAL_OK) {
    /* Transmission request Error */
    Error_Handler();
  }
}

double lipoCellPercent(double tension){
  // Fonctions affine déduite des données suivantes :
  // %	  U
  // 0	  3.0
  // 5	  3.3
  // 10	  3.6
  // 20	  3.7
  // 30	  3.75
  // 40	  3.79
  // 50	  3.83
  // 60	  3.87
  // 70	  3.92
  // 80	  3.97
  // 90	  4.1
  // 100	4.2
  //
  // Region 1 : y = 0.035 x + 3
  // Region 2 : y = 0.00528 x + 3.54714
  // Region 3 : y = 0.0115 x + 3.05

  // y = m * x + b
  // x = y - b / a
  // y = tension
  // x = percent
  double m;
  double b;
  if (tension <= 3.6) {
    m = 0.035;
    b = 3;
  } else if (tension > 3.6 && tension <= 3.97) {
    m = 0.00528;
    b = 3.54714;
  } else {
    m = 0.0115;
    b = 3.05;
  }
  return (tension - b) / m;
}
/* USER CODE END Application */
