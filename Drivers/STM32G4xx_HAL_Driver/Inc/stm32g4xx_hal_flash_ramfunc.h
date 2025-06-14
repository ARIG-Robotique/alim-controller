/**
  ******************************************************************************
  * @file    stm32g4xx_hal_flash_ramfunc.h
  * @author  MCD Application Team
  * @brief   Header file of FLASH RAMFUNC driver.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file in
  * the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STM32G4xx_FLASH_RAMFUNC_H
#define STM32G4xx_FLASH_RAMFUNC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal_def.h"

/** @addtogroup STM32G4xx_HAL_Driver
  * @{
  */

/** @addtogroup FLASH_RAMFUNC
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/** @addtogroup FLASH_RAMFUNC_Exported_Functions
  * @{
  */

/** @addtogroup FLASH_RAMFUNC_Exported_Functions_Group1
  * @{
  */
/* Peripheral Control functions  ************************************************/
__RAM_FUNC HAL_StatusTypeDef HAL_FLASHEx_EnableRunPowerDown(void);
__RAM_FUNC HAL_StatusTypeDef HAL_FLASHEx_DisableRunPowerDown(void);
#if defined (FLASH_OPTR_DBANK)
__RAM_FUNC HAL_StatusTypeDef HAL_FLASHEx_OB_DBankConfig(uint32_t DBankConfig);
#endif
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* STM32G4xx_FLASH_RAMFUNC_H */
