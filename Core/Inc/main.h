/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#define JSLEFT_SWF_Pin GPIO_PIN_0
#define JSLEFT_SWF_GPIO_Port GPIOC
#define JSLEFT_SWB_Pin GPIO_PIN_1
#define JSLEFT_SWB_GPIO_Port GPIOC
#define JSLEFT_SWH_Pin GPIO_PIN_2
#define JSLEFT_SWH_GPIO_Port GPIOC
#define JSLEFT_SPD_Pin GPIO_PIN_3
#define JSLEFT_SPD_GPIO_Port GPIOC
#define JSRIGHT_SWF_Pin GPIO_PIN_0
#define JSRIGHT_SWF_GPIO_Port GPIOA
#define JSRIGHT_SWB_Pin GPIO_PIN_1
#define JSRIGHT_SWB_GPIO_Port GPIOA
#define JSRIGHT_SWH_Pin GPIO_PIN_2
#define JSRIGHT_SWH_GPIO_Port GPIOA
#define JSRIGHT_SPD_Pin GPIO_PIN_3
#define JSRIGHT_SPD_GPIO_Port GPIOA
#define COMSPI_NSS_Pin GPIO_PIN_4
#define COMSPI_NSS_GPIO_Port GPIOA
#define COMSPI_SCK_Pin GPIO_PIN_5
#define COMSPI_SCK_GPIO_Port GPIOA
#define COMSPI_MISO_Pin GPIO_PIN_6
#define COMSPI_MISO_GPIO_Port GPIOA
#define COMSPI_MOSI_Pin GPIO_PIN_7
#define COMSPI_MOSI_GPIO_Port GPIOA
#define LED_NFC_Pin GPIO_PIN_7
#define LED_NFC_GPIO_Port GPIOE
#define LED_UWB_Pin GPIO_PIN_8
#define LED_UWB_GPIO_Port GPIOE
#define LED_IMU_Pin GPIO_PIN_9
#define LED_IMU_GPIO_Port GPIOE
#define LED_MDR_Pin GPIO_PIN_10
#define LED_MDR_GPIO_Port GPIOE
#define LED_MDL_Pin GPIO_PIN_11
#define LED_MDL_GPIO_Port GPIOE
#define LED_COM_Pin GPIO_PIN_12
#define LED_COM_GPIO_Port GPIOE
#define UWB_TX_Pin GPIO_PIN_10
#define UWB_TX_GPIO_Port GPIOB
#define UWB_RX_Pin GPIO_PIN_11
#define UWB_RX_GPIO_Port GPIOB
#define IMU_SCL_Pin GPIO_PIN_6
#define IMU_SCL_GPIO_Port GPIOB
#define IMU_SDA_Pin GPIO_PIN_7
#define IMU_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
