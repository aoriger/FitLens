/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

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
#define GPS_RX_Pin GPIO_PIN_0
#define GPS_RX_GPIO_Port GPIOC
#define GPS_TX_Pin GPIO_PIN_1
#define GPS_TX_GPIO_Port GPIOC
#define VCP_TX_Pin GPIO_PIN_2
#define VCP_TX_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_3
#define VCP_RX_GPIO_Port GPIOA
#define Light_Sensor_Pin GPIO_PIN_4
#define Light_Sensor_GPIO_Port GPIOA
#define Display_CS_Pin GPIO_PIN_12
#define Display_CS_GPIO_Port GPIOB
#define Display_SCK_Pin GPIO_PIN_13
#define Display_SCK_GPIO_Port GPIOB
#define Display_DC_Pin GPIO_PIN_14
#define Display_DC_GPIO_Port GPIOB
#define Display_MOSI_Pin GPIO_PIN_15
#define Display_MOSI_GPIO_Port GPIOB
#define Display_RST_Pin GPIO_PIN_6
#define Display_RST_GPIO_Port GPIOC
#define LED_Pin GPIO_PIN_9
#define LED_GPIO_Port GPIOA
#define SPI3_NSS_Pin GPIO_PIN_15
#define SPI3_NSS_GPIO_Port GPIOA
#define DATA_READY_Pin GPIO_PIN_3
#define DATA_READY_GPIO_Port GPIOB
#define NRF_READY_Pin GPIO_PIN_4
#define NRF_READY_GPIO_Port GPIOB
#define STM_READY_Pin GPIO_PIN_5
#define STM_READY_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
