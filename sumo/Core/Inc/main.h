/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32h7xx_hal.h"
#include "stm32h7xx_ll_adc.h"
#include "stm32h7xx_ll_tim.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_cortex.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_system.h"
#include "stm32h7xx_ll_utils.h"
#include "stm32h7xx_ll_pwr.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_dma.h"

#include "stm32h7xx_ll_exti.h"

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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define DatCmd_Pin GPIO_PIN_3
#define DatCmd_GPIO_Port GPIOF
#define WSRST_Pin GPIO_PIN_4
#define WSRST_GPIO_Port GPIOF
#define SPINCS_Pin GPIO_PIN_5
#define SPINCS_GPIO_Port GPIOF
#define LED_GREEN_Pin GPIO_PIN_0
#define LED_GREEN_GPIO_Port GPIOB
#define Physical_Pin GPIO_PIN_0
#define Physical_GPIO_Port GPIOG
#define Physical_EXTI_IRQn EXTI0_IRQn
#define Remote_Pin GPIO_PIN_1
#define Remote_GPIO_Port GPIOG
#define Remote_EXTI_IRQn EXTI1_IRQn
#define Right60Xshut_Pin GPIO_PIN_10
#define Right60Xshut_GPIO_Port GPIOE
#define Right30Xshut_Pin GPIO_PIN_12
#define Right30Xshut_GPIO_Port GPIOE
#define MiddleXshut_Pin GPIO_PIN_14
#define MiddleXshut_GPIO_Port GPIOE
#define LED_RED_Pin GPIO_PIN_14
#define LED_RED_GPIO_Port GPIOB
#define STLK_VCP_RX_Pin GPIO_PIN_8
#define STLK_VCP_RX_GPIO_Port GPIOD
#define STLK_VCP_TX_Pin GPIO_PIN_9
#define STLK_VCP_TX_GPIO_Port GPIOD
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define Left30Xshut_Pin GPIO_PIN_4
#define Left30Xshut_GPIO_Port GPIOD
#define Left60Xshut_Pin GPIO_PIN_5
#define Left60Xshut_GPIO_Port GPIOD
#define LED_YELLOW_Pin GPIO_PIN_1
#define LED_YELLOW_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
