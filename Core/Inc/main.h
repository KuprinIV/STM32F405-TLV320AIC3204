/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_R_Pin GPIO_PIN_13
#define LED_R_GPIO_Port GPIOC
#define LED_G_Pin GPIO_PIN_14
#define LED_G_GPIO_Port GPIOC
#define HP_DET_Pin GPIO_PIN_2
#define HP_DET_GPIO_Port GPIOC
#define SB6_Pin GPIO_PIN_0
#define SB6_GPIO_Port GPIOA
#define SB5_Pin GPIO_PIN_1
#define SB5_GPIO_Port GPIOA
#define SB4_Pin GPIO_PIN_2
#define SB4_GPIO_Port GPIOA
#define SB3_Pin GPIO_PIN_3
#define SB3_GPIO_Port GPIOA
#define SB2_Pin GPIO_PIN_4
#define SB2_GPIO_Port GPIOA
#define SB1_Pin GPIO_PIN_5
#define SB1_GPIO_Port GPIOA
#define J1_AV_Pin GPIO_PIN_6
#define J1_AV_GPIO_Port GPIOA
#define J1_AH_Pin GPIO_PIN_7
#define J1_AH_GPIO_Port GPIOA
#define J2_AV_Pin GPIO_PIN_4
#define J2_AV_GPIO_Port GPIOC
#define J2_AH_Pin GPIO_PIN_5
#define J2_AH_GPIO_Port GPIOC
#define LS_EN_Pin GPIO_PIN_2
#define LS_EN_GPIO_Port GPIOB
#define CODEC_RST_Pin GPIO_PIN_7
#define CODEC_RST_GPIO_Port GPIOC
#define AUD_EN_Pin GPIO_PIN_8
#define AUD_EN_GPIO_Port GPIOC
#define BT_BOOT_Pin GPIO_PIN_12
#define BT_BOOT_GPIO_Port GPIOC
#define BT_RST_Pin GPIO_PIN_2
#define BT_RST_GPIO_Port GPIOD
#define LED_Pin GPIO_PIN_6
#define LED_GPIO_Port GPIOB

#define PWR_BTN_Pin GPIO_PIN_7
#define PWR_BTN_GPIO_Port GPIOB
#define GHGEN_Pin GPIO_PIN_3
#define GHGEN_GPIO_Port GPIOB
#define INT_Pin GPIO_PIN_4
#define INT_GPIO_Port GPIOB
#define INT_EXTI_IRQn EXTI4_IRQn
#define PG_Pin GPIO_PIN_5
#define PG_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
