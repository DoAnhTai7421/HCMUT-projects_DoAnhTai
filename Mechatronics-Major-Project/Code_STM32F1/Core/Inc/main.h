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
#include "stm32f1xx_hal.h"

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
#define TCR_1_Pin GPIO_PIN_0
#define TCR_1_GPIO_Port GPIOA
#define TCR_2_Pin GPIO_PIN_1
#define TCR_2_GPIO_Port GPIOA
#define TCR_3_Pin GPIO_PIN_2
#define TCR_3_GPIO_Port GPIOA
#define TCR_4_Pin GPIO_PIN_3
#define TCR_4_GPIO_Port GPIOA
#define TCR_5_Pin GPIO_PIN_4
#define TCR_5_GPIO_Port GPIOA
#define PWM_A_Pin GPIO_PIN_6
#define PWM_A_GPIO_Port GPIOA
#define PWM_B_Pin GPIO_PIN_7
#define PWM_B_GPIO_Port GPIOA
#define B2_Pin GPIO_PIN_12
#define B2_GPIO_Port GPIOB
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOB
#define A1_Pin GPIO_PIN_14
#define A1_GPIO_Port GPIOB
#define A2_Pin GPIO_PIN_15
#define A2_GPIO_Port GPIOB
#define ENA_A_Pin GPIO_PIN_8
#define ENA_A_GPIO_Port GPIOA
#define ENA_B_Pin GPIO_PIN_9
#define ENA_B_GPIO_Port GPIOA
#define ENB_A_Pin GPIO_PIN_15
#define ENB_A_GPIO_Port GPIOA
#define ENB_B_Pin GPIO_PIN_3
#define ENB_B_GPIO_Port GPIOB
#define S0_Pin GPIO_PIN_4
#define S0_GPIO_Port GPIOB
#define S1_Pin GPIO_PIN_5
#define S1_GPIO_Port GPIOB
#define PWM_INPUT_Pin GPIO_PIN_6
#define PWM_INPUT_GPIO_Port GPIOB
#define S2_Pin GPIO_PIN_7
#define S2_GPIO_Port GPIOB
#define S3_Pin GPIO_PIN_8
#define S3_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
