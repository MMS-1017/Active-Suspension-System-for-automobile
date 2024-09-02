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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdint.h"
#include "../Inc/Position_Control.h"
#include "../Inc/mpu_readings.h"
#include "../Inc/ParametersOfVehicle.h"
#include "../Inc/ICU_Functions.h"
#include "math.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define mpu
#define mpu2
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
#define mf_pwm_Pin GPIO_PIN_0
#define mf_pwm_GPIO_Port GPIOA
#define mf_dir1_Pin GPIO_PIN_1
#define mf_dir1_GPIO_Port GPIOA
#define mf_dir2_Pin GPIO_PIN_2
#define mf_dir2_GPIO_Port GPIOA
#define mb_dir1_Pin GPIO_PIN_3
#define mb_dir1_GPIO_Port GPIOA
#define mb_dir2_Pin GPIO_PIN_4
#define mb_dir2_GPIO_Port GPIOA
#define mb_pwm_Pin GPIO_PIN_5
#define mb_pwm_GPIO_Port GPIOA
#define mr_pwm_Pin GPIO_PIN_6
#define mr_pwm_GPIO_Port GPIOA
#define mr_dir1_Pin GPIO_PIN_7
#define mr_dir1_GPIO_Port GPIOA
#define mr_dir2_Pin GPIO_PIN_0
#define mr_dir2_GPIO_Port GPIOB
#define ml_dir1_Pin GPIO_PIN_1
#define ml_dir1_GPIO_Port GPIOB
#define ml_dir2_Pin GPIO_PIN_2
#define ml_dir2_GPIO_Port GPIOB
#define ml_pwm_Pin GPIO_PIN_10
#define ml_pwm_GPIO_Port GPIOB
#define MPU_int_Pin GPIO_PIN_13
#define MPU_int_GPIO_Port GPIOB
#define MPU_int_EXTI_IRQn EXTI15_10_IRQn
#define ml_enc2_Pin GPIO_PIN_11
#define ml_enc2_GPIO_Port GPIOA
#define ml_enc1_Pin GPIO_PIN_12
#define ml_enc1_GPIO_Port GPIOA
#define ml_enc1_EXTI_IRQn EXTI15_10_IRQn
#define mr_enc2_Pin GPIO_PIN_15
#define mr_enc2_GPIO_Port GPIOA
#define mr_enc1_Pin GPIO_PIN_3
#define mr_enc1_GPIO_Port GPIOB
#define mr_enc1_EXTI_IRQn EXTI3_IRQn
#define mb_enc2_Pin GPIO_PIN_4
#define mb_enc2_GPIO_Port GPIOB
#define mb_enc1_Pin GPIO_PIN_5
#define mb_enc1_GPIO_Port GPIOB
#define mb_enc1_EXTI_IRQn EXTI9_5_IRQn
#define mf_enc2_Pin GPIO_PIN_8
#define mf_enc2_GPIO_Port GPIOB
#define mf_enc1_Pin GPIO_PIN_9
#define mf_enc1_GPIO_Port GPIOB
#define mf_enc1_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
