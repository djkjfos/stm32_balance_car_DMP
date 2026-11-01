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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define STATUS_LED_Pin GPIO_PIN_13
#define STATUS_LED_GPIO_Port GPIOC
#define TIM2_CH1_Pin GPIO_PIN_0
#define TIM2_CH1_GPIO_Port GPIOA
#define TIM2_CH2_Pin GPIO_PIN_1
#define TIM2_CH2_GPIO_Port GPIOA
#define LCD_CS_Pin GPIO_PIN_5
#define LCD_CS_GPIO_Port GPIOA
#define TIM3_CH1_Pin GPIO_PIN_6
#define TIM3_CH1_GPIO_Port GPIOA
#define TIM3_CH2_Pin GPIO_PIN_7
#define TIM3_CH2_GPIO_Port GPIOA
#define MOTOR_STBY_Pin GPIO_PIN_0
#define MOTOR_STBY_GPIO_Port GPIOB
#define USER_KEY_Pin GPIO_PIN_1
#define USER_KEY_GPIO_Port GPIOB
#define AIN1_Pin GPIO_PIN_12
#define AIN1_GPIO_Port GPIOB
#define AIN2_Pin GPIO_PIN_13
#define AIN2_GPIO_Port GPIOB
#define BIN1_Pin GPIO_PIN_14
#define BIN1_GPIO_Port GPIOB
#define BIN2_Pin GPIO_PIN_15
#define BIN2_GPIO_Port GPIOB
#define LCD_DC_Pin GPIO_PIN_8
#define LCD_DC_GPIO_Port GPIOA
#define LCD_RES_Pin GPIO_PIN_15
#define LCD_RES_GPIO_Port GPIOA
#define TIM4_CH1_Pin GPIO_PIN_6
#define TIM4_CH1_GPIO_Port GPIOB
#define TIM4_CH2_Pin GPIO_PIN_7
#define TIM4_CH2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
/*
 * 本头文件集中�? CubeMX 生成的引脚命名：STATUS_LED、USER_KEY、MOTOR_STBY 等�??
 * 电机/编码器模块还各有接口映射与方向宏，改接线时应与此处及 I2C.ioc �?起核对�??
 */


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
