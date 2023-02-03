/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern uint8_t Position_zeroed_Flag;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */



/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define M5_PWM_Pin GPIO_PIN_5
#define M5_PWM_GPIO_Port GPIOE
#define M6_PWM_Pin GPIO_PIN_6
#define M6_PWM_GPIO_Port GPIOE
#define LED1_Pin GPIO_PIN_0
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_1
#define LED2_GPIO_Port GPIOC
#define LED3_Pin GPIO_PIN_2
#define LED3_GPIO_Port GPIOC
#define M5_EncoderA_Pin GPIO_PIN_0
#define M5_EncoderA_GPIO_Port GPIOA
#define M5_EncoderB_Pin GPIO_PIN_1
#define M5_EncoderB_GPIO_Port GPIOA
#define M5_ADC_Pin GPIO_PIN_4
#define M5_ADC_GPIO_Port GPIOA
#define M6_ADC_Pin GPIO_PIN_5
#define M6_ADC_GPIO_Port GPIOA
#define M3_PWM_Pin GPIO_PIN_6
#define M3_PWM_GPIO_Port GPIOA
#define M4_PWM_Pin GPIO_PIN_7
#define M4_PWM_GPIO_Port GPIOA
#define M1_ADC_Pin GPIO_PIN_4
#define M1_ADC_GPIO_Port GPIOC
#define M3_ADC_Pin GPIO_PIN_5
#define M3_ADC_GPIO_Port GPIOC
#define M4_ADC_Pin GPIO_PIN_0
#define M4_ADC_GPIO_Port GPIOB
#define M2_ADC_Pin GPIO_PIN_1
#define M2_ADC_GPIO_Port GPIOB
#define M1_EncoderA_Pin GPIO_PIN_9
#define M1_EncoderA_GPIO_Port GPIOE
#define M1_EncoderB_Pin GPIO_PIN_11
#define M1_EncoderB_GPIO_Port GPIOE
#define M5_In1_Pin GPIO_PIN_12
#define M5_In1_GPIO_Port GPIOE
#define M5_In2_Pin GPIO_PIN_13
#define M5_In2_GPIO_Port GPIOE
#define M6_In1_Pin GPIO_PIN_14
#define M6_In1_GPIO_Port GPIOE
#define M6_In2_Pin GPIO_PIN_15
#define M6_In2_GPIO_Port GPIOE
#define M1_PWM_Pin GPIO_PIN_14
#define M1_PWM_GPIO_Port GPIOB
#define M2_PWM_Pin GPIO_PIN_15
#define M2_PWM_GPIO_Port GPIOB
#define KEY_Up_Pin GPIO_PIN_8
#define KEY_Up_GPIO_Port GPIOD
#define KEY_Down_Pin GPIO_PIN_9
#define KEY_Down_GPIO_Port GPIOD
#define KEY_Last_Pin GPIO_PIN_10
#define KEY_Last_GPIO_Port GPIOD
#define KEY_Next_Pin GPIO_PIN_11
#define KEY_Next_GPIO_Port GPIOD
#define M4_EncoderA_Pin GPIO_PIN_12
#define M4_EncoderA_GPIO_Port GPIOD
#define M4_EncoderB_Pin GPIO_PIN_13
#define M4_EncoderB_GPIO_Port GPIOD
#define M6_EncoderA_Pin GPIO_PIN_6
#define M6_EncoderA_GPIO_Port GPIOC
#define M6_EncoderB_Pin GPIO_PIN_7
#define M6_EncoderB_GPIO_Port GPIOC
#define M2_EncoderA_Pin GPIO_PIN_15
#define M2_EncoderA_GPIO_Port GPIOA
#define M1_In1_Pin GPIO_PIN_0
#define M1_In1_GPIO_Port GPIOD
#define M1_In2_Pin GPIO_PIN_1
#define M1_In2_GPIO_Port GPIOD
#define M2_In1_Pin GPIO_PIN_2
#define M2_In1_GPIO_Port GPIOD
#define M2_In2_Pin GPIO_PIN_3
#define M2_In2_GPIO_Port GPIOD
#define M3_In1_Pin GPIO_PIN_4
#define M3_In1_GPIO_Port GPIOD
#define M3_In2_Pin GPIO_PIN_5
#define M3_In2_GPIO_Port GPIOD
#define M4_In1_Pin GPIO_PIN_6
#define M4_In1_GPIO_Port GPIOD
#define M4_In2_Pin GPIO_PIN_7
#define M4_In2_GPIO_Port GPIOD
#define M2_EncoderB_Pin GPIO_PIN_3
#define M2_EncoderB_GPIO_Port GPIOB
#define M3_EncoderA_Pin GPIO_PIN_4
#define M3_EncoderA_GPIO_Port GPIOB
#define M3_EncoderB_Pin GPIO_PIN_5
#define M3_EncoderB_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
