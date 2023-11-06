/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#define in2_1_Pin GPIO_PIN_2
#define in2_1_GPIO_Port GPIOC
#define in2_2_Pin GPIO_PIN_3
#define in2_2_GPIO_Port GPIOC
#define Encoder3A_Pin GPIO_PIN_0
#define Encoder3A_GPIO_Port GPIOA
#define Encoder3B_Pin GPIO_PIN_1
#define Encoder3B_GPIO_Port GPIOA
#define hc05_TX_Pin GPIO_PIN_2
#define hc05_TX_GPIO_Port GPIOA
#define hc05_RX_Pin GPIO_PIN_3
#define hc05_RX_GPIO_Port GPIOA
#define Encoder2A_Pin GPIO_PIN_6
#define Encoder2A_GPIO_Port GPIOA
#define Encoder2B_Pin GPIO_PIN_7
#define Encoder2B_GPIO_Port GPIOA
#define in1_2_Pin GPIO_PIN_0
#define in1_2_GPIO_Port GPIOB
#define in1_1_Pin GPIO_PIN_1
#define in1_1_GPIO_Port GPIOB
#define in3_1_Pin GPIO_PIN_12
#define in3_1_GPIO_Port GPIOB
#define in3_2_Pin GPIO_PIN_13
#define in3_2_GPIO_Port GPIOB
#define in4_1_Pin GPIO_PIN_14
#define in4_1_GPIO_Port GPIOB
#define in4_2_Pin GPIO_PIN_15
#define in4_2_GPIO_Port GPIOB
#define Encoder4A_Pin GPIO_PIN_6
#define Encoder4A_GPIO_Port GPIOC
#define Encoder4B_Pin GPIO_PIN_7
#define Encoder4B_GPIO_Port GPIOC
#define PWM1_Pin GPIO_PIN_8
#define PWM1_GPIO_Port GPIOA
#define PWM2_Pin GPIO_PIN_9
#define PWM2_GPIO_Port GPIOA
#define PWM3_Pin GPIO_PIN_10
#define PWM3_GPIO_Port GPIOA
#define PWM4_Pin GPIO_PIN_11
#define PWM4_GPIO_Port GPIOA
#define reset_Pin GPIO_PIN_12
#define reset_GPIO_Port GPIOA
#define Encoder1A_Pin GPIO_PIN_15
#define Encoder1A_GPIO_Port GPIOA
#define jy62_TX_Pin GPIO_PIN_10
#define jy62_TX_GPIO_Port GPIOC
#define jy62_RX_Pin GPIO_PIN_11
#define jy62_RX_GPIO_Port GPIOC
#define Encoder1B_Pin GPIO_PIN_3
#define Encoder1B_GPIO_Port GPIOB
#define zigbee_TX_Pin GPIO_PIN_6
#define zigbee_TX_GPIO_Port GPIOB
#define zigbee_RX_Pin GPIO_PIN_7
#define zigbee_RX_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
