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
#include "stm32f7xx_hal.h"

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
#define ADC_INPUT4_Pin GPIO_PIN_3
#define ADC_INPUT4_GPIO_Port GPIOF
#define ADC_INPUT5_Pin GPIO_PIN_4
#define ADC_INPUT5_GPIO_Port GPIOF
#define ADC_INPUT2_Pin GPIO_PIN_5
#define ADC_INPUT2_GPIO_Port GPIOF
#define ADC_INPUT0_Pin GPIO_PIN_6
#define ADC_INPUT0_GPIO_Port GPIOF
#define ADC_INPUT3_Pin GPIO_PIN_7
#define ADC_INPUT3_GPIO_Port GPIOF
#define ADC_INPUT6_Pin GPIO_PIN_8
#define ADC_INPUT6_GPIO_Port GPIOF
#define ADC_INPUT1_Pin GPIO_PIN_9
#define ADC_INPUT1_GPIO_Port GPIOF
#define ADC_INPUT7_Pin GPIO_PIN_10
#define ADC_INPUT7_GPIO_Port GPIOF
#define ADC_INPUT8_Pin GPIO_PIN_0
#define ADC_INPUT8_GPIO_Port GPIOC
#define ADC_INPUT9_Pin GPIO_PIN_1
#define ADC_INPUT9_GPIO_Port GPIOC
#define ADC_INPUT11_Pin GPIO_PIN_2
#define ADC_INPUT11_GPIO_Port GPIOC
#define ADC_INPUT15_Pin GPIO_PIN_3
#define ADC_INPUT15_GPIO_Port GPIOC
#define ADC_INPUT14_Pin GPIO_PIN_0
#define ADC_INPUT14_GPIO_Port GPIOA
#define ADC_INPUT10_Pin GPIO_PIN_1
#define ADC_INPUT10_GPIO_Port GPIOA
#define ADC_INPUT13_Pin GPIO_PIN_2
#define ADC_INPUT13_GPIO_Port GPIOA
#define ADC_INPUT12_Pin GPIO_PIN_3
#define ADC_INPUT12_GPIO_Port GPIOA
#define ADC_INPUT19_Pin GPIO_PIN_4
#define ADC_INPUT19_GPIO_Port GPIOA
#define ADC_INPUT18_Pin GPIO_PIN_5
#define ADC_INPUT18_GPIO_Port GPIOA
#define ADC_INPUT17_Pin GPIO_PIN_6
#define ADC_INPUT17_GPIO_Port GPIOA
#define ADC_INPUT16_Pin GPIO_PIN_7
#define ADC_INPUT16_GPIO_Port GPIOA
#define SCL_Pin GPIO_PIN_14
#define SCL_GPIO_Port GPIOF
#define SDA_Pin GPIO_PIN_15
#define SDA_GPIO_Port GPIOF
#define SHAKE_INT_Pin GPIO_PIN_0
#define SHAKE_INT_GPIO_Port GPIOG
#define EN13_Pin GPIO_PIN_13
#define EN13_GPIO_Port GPIOE
#define EN12_Pin GPIO_PIN_14
#define EN12_GPIO_Port GPIOE
#define EN19_Pin GPIO_PIN_15
#define EN19_GPIO_Port GPIOE
#define EN18_Pin GPIO_PIN_10
#define EN18_GPIO_Port GPIOB
#define EN17_Pin GPIO_PIN_11
#define EN17_GPIO_Port GPIOB
#define EN16_Pin GPIO_PIN_12
#define EN16_GPIO_Port GPIOB
#define DIA_EN_Pin GPIO_PIN_14
#define DIA_EN_GPIO_Port GPIOB
#define SEL2_Pin GPIO_PIN_15
#define SEL2_GPIO_Port GPIOB
#define SEL1_Pin GPIO_PIN_8
#define SEL1_GPIO_Port GPIOD
#define EN11_Pin GPIO_PIN_10
#define EN11_GPIO_Port GPIOD
#define EN10_Pin GPIO_PIN_11
#define EN10_GPIO_Port GPIOD
#define EN15_Pin GPIO_PIN_12
#define EN15_GPIO_Port GPIOD
#define LATCH_Pin GPIO_PIN_4
#define LATCH_GPIO_Port GPIOG
#define EN14_Pin GPIO_PIN_7
#define EN14_GPIO_Port GPIOC
#define EN9_Pin GPIO_PIN_9
#define EN9_GPIO_Port GPIOA
#define EN8_Pin GPIO_PIN_10
#define EN8_GPIO_Port GPIOA
#define EN7_Pin GPIO_PIN_11
#define EN7_GPIO_Port GPIOA
#define EN6_Pin GPIO_PIN_12
#define EN6_GPIO_Port GPIOA
#define EN1_Pin GPIO_PIN_15
#define EN1_GPIO_Port GPIOA
#define EN0_Pin GPIO_PIN_10
#define EN0_GPIO_Port GPIOC
#define EN3_Pin GPIO_PIN_11
#define EN3_GPIO_Port GPIOC
#define EN2_Pin GPIO_PIN_12
#define EN2_GPIO_Port GPIOC
#define EN5_Pin GPIO_PIN_0
#define EN5_GPIO_Port GPIOD
#define EN4_Pin GPIO_PIN_1
#define EN4_GPIO_Port GPIOD
#define ADC_LED_Pin GPIO_PIN_9
#define ADC_LED_GPIO_Port GPIOG
#define STATUS_LED_Pin GPIO_PIN_10
#define STATUS_LED_GPIO_Port GPIOG
#define FAULT_LED_Pin GPIO_PIN_11
#define FAULT_LED_GPIO_Port GPIOG
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
