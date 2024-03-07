/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct {
	uint8_t header[4];
	uint16_t temp[10];
	uint32_t flow1;
	uint32_t flow2;
	uint32_t duration;
	uint8_t state_mask[2];
	uint8_t end[2];	
} state_t;
#define STATE_DMA_SIZE sizeof(state_t)

typedef struct {
	uint8_t header[4];
	uint32_t flow1;
	uint32_t flow2;
	uint32_t duration;
	uint8_t dummy[17];
	uint8_t end[2];
} flow_t;

typedef struct {
		uint32_t channel : 1;
		uint32_t tempDeltaMode : 1; // if 1 temerature is relative to default value(signed+- value), if 0 - absolute unsigned mode
		uint32_t reserved : 4; // reserved for some new features
		uint32_t temp : 12; // 12bit adc value for corresponding channel or delta if tempDeltaMode=1
		uint32_t min : 6;
		uint32_t hour : 5;
		uint32_t weekday : 3; // 0 - Sunday, 1 monday, ...  if weekday = 7 - all days timer(todo)? 
	} scheduler_t;


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
void handleScheduler();
uint32_t RTC_ReadTimeCounter(RTC_HandleTypeDef *hrtc);

	
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ADC_CHANNELS_NUM 10
#define LED_Pin_Pin GPIO_PIN_13
#define LED_Pin_GPIO_Port GPIOC
#define ONE_WIRE_LINE_Pin GPIO_PIN_9
#define ONE_WIRE_LINE_GPIO_Port GPIOA
#define EEV1_DIR_Pin GPIO_PIN_10
#define EEV1_DIR_GPIO_Port GPIOA
#define EEV1_STEP_Pin GPIO_PIN_11
#define EEV1_STEP_GPIO_Port GPIOA
#define HEATER_Pin GPIO_PIN_12
#define HEATER_GPIO_Port GPIOA
#define CIRCULATION_PUMP2_Pin GPIO_PIN_3
#define CIRCULATION_PUMP2_GPIO_Port GPIOB
#define CIRCULATION_PUMP1_Pin GPIO_PIN_4
#define CIRCULATION_PUMP1_GPIO_Port GPIOB
#define WATER_PUMP_Pin GPIO_PIN_8
#define WATER_PUMP_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define ADC_MA_SHIFT   2
#define ADC_CHANNELS_NUM_RAW   ADC_CHANNELS_NUM<<ADC_MA_SHIFT

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
