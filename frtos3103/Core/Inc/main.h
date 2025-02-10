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
	uint32_t header 	: 8;
	uint32_t datatype 	: 4;
	uint32_t reserverd 	: 4;
	uint32_t address 	: 8;
	uint32_t length 	: 8;
} header_t;

typedef struct {
	uint32_t crc 	: 16;
	uint32_t cr 	: 8;
	uint32_t cn 	: 8;
} end_t;


typedef struct {
	uint8_t compressor 			: 1; 
	uint8_t heater1 				: 1; 
	uint8_t heater2 				: 1; 
	uint8_t waterPump 			: 1; 
	uint8_t circulPump1 		: 1; 
	uint8_t circulPump2 		: 1; 
	uint8_t CrankcaseHeater : 1; 
	uint8_t reserved				: 1;
} gpioEx_t;

typedef struct {
		uint8_t fsmState 		: 4;
		uint8_t scheduler 	: 1; 
		uint8_t disabled 		: 1;
		uint8_t reserved 		: 2;
	} state_mask_t;

typedef struct {
		uint32_t channel : 1;
		uint32_t tempDeltaMode : 1; // if 1 temerature is relative to default value(signed+- value), if 0 - absolute unsigned mode
		uint32_t reserved : 4; // reserved for some new features
		uint32_t temp : 12; // 12bit adc value for corresponding channel or delta if tempDeltaMode=1
		uint32_t min : 6;
		uint32_t hour : 5;
		uint32_t weekday : 3; // 0 - Sunday, 1 monday, ...  if weekday = 7 - all days timer(todo)? 
	} scheduler_t;

typedef struct {
	uint32_t set1 	: 16;
	uint32_t set2 	: 16;
} setTempCompact_t;
	
	
typedef struct {
	header_t header;
	
	uint16_t temp[10];
	setTempCompact_t setT;
	uint32_t flow1;
	uint32_t flow2;
	uint32_t duration;
	
	state_mask_t state_mask;//8
	gpioEx_t gpio_mask; //8
	uint8_t reserved[2]; //16
	end_t end;
} state_t;
#define STATE_DMA_SIZE sizeof(state_t)

typedef struct {                                // object data type
  uint8_t cmd;
  uint16_t value;
} MSGQUEUE_OBJ_t;

typedef struct {
	header_t header;
	uint8_t cmd;
	uint8_t val[3];
	end_t end;	
} cmd_t;
#define CMD_DMA_SIZE sizeof(cmd_t)

typedef struct {
	uint32_t Seconds 	: 6;
	uint32_t Minutes 	: 6;
	uint32_t Hours 		: 5;
  uint32_t Date 		: 5;
	uint32_t Month 		: 4;
	uint32_t Year 		: 6; //2024 - 0, 2025 - 1, ...
} timestampCompact_t;





typedef struct {
	header_t header;
  timestampCompact_t ts;
  uint8_t cmd; // cmd from mqtt command topic
  int8_t status; // 0 init, 1 connected to mqtt, 2 disconnected(fail)
  uint16_t dummy;
	end_t end;
//  uint32_t schedule_len;
//  scheduler_t schedule[32];
//  uint32_t array[64];
} esp2stm_i2c_status1_t;

typedef enum {
	FSM_RESTART,
	FSM_RESTART_GPIO_MODULE_CONNECTING,
	FSM_RESTART_RS485_REMOTE_CONNECTING,
	FSM_CHECK_DEVICES_ERROR,
	FSM_MAIN_RESTRICT_WORKING,
	FSM_MAIN_UNRESTRICT_WORKING,
} fsm_state_t;

typedef struct {
	header_t header;
	uint16_t setTempFloor;
	int16_t setTempFloorHysteresis;
	uint16_t setTempWater;
	int16_t setTempWaterHysteresis;
	uint16_t coldSideFlowLPH;
	uint16_t hotSideFlowLPH;
	state_mask_t state_mask;
	
	uint8_t error;
	end_t end;	
} config_t;


/*
typedef struct {
	uint8_t header[4];
	uint32_t flow1;
	uint32_t flow2;
	uint32_t duration;
	uint8_t dummy[17];
	uint8_t end[2];
} flow_t;
*/

typedef struct {
  uint32_t header;
  uint32_t schedule_len;
  scheduler_t schedule[32];
} esp2stm_i2c_scheduler_t;


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
int changeGPIOstate(uint8_t on, uint8_t off);
uint16_t usMBCRC16( uint8_t * pucFrame, uint16_t usLen );
	void canSend();

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ADC_CHANNELS_NUM 10
#define LED_Pin_Pin GPIO_PIN_13
#define LED_Pin_GPIO_Port GPIOC
#define FLOW_SENSOR2_Pin GPIO_PIN_12
#define FLOW_SENSOR2_GPIO_Port GPIOB
#define FLOW_SENSOR2_EXTI_IRQn EXTI15_10_IRQn
#define ONE_WIRE_LINE_Pin GPIO_PIN_9
#define ONE_WIRE_LINE_GPIO_Port GPIOA
#define zeroCrossLine3_Pin GPIO_PIN_3
#define zeroCrossLine3_GPIO_Port GPIOB
#define zeroCrossLine3_EXTI_IRQn EXTI3_IRQn
#define FLOW_SENSOR1_Pin GPIO_PIN_9
#define FLOW_SENSOR1_GPIO_Port GPIOB
#define FLOW_SENSOR1_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */
#define ADC_MA_SHIFT   2
#define ADC_CHANNELS_NUM_RAW   ADC_CHANNELS_NUM<<ADC_MA_SHIFT

#define COMM_STRUCT_TYPE_STATUS 0x00
#define COMM_STRUCT_TYPE_STATE 0x01
#define COMM_STRUCT_TYPE_CMD 0x02


#define COMM_ADDR_GATEWAY 0x2
#define COMM_ADDR_MAIN_CONTROLLER 0x1

#define RX_BUFER_MAX_SIZE 512

#define defaultTempSet0 1825
#define defaultTempSet1 707

#define gpio0 1<<0
#define gpio1 1<<1
#define gpio2 1<<2
#define gpio3 1<<3
#define gpio4 1<<4
#define gpio5 1<<5
#define gpio6 1<<6
#define gpio7 1<<7
#define port20gpioAll 0xff


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
