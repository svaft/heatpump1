/* USER CODE BEGIN Header */
/**
	******************************************************************************
	* @file           : main.c
	* @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "can.h"
#include "crc.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "slip.h"
// C program to implement Quick Sort Algorithm 
#include <stdio.h> 
#include <stdlib.h> 
#include <math.h>
#include "string.h"
#include "onewire.h"
#include "ds18b20.h"
#include "OneWire1.h"
#include "string.h"
#include "llist.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

config_t config;

// CAN bus
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8] = {0,};
uint8_t RxData[8] = {0,};
uint32_t TxMailbox = 0;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);  
    }
}
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	
    uint32_t er = HAL_CAN_GetError(hcan);
 //   sprintf(trans_str,"ER CAN %lu %08lX", er, er);
  //  HAL_UART_Transmit(&huart1, (uint8_t*)trans_str, strlen(trans_str), 100);
}

void canSend(){
	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0);
	if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)	{
//					HAL_UART_Transmit(&huart1, (uint8_t*)"ER SEND\n", 8, 100);
	}
}




//DS18B20 temperatureSensor;
RTC_TimeTypeDef sTime = {0};
RTC_DateTypeDef sDate = {0};
char trans_str[64] = {0,};


llist *scheduler_list;
scheduler_t scheduler[100];
uint8_t dummyAlert = 0;
int quickDayChange = 0;

int sch_pos = 0; //
uint8_t sch_size = 50;

state_t state;

uint8_t uart_dma_buf_tx[STATE_DMA_SIZE] = {0};
uint8_t uart_dma_buf_rx[RX_BUFER_MAX_SIZE] = {0};

uint8_t buf[64] = {0};
uint8_t separator[] = " . ";
uint8_t new_line[] = "\r\n";
uint8_t start_text[] = "Start scanning I2C: \r\n";
uint8_t end_text[] = "\r\nStop scanning";
#define I2C_ESP32_ADDRESS   0x55

int cnt_flow1 = 0;
int cnt_flow2 = 0;

esp2stm_i2c_status1_t 	i2c_rxStatus;
esp2stm_i2c_scheduler_t i2c_rxScheduler;

uint8_t i2c_tx_end = 0;
uint8_t i2c_rx_end = 0;


/* adc vars */ 
uint16_t ADC_Raw[ADC_CHANNELS_NUM_RAW];                                       
//uint16_t adcData[ADC_CHANNELS_NUM];
float adcVoltage[ADC_CHANNELS_NUM];

extern float Temp[MAXDEVICES_ON_THE_BUS];
uint8_t adc_complete = 0;

TaskHandle_t xTask1 = NULL, xTask2 = NULL;
extern osMessageQueueId_t myQueue01Handle;
extern osSemaphoreId_t txBinSemaphoreHandle;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define I2C_ESP32_GET_STATUS 1
#define I2C_ESP32_GET_SCHEDULE 2



//uint8_t getEsp32cmd(uint8_t cmd_in, uint8_t *pData, uint16_t Size){
//	cmd_t cmd;
//	HAL_StatusTypeDef hStat;
//	hStat = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)17<<1, 10,100);
//	if(hStat == HAL_OK){
//		cmd.cmd = cmd_in;
//		cmd.header.header = ':';
//		cmd.header.length = sizeof(cmd_t);
//		cmd.header.datatype = COMM_STRUCT_TYPE_CMD;

//		cmd.end.cr = '\r';
//		cmd.end.cn = '\n';

//		if(HAL_I2C_Master_Transmit_DMA(&hi2c1, (uint16_t)17<<1, (uint8_t *) &cmd, CMD_DMA_SIZE)!= HAL_OK){
//			config.error = 1;
//			Error_Handler();
//		}
//	} else if(hStat == HAL_BUSY){
//			config.error = 2;
//		Error_Handler();
//	} 
//	
//	int rxcnt =0;
//	int af_cnt =0;
//	while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY){af_cnt++;}
//	if(i2c_tx_end == 1) {
//		i2c_rx_end = 0;
//		/*##-4- Put I2C peripheral in reception process ############################*/  
//		do
//		{
//			if(HAL_I2C_Master_Receive_DMA(&hi2c1, (uint16_t)17<<1, pData, Size) != HAL_OK){
//				config.error = 3;
//				Error_Handler();
//			}
//			af_cnt++;
//			
//		}
//		while(HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF);

//		while(i2c_rx_end != 2){
//			rxcnt++;
//			HAL_Delay(1);
//		}
//		return 1;
//	}
//	return 0;
//}




//float adcVoltage[3];
uint16_t adc_cnt=0;
uint16_t t1sum_tmp=0;
uint16_t t2sum_tmp=0;

uint16_t t1sum=0;
uint16_t t2sum=0;
float steinhart = 0;

int comp (const void * a, const void * b){
	uint32_t *s1 = (uint32_t *)a;
	uint32_t *s2 = (uint32_t *)b;
	if(*s1 > *s2) {
			return 1;
	}
	else if(*s1 == *s2) {
			return 0;
	}
	else {
			return -1;
	}
}

int get_rand(int lower, int upper){
	return  (rand() % (upper - lower + 1)) + lower;
}




int i2ccnt = 0;
int i2ccntRX = 0;
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
	i2c_tx_end = 1;
	i2ccnt++;
}
uint8_t aRxBuffer[4];

uint8_t i2c_esp32_RxBuffer[100];


uint8_t aRxBufferCp[4];

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
//	uint32_t *rxBufPtr = (uint32_t *)&aRxBuffer;
//	if(*(uint32_t *)&aRxBuffer != 0)
//		*(uint32_t *)&aRxBufferCp = *(uint32_t *)&aRxBuffer;
	i2c_rx_end = 1;
	i2ccntRX++;
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *I2cHandle)
{
	/** Error_Handler() function is called when error occurs.
		* 1- When Slave doesn't acknowledge its address, Master restarts communication.
		* 2- When Master doesn't acknowledge the last data transferred, Slave doesn't care in this example.
		*/
	if (HAL_I2C_GetError(I2cHandle) != HAL_I2C_ERROR_AF)
	{
		Error_Handler();
	}
}



uint32_t RTC_ReadTimeCounter(RTC_HandleTypeDef *hrtc)
{
	uint16_t high1 = 0U, high2 = 0U, low = 0U;
	uint32_t timecounter = 0U;

	high1 = READ_REG(hrtc->Instance->CNTH & RTC_CNTH_RTC_CNT);
	low   = READ_REG(hrtc->Instance->CNTL & RTC_CNTL_RTC_CNT);
	high2 = READ_REG(hrtc->Instance->CNTH & RTC_CNTH_RTC_CNT);

	if (high1 != high2)
	{
		/* In this case the counter roll over during reading of CNTL and CNTH registers,
			 read again CNTL register then return the counter value */
		timecounter = (((uint32_t) high2 << 16U) | READ_REG(hrtc->Instance->CNTL & RTC_CNTL_RTC_CNT));
	}
	else
	{
		/* No counter roll over during reading of CNTL and CNTH registers, counter
			 value is equal to first value of CNTL and CNTH */
		timecounter = (((uint32_t) high1 << 16U) | low);
	}
	return timecounter;
}




uint8_t tx_end =0;
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	tx_end = 1;
	osSemaphoreRelease(txBinSemaphoreHandle);
}

uint8_t rx_end = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
 	rx_end = 1;
}



static const uint8_t aucCRCHi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40
};

static const uint8_t aucCRCLo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7,
    0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,
    0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9,
    0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
    0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
    0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D,
    0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
    0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF,
    0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1,
    0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
    0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB,
    0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
    0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
    0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97,
    0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E,
    0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89,
    0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83,
    0x41, 0x81, 0x80, 0x40
};

uint16_t usMBCRC16( uint8_t * pucFrame, uint16_t usLen ){
    uint8_t           ucCRCHi = 0xFF;
    uint8_t           ucCRCLo = 0xFF;
    int             iIndex;

    while( usLen-- ) {
        iIndex = ucCRCLo ^ *( pucFrame++ );
        ucCRCLo = ( uint8_t )( ucCRCHi ^ aucCRCHi[iIndex] );
        ucCRCHi = aucCRCLo[iIndex];
    }
    return ( uint16_t )( ucCRCHi << 8 | ucCRCLo );
}




void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size){
	header_t *h_ref = (header_t *)&uart_dma_buf_rx;
	if(Size < 8 || h_ref->header != ':'){ // error? handle?
		return;
	}
	if(	h_ref->length != Size){
		return;
	} else { // check CRC here
		end_t *end_ref = (end_t *)(uart_dma_buf_rx+Size-sizeof(end_t));
		if(usMBCRC16((uint8_t *)uart_dma_buf_rx,(Size)-sizeof(end_t)) != end_ref->crc){
			return;
		}
	}
	// header and crc is valid, cast received type:
	switch(h_ref->datatype){
		case COMM_STRUCT_TYPE_STATUS:
			if(Size == sizeof(esp2stm_i2c_status1_t)){
				memcpy((uint8_t *)&i2c_rxStatus, &uart_dma_buf_rx, sizeof(esp2stm_i2c_status1_t));
				osMessageQueuePut(myQueue01Handle,&i2c_rxStatus,0,0);
			}
			break;
		default: // unknown data, error
			break;
	}
	HAL_UARTEx_ReceiveToIdle_DMA(&huart3, uart_dma_buf_rx, RX_BUFER_MAX_SIZE);
	__HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
}


int aaa=0;
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
	aaa++;
 //       snprintf(trans_str, 63, "ALARM\n");
//        HAL_UART_Transmit(&huart1, (uint8_t*)trans_str, strlen(trans_str), 1000);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance == ADC1){
		HAL_ADC_Stop_DMA(&hadc1);
		uint32_t adcDataTmp[ADC_CHANNELS_NUM];
		memset(adcDataTmp, 0, ADC_CHANNELS_NUM*4);
		for (uint16_t ich = 0; ich < ADC_CHANNELS_NUM_RAW; ich+=ADC_CHANNELS_NUM){
			for (uint8_t a = 0; a < ADC_CHANNELS_NUM; a++){
				adcDataTmp[a]+=ADC_Raw[ich+a];
			}
		}
		for (uint8_t a = 0; a < ADC_CHANNELS_NUM; a++){
			state.temp[a]=adcDataTmp[a]>>ADC_MA_SHIFT;
		}
	}
	adc_complete = 1;
}


void handleScheduler(){
	if(config.state_mask.scheduler == 0)
		return;
	if(sch_size > 0){
		RTC_AlarmTypeDef salarmstructure;
		salarmstructure.Alarm = RTC_ALARM_A;
		HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN); // RTC_FORMAT_BIN , RTC_FORMAT_BCD
		HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN); // RTC_FORMAT_BIN , RTC_FORMAT_BCD
		int dayToFind = sDate.WeekDay;
		int nextDay = sDate.WeekDay == 6? 0 : sDate.WeekDay+1;
		int minutesFromDaystart = sTime.Hours*60+sTime.Minutes;

		for(int a = 0; a < sch_size;a++){
			int min2 = scheduler[a].hour * 60 + scheduler[a].min;
			if(
				(scheduler[a].weekday == dayToFind && min2 > minutesFromDaystart) 
				|| 
				(scheduler[a].weekday == nextDay && min2 < minutesFromDaystart)
			){
				sch_pos = a;
				dummyAlert = 0;
				salarmstructure.AlarmTime.Hours   = scheduler[sch_pos].hour;
				salarmstructure.AlarmTime.Minutes = scheduler[sch_pos].min;
				salarmstructure.AlarmTime.Seconds = 0;
				HAL_RTC_SetAlarm_IT(&hrtc, &salarmstructure,RTC_FORMAT_BIN);
				return;
			}
		}
		salarmstructure.AlarmTime.Hours   = sTime.Hours;
		salarmstructure.AlarmTime.Minutes = sTime.Minutes;
		salarmstructure.AlarmTime.Seconds = 0;
		salarmstructure.Alarm = RTC_ALARM_A;
		dummyAlert = 1;
		HAL_RTC_SetAlarm_IT(&hrtc, &salarmstructure,RTC_FORMAT_BIN);
	}
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	 if(GPIO_Pin == GPIO_PIN_9)
	 {
		 state.flow1++;
		 cnt_flow1++;
	 }
	 if(GPIO_Pin == GPIO_PIN_12) 
	 {
		 state.flow2++;
		 cnt_flow2++;
	 }
}



int aa = 0;
int bb = 0;
int numcmp(void *a, void *b){
	if (*(int *)a < *(int *)b)
		return -1;
	if (*(int *)a > *(int *)b)
			return 1;
	return 0;
}
uint32_t i2c_error_count = 0;
int changeGPIOstate(uint8_t on, uint8_t off){
	if(on)
		*(uint8_t *)&state.gpio_mask &= (~on);
	if(off)
		*(uint8_t *)&state.gpio_mask |= off;

  while(HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)0x20<<1, (uint8_t*)&state.gpio_mask, 1, 1000)!= HAL_OK){
    uint32_t i2c_error = HAL_I2C_GetError(&hi2c1);
		if(i2c_error_count++ > 10)
			return pdFALSE;
		if (i2c_error != HAL_I2C_ERROR_AF){
			return pdFALSE;
//      Error_Handler();
    }
  }
	i2c_error_count = 0;
	return pdTRUE;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
//can init
TxHeader.StdId = 0x0378;
TxHeader.ExtId = 0;
TxHeader.RTR = CAN_RTR_DATA; //CAN_RTR_REMOTE
TxHeader.IDE = CAN_ID_STD;   // CAN_ID_EXT
TxHeader.DLC = 8;
TxHeader.TransmitGlobalTime = 0;
for(uint8_t i = 0; i < 8; i++)
{
    TxData[i] = (i + 10);
}	
	
	
	
	
	config.setTempFloor = defaultTempSet0;
	config.setTempWater = defaultTempSet1;
	state.setT.set1 = config.setTempFloor;
	state.setT.set2 = config.setTempWater;
	
	quickDayChange = 20;
	sch_size = 3;

	scheduler[0].weekday = 5;
	scheduler[0].hour = 15;
	scheduler[0].min = 21;
	scheduler[0].temp = 1500;
	scheduler[0].channel = 0;

	scheduler[1].weekday = 5;
	scheduler[1].hour = 15;
	scheduler[1].min = 21;
	scheduler[1].temp = 70;
	scheduler[1].channel = 1;

	scheduler[2].weekday = 5;
	scheduler[2].hour = 15;
	scheduler[2].min = 22;
	scheduler[2].temp = 150;
	scheduler[2].channel = 1;

	state.header.header = ':';
	state.header.datatype = COMM_STRUCT_TYPE_STATE;
	state.header.address = COMM_ADDR_GATEWAY;
	state.header.length = sizeof(state_t); // content length

	state.end.cr = '\r';
	state.end.cn = '\n';

//	fsm_state_t fsm = FSM_START;
	state.state_mask.fsmState = FSM_RESTART;
	*(uint8_t *)&state.gpio_mask=0xff; // set all GPIO pins to 1(turn off all relays)

	state.flow1 = 0;
	state.flow2 = 0;
	state.duration = 0;

	aa = sizeof(state_t);
	
//	uint32_t from = 1;
//	(*(scheduler_t *)&from).weekday = 7; // uint to struct
//	*(uint32_t *)&scheduler = 0xffffffff; // cast struct to uint




  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */


  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_RTC_Init();
  MX_IWDG_Init();
  MX_TIM1_Init();
  MX_CAN_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
//	changeGPIOstate(0, port20gpioAll);
	


	HAL_CAN_Start(&hcan);
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR | CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE);
	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0);
	if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)	{
//					HAL_UART_Transmit(&huart1, (uint8_t*)"ER SEND\n", 8, 100);
	}
	
	HAL_Delay(500);
/*
	1. reset i2c gpio
	
*/
	
//	HAL_Delay(1000);
	
	#define I2C_ESP32_GET_STATUS 		1
	#define I2C_ESP32_GET_SCHEDULE 	2

	int rereadCnt = 0;
	int rereadCnt2 = 0;
	

/*
	do{
		if (getEsp32cmd(I2C_ESP32_GET_SCHEDULE, (uint8_t *)&i2c_rxScheduler, sizeof(esp2stm_i2c_scheduler_t))){

		}
	//	HAL_Delay(1);
		rereadCnt++;
	} while(i2c_rxScheduler.header != 0xdeadbeef);

	//HAL_Delay(300);
	do{
	//for(int a = 0; a < 2; a++){
		if (getEsp32cmd(I2C_ESP32_GET_STATUS, (uint8_t *)&i2c_rxStatus, sizeof(esp2stm_i2c_status1_t))){

		}
	//	HAL_Delay(2);
	//	HAL_Delay(1);
		rereadCnt2++;
	} while(i2c_rxStatus.header != 0xdeadbe1f);
*/
//	HAL_UART_Transmit(&huart3,(uint8_t *) "AT+BAUD8\r\n", 10, 128);
//	HAL_UART_Receive_DMA(&huart3,(uint8_t *)&i2c_rxStatus,sizeof(esp2stm_i2c_status1_t));
//	HAL_Delay(1000);	

	sDate.Date 		= i2c_rxStatus.ts.Date;
	sDate.Month 	= i2c_rxStatus.ts.Month;
//	sDate.WeekDay = i2c_esp32_RxBuffer[2];
	sDate.Year 		= i2c_rxStatus.ts.Year+24;
	sTime.Hours 	= i2c_rxStatus.ts.Hours;
	sTime.Minutes = i2c_rxStatus.ts.Seconds;


	HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN); // RTC_FORMAT_BIN , RTC_FORMAT_BCD
	HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN); // RTC_FORMAT_BIN , RTC_FORMAT_BCD

	int timeCounter = RTC_ReadTimeCounter(&hrtc);

	handleScheduler();

//	HAL_Delay(1000);


	HAL_UART_Transmit(&huart3,(uint8_t *) "start controller\r\n", 18, 128);

	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&ADC_Raw, ADC_CHANNELS_NUM_RAW);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
		 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
