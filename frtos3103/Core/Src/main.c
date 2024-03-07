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
#include "dma.h"
#include "i2c.h"
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


int defaultTempSet0 = 2400;
int defaultTempSet1 = 200;

int tempSet0 = 0;
int tempSet1 = 0;

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
uint8_t uart_dma_buf_rx[8] = {0};

uint8_t buf[64] = {0};
uint8_t separator[] = " . ";
uint8_t new_line[] = "\r\n";
uint8_t start_text[] = "Start scanning I2C: \r\n";
uint8_t end_text[] = "\r\nStop scanning";
#define I2C_ESP32_ADDRESS   0x55

int cnt1 = 0;
int cnt2 = 0;
int cnt_flow1 = 0;
int cnt_flow2 = 0;


/* adc vars */ 
uint16_t ADC_Raw[ADC_CHANNELS_NUM_RAW];                                       
//uint16_t adcData[ADC_CHANNELS_NUM];
float adcVoltage[ADC_CHANNELS_NUM];

extern float Temp[MAXDEVICES_ON_THE_BUS];


TaskHandle_t xTask1 = NULL, xTask2 = NULL;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
	i2ccnt++;
}
uint8_t i2c_rx_end = 0;
uint8_t aRxBuffer[4];

uint8_t aRxBufferCp[4];

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
	uint32_t *rxBufPtr = (uint32_t *)&aRxBuffer;
	if(*(uint32_t *)&aRxBuffer != 0)
		*(uint32_t *)&aRxBufferCp = *(uint32_t *)&aRxBuffer;
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
}

uint8_t rx_end = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	tx_end = 1;
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
  
  if(hadc->Instance == ADC1)
  {
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
		
/*
		
		for (uint8_t i = 0; i < ADC_CHANNELS_NUM; i++)
    {
			#define NTC_UP_R 9910.0f
// constants of Steinhart-Hart equation
			#define A 0.001024126754f
			#define B 0.0002527981762f
			#define C 0.000000001961346490f
			float Ntc_Tmp = 0;
			uint16_t Ntc_R;
			Ntc_R = ((NTC_UP_R)/((4095.0/adcData[i]) - 1));
	// temp
			float Ntc_Ln = log(Ntc_R);
			//calc. temperature
			Ntc_Tmp = (1.0/(A + B*Ntc_Ln + C*Ntc_Ln*Ntc_Ln*Ntc_Ln)) - 273.15;

      adcVoltage[i] = Ntc_Tmp; //adcData[i] * 3.3 / 4095;
    }
	*/	
		
		/*
		if(adc_cnt++<16){
			t1sum_tmp += ADC_Raw[0];
		} else {
			t1sum = t1sum_tmp >> 4; 

			float average = 4095.0 / t1sum - 1;
			average = NTC_UP_R / average;
			#define BCOEFFICIENT 3950
			#define THERMISTORNOMINAL 10000
			steinhart = (BCOEFFICIENT * 298.15 ) / (BCOEFFICIENT + (298.15 * log(average / THERMISTORNOMINAL)))-273.15;

			t1sum_tmp = 0;
			adc_cnt = 0;
		}
		*/
		
		
//		ADC_Raw[0] = HAL_ADC_GetValue(&hadc1);
		
//    for (uint8_t i = 0; i < 3; i++)
//    {
//      adcVoltage[i] = ADC_Raw[i] * 3.3 / 4095;
//    }
  }
}


void handleScheduler(){
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
//		 state.flow1++;
   }
   if(GPIO_Pin == GPIO_PIN_15) 
   {
//		 state.flow2++;
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


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	/*
	sch_size = 5;
	// init random scheduler data
	for(int a = 0; a<sch_size;a++){
		scheduler[a].weekday = get_rand(0,6);
		scheduler[a].hour = get_rand(0,23);
		scheduler[a].min = get_rand(0,59);
		scheduler[a].temp = get_rand(1000,2000);
	}
	// sort random data:
  qsort((uint32_t *)&scheduler, sch_size, sizeof(scheduler[0]), comp);	
*/
	quickDayChange = 20;
	tempSet0 = 2000;
	tempSet1 = 150;
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

	state.header[0] = 0xde;
	state.header[1] = 0xad;
	state.header[2] = 0;
	state.header[3] = sizeof(state_t)-4;

	state.state_mask[0] = 0;
	state.state_mask[1] = 0;

	state.flow1 = 0;
	state.flow2 = 0;
	state.duration = 0;

	
	state.end[0] = '\r';
	state.end[1] = '\n';


	
	
//	uint32_t from = 1;
//	(*(scheduler_t *)&from).weekday = 7; // uint to struct
//	*(uint32_t *)&scheduler = 0xffffffff; // cast struct to uint

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	state.header[0] = 0xde;
	state.header[1] = 0xad;
	state.header[2] = 0;
	state.header[3] = sizeof(state_t)-6;

	state.state_mask[0] = 1;
	state.state_mask[1] = 2;

		state.flow1 = 5;
		state.flow2 = 6;
		state.duration = 7;

	
	state.end[0] = '\r';
	state.end[1] = '\n';
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
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

	sDate.Date = 1;
	sDate.Month = 03;
	sDate.WeekDay = 5;
	sDate.Year = 24;
	sTime.Hours = 15;
	sTime.Minutes = 20;


	HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN); // RTC_FORMAT_BIN , RTC_FORMAT_BCD
	HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN); // RTC_FORMAT_BIN , RTC_FORMAT_BCD

	int timeCounter = RTC_ReadTimeCounter(&hrtc);

	handleScheduler();
		
		
		      while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
      {
      }


//		HAL_I2C_Master_Transmit_DMA(&hi2c1,17<<1,(uint8_t *) &state,STATE_DMA_SIZE);
//		HAL_I2C_Master_Transmit(&hi2c1,17<<1,(uint8_t *) &state,STATE_DMA_SIZE, 10000);
   int i2cerr = HAL_I2C_GetError(&hi2c1);

			
  HAL_UART_Transmit(&huart3,(uint8_t *) "start controller\r\n", 18, 128);
	memcpy(uart_dma_buf_tx,"test1\r\n",7);
	HAL_UART_Transmit_DMA(&huart3, uart_dma_buf_tx, STATE_DMA_SIZE);

//	HAL_UART_Receive_DMA();
//	get_ROMid();

//	get_Temperature();
//	HAL_Delay(2000);

/*




DS18B20_Init(&temperatureSensor, &huart1);

  DS18B20_InitializationCommand(&temperatureSensor);
  DS18B20_ReadRom(&temperatureSensor);
  DS18B20_ReadScratchpad(&temperatureSensor);

  uint8_t settings[3];
  settings[0] = temperatureSensor.temperatureLimitHigh;
  settings[1] = temperatureSensor.temperatureLimitLow;
  settings[2] = DS18B20_12_BITS_CONFIG;

  DS18B20_InitializationCommand(&temperatureSensor);
  DS18B20_SkipRom(&temperatureSensor);
  DS18B20_WriteScratchpad(&temperatureSensor, settings);
*/

	
//	adcData[7] = 2000;
	slipBuffer_t slip_buf;
	uint8_t input_buffer[100];

	
	
	init_slip_buffer((slipBuffer_t* )&slip_buf, (uint8_t* )input_buffer, 100);

	uint8_t ret;

//  HAL_Delay(1000);

	uint8_t regData = 0;
	uint8_t regAddress = 1;

	HAL_ADCEx_Calibration_Start(&hadc1);
//	HAL_ADC_Start(&hadc1);
//  HAL_ADC_PollForConversion(&hadc1, 1000);
//  int digital_result = HAL_ADC_GetValue(&hadc1);

	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&ADC_Raw, ADC_CHANNELS_NUM_RAW);
//	DMA_PDATAALIGN_BYTE


//  HAL_UART_Transmit(&huart3, start_text, sizeof(start_text), 128);
	
//	HAL_I2C_Master_Transmit(&hi2c1, (I2C_ESP32_ADDRESS << 1), &regAddress, 1,  1000);
//	HAL_I2C_Master_Receive(&hi2c1, (I2C_ESP32_ADDRESS << 1), (uint8_t *)&buf, 64,  1000);
/*
	
// процедура сканирования
  //HAL_UART_Transmit(&huart1, start_text, sizeof(start_text), 128);
  for( uint8_t i=1; i<128; i++ ){
      ret = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i<<1), 3, 5);

      if ( ret != HAL_OK ){ // нет ответа от адреса
//          HAL_UART_Transmit(&huart1, separator, sizeof(separator), 128);
      }

      else if(ret == HAL_OK){ // есть ответ
				buf[0] = i;
//          sprintf(buf, "0x%X", i);
//          HAL_UART_Transmit(&huart1, buf, sizeof(buf), 128);
      }

//      if( i == 15 ){
//    	  i = 0;
//          HAL_UART_Transmit(&huart1, new_line, sizeof(new_line), 128);
//      } else
//    	  row ++;
  }
//  HAL_UART_Transmit(&huart1, end_text, sizeof(end_text), 128);
	*/
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
	/*	
    DS18B20_InitializationCommand(&temperatureSensor);
    DS18B20_SkipRom(&temperatureSensor);
    DS18B20_ConvertT(&temperatureSensor, DS18B20_DATA);

    DS18B20_InitializationCommand(&temperatureSensor);
    DS18B20_SkipRom(&temperatureSensor);
    DS18B20_ReadScratchpad(&temperatureSensor);
*/
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
