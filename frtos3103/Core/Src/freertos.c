/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c.h"
#include "adc.h"
#include "ntc_table.h"
#include "slip.h"
#include <math.h>
#include "onewire.h"
#include "ds18b20.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
		#define I2C_ESP32_ADDRESS   0x55

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

uint32_t rtc_conter, rtc_conter_delta=0;


/* Prototypes of the two tasks created by main(). */
static void prvTask1( void *pvParameters );
static void prvTask2( void *pvParameters );

/* Handles for the tasks create by main(). */
extern TaskHandle_t xTask1;
	//static TaskHandle_t xTask1 = NULL, xTask2 = NULL;


extern scheduler_t scheduler[];
extern int sch_pos;
extern uint8_t sch_size;

extern int cnt;
extern uint8_t dummyAlert;

extern int tempSet1;
extern int tempSet0;

extern int defaultTempSet0;
extern int defaultTempSet1;


extern int cnt1;
extern int cnt2;
extern int cnt_flow1;
extern int cnt_flow2;
extern UART_HandleTypeDef huart3;
extern uint8_t uart_dma_buf_tx[STATE_DMA_SIZE];
extern uint8_t uart_dma_buf_rx[8];
extern state_t state;

extern RTC_TimeTypeDef sTime;
extern RTC_DateTypeDef sDate;
extern RTC_HandleTypeDef hrtc;


uint8_t send_buf[64];
extern uint16_t ADC_Raw[ADC_CHANNELS_NUM_RAW];  
//extern uint16_t adcData[ADC_CHANNELS_NUM];
extern float adcVoltage[ADC_CHANNELS_NUM];
extern uint16_t t1sum;
extern DS18B20 temperatureSensor;

/* ntc vars */
float Ntc_Tmp = 0;
uint16_t Ntc_R;
/* sheduler vars */
uint8_t Sch_100ms = 255;

/* R1 resistance */
#define NTC_UP_R 10000.0f
/* constants of Steinhart-Hart equation */
#define A 0.001111f
#define B 0.000237987f
#define C 0.000000065f
unsigned int  literperhour1;
unsigned int  literperhour2;
int  lphd;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myQueue01 */
osMessageQueueId_t myQueue01Handle;
const osMessageQueueAttr_t myQueue01_attributes = {
  .name = "myQueue01"
};
/* Definitions for myTimer01 */
osTimerId_t myTimer01Handle;
const osTimerAttr_t myTimer01_attributes = {
  .name = "myTimer01"
};
/* Definitions for myEvent01 */
osEventFlagsId_t myEvent01Handle;
const osEventFlagsAttr_t myEvent01_attributes = {
  .name = "myEvent01"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

static uint32_t rtc_counter = 0;
int task_callCnt = 0;
static void prvTask1( void *pvParameters )
{
    for( ;; )
    {
			ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
			task_callCnt++;
			

			RTC_AlarmTypeDef salarmstructure;
			if(sch_size>0) {
				if(dummyAlert == 1){
					dummyAlert = 0;
				} else { // apply temp correction from scheduler:
					if(scheduler[sch_pos].channel == 0)
						tempSet0 = scheduler[sch_pos].temp > 0 ? scheduler[sch_pos].temp : defaultTempSet0;
					else 
						tempSet1 = scheduler[sch_pos].temp > 0 ? scheduler[sch_pos].temp : defaultTempSet1;
					if(sch_pos<sch_size-1){ // check next entry if it with the same tima and date but other channel:
						int mask1 = *(uint32_t *)&scheduler[sch_pos] & 0xFFFC0000;
						int mask2 = *(uint32_t *)&scheduler[sch_pos+1] & 0xFFFC0000;
						if( mask1 == mask2 ){
							if(scheduler[sch_pos+1].channel == 0)
								tempSet0 = scheduler[sch_pos+1].temp > 0 ? scheduler[sch_pos+1].temp : defaultTempSet0;// scheduler[sch_pos+1].temp;
							else 
								tempSet1 = scheduler[sch_pos+1].temp > 0 ? scheduler[sch_pos+1].temp : defaultTempSet1;
						}
					}
				}
				handleScheduler();
			}
	}
}

void send_char(char c){
	HAL_I2C_Master_Transmit(&hi2c1, (I2C_ESP32_ADDRESS << 1), (uint8_t *)&c, 1,  5);
}

void uint32toa(uint32_t n, uint8_t s[]){
	int i = 9;
	do {       /* генерируем цифры в обратном порядке */
		s[i--] = n % 10 + '0';   /* берем следующую цифру */
	} while ((n /= 10) > 0);     /* удаляем */
	while(i>=0)
		s[i--] = '0';
 }


 void uint16_toa(uint16_t n, uint8_t s[]){
	int i = 5;
	do {       /* генерируем цифры в обратном порядке */
		s[i--] = n % 10 + '0';   /* берем следующую цифру */
	} while ((n /= 10) > 0);     /* удаляем */
	while(i>=0)
		s[i--] = '0';
 }



/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void Callback01(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of myTimer01 */
  myTimer01Handle = osTimerNew(Callback01, osTimerPeriodic, NULL, &myTimer01_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
	osTimerStart(myTimer01Handle, 1000);
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of myQueue01 */
  myQueue01Handle = osMessageQueueNew (16, sizeof(uint16_t), &myQueue01_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(StartTask02, NULL, &myTask02_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	rtc_conter = RTC_ReadTimeCounter(&hrtc);
	state.duration = 0;
	xTaskCreate( prvTask1, "Task1", 200, NULL, tskIDLE_PRIORITY, &xTask1 );

/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of myEvent01 */
  myEvent01Handle = osEventFlagsNew(&myEvent01_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
extern uint8_t aRxBuffer[];

int hal_busy_cnt = 0;
int sleepD=10;
uint32_t i2cerr = 0;		
uint16_t temp = 0;
		uint32_t hal_err = 0;
		uint16_t Ntc_Tmp_Raw_prew = 0; 
		extern int quickDayChange;
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);


		rtc_conter_delta = RTC_ReadTimeCounter(&hrtc) - rtc_conter;
		state.duration++;
		state.flow1+=90;
		state.flow2+=80;
/*	
		HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN); // RTC_FORMAT_BIN , RTC_FORMAT_BCD
		HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN); // RTC_FORMAT_BIN , RTC_FORMAT_BCD

		if(quickDayChange++ == 5){ // emulate day change
			sTime.Hours = 23;
			sTime.Minutes = 59;
			sTime.Seconds = 58;
			HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN); // RTC_FORMAT_BIN , RTC_FORMAT_BCD
		}
		if(quickDayChange == 10){
			RTC_AlarmTypeDef salarmstructure;
			HAL_RTC_GetAlarm(&hrtc, &salarmstructure,RTC_ALARM_A, RTC_FORMAT_BIN);
			int mins = salarmstructure.AlarmTime.Hours * 60 + salarmstructure.AlarmTime.Minutes - 1;
			
			sTime.Hours = mins / 60;
			sTime.Minutes = mins % 60;
			sTime.Seconds = 55;

			HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN); // RTC_FORMAT_BIN , RTC_FORMAT_BCD
		}
*/


		HAL_StatusTypeDef hStat;
		hStat = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)17<<1, 10,100);
		if(hStat == HAL_OK){
			hal_busy_cnt = 0;
    if(HAL_I2C_Master_Transmit_DMA(&hi2c1, (uint16_t)17<<1, (uint8_t *) &state,STATE_DMA_SIZE)!= HAL_OK)
			
//    if(HAL_I2C_Master_Seq_Transmit_IT(&hi2c1, (uint16_t)17<<1, (uint8_t *) &state,STATE_DMA_SIZE, I2C_FIRST_FRAME)!= HAL_OK)
			    {
      /* Error_Handler() function is called when error occurs. */
      Error_Handler();
    }} else if(hStat == HAL_BUSY){
			hal_busy_cnt++;
			if(hal_busy_cnt > 5){
				HAL_I2C_DeInit(&hi2c1);
				HAL_I2C_Init(&hi2c1);
			}
		} 
	
//		i2cerr = HAL_I2C_GetError(&hi2c1);
//		while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
//   {
//			osDelay(sleepD);
//			i2cerr++;
//    }


		
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Raw, ADC_CHANNELS_NUM_RAW);
		HAL_UART_Transmit_DMA(&huart3,(uint8_t *) &state, STATE_DMA_SIZE);

		/*
		DS18B20_InitializationCommand(&temperatureSensor);
    DS18B20_SkipRom(&temperatureSensor);
    DS18B20_ConvertT(&temperatureSensor, DS18B20_DATA);

    DS18B20_InitializationCommand(&temperatureSensor);
    DS18B20_SkipRom(&temperatureSensor);
    DS18B20_ReadScratchpad(&temperatureSensor);

		
		
// A7 - 10.07K
		// A0 - 9.91K
//		LL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

#define NUMSAMPLES 10// сколько показаний берется для определения среднего значения
#define SERIESRESISTOR 1070 //номинал подтягивающего резистора для термодатчика
#define THERMISTORNOMINAL 10000 // сопротивление при 25 градусах по Цельсию
#define TEMPERATURENOMINAL 25 // t. для номинального сопротивления (практически всегда равна 25 C)
#define BCOEFFICIENT 3950 // бета коэффициент термистора (из описания продавца)

		
		uint16_t Ntc_Tmp_Raw = calc_temperature(t1sum); //ADC_Raw[0]);
		if(Ntc_Tmp_Raw_prew == Ntc_Tmp_Raw){
			continue;
		}
		Ntc_Tmp_Raw_prew = Ntc_Tmp_Raw;
		uint8_t buf_text[] = "hvac1/group1/sensor1:     ";
	  uint16_toa(Ntc_Tmp_Raw, &buf_text[21]);
		
//		slip_send_packet(buf_text, 25, &send_char);
//    osDelay(10);
//		temp=Ntc_Tmp_Raw;
		int ret = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(I2C_ESP32_ADDRESS<<1), 3, 5);
		if(ret == HAL_OK)
			HAL_I2C_Master_Transmit(&hi2c1, (I2C_ESP32_ADDRESS << 1), (uint8_t *)&buf_text, 27,  5);
		else 
			hal_err++;

		*/
		
		
		
		
		
		
		
		
	/* get adc value */
//	HAL_ADC_Start_IT(&hadc1);
	/* filtering (sma) */


//	HAL_ADC_Start(&hadc1);
	/* Wait for conversion completion before conditional check hereafter */
//	HAL_ADC_PollForConversion(&hadc1, 1);

//	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Raw, 2);






		
//		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
//		osDelay(40);
		
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
int rx_cnt_ack = 0;
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(250);
//	i2c_rx_end = 1;
		if(HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_READY )
  /*##-4- Put I2C peripheral in reception process ############################*/  
  do
  {
    if(HAL_I2C_Master_Receive_DMA(&hi2c1, (uint16_t)17<<1, (uint8_t *)aRxBuffer, 4) != HAL_OK)
    {
      /* Error_Handler() function is called in case of error. */
//      Error_Handler();
    }

    /* When Acknowledge failure occurs (Slave don't acknowledge its address)
    Master restarts communication */
		rx_cnt_ack++;

  }
  while(HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF);
	
  }
  /* USER CODE END StartTask02 */
}

/* Callback01 function */
void Callback01(void *argument)
{
  /* USER CODE BEGIN Callback01 */
/*	
			float ntc_up_r7 = 10080.0f;
//			#define NTC_UP_R 1.0f
	
			float Ntc_R = (ntc_up_r7/((4095.0/adcData[7]) - 1));
			#define BCOEFFICIENT 3950
			#define THERMISTORNOMINAL 10000
			adcVoltage[7] = (BCOEFFICIENT * 298.15 ) / (BCOEFFICIENT + (298.15 * log(Ntc_R / THERMISTORNOMINAL)))-273.15;
*/
	/*
		HAL_ADC_Stop_DMA(&hadc1);
		HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&ADC_Raw, ADC_CHANNELS_NUM_RAW);

		float ntc_up_r7 = 10080.0f;
//			#define NTC_UP_R 1.0f
	
		for (uint8_t i = 0; i < ADC_CHANNELS_NUM; i++)
    {
			float Ntc_R = (ntc_up_r7/((4095.0/adcData[i]) - 1));
			#define BCOEFFICIENT 3950
			#define THERMISTORNOMINAL 10000
			adcVoltage[i] = (BCOEFFICIENT * 298.15 ) / (BCOEFFICIENT + (298.15 * log(Ntc_R / THERMISTORNOMINAL)))-273.15;
		}
		
		*/
		
	/*
	for (uint8_t i = 0; i < ADC_CHANNELS_NUM; i++)
    {
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
	
//	cnt_flow1 = cnt1;
//	cnt_flow2 = cnt2;

//	literperhour1 = (cnt_flow1 * 60 / 7.5);
//	literperhour1 = (cnt_flow1 <<3);
//	literperhour2 = (cnt_flow2 <<3);
//	lphd = literperhour1 - literperhour2;
	#define LED_Pin LL_GPIO_PIN_13
	#define LED_GPIO_Port GPIOC
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); 
  /* USER CODE END Callback01 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

