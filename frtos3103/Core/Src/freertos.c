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
#include "ssd1306.h"
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

#define FLAGS_MSK1 0x00000001U
 
esp2stm_i2c_status1_t msg_by_rs485;
uint32_t rtc_conter, rtc_conter_delta=0;


/* Prototypes of the two tasks created by main(). */
static void prvTask1( void *pvParameters );
static void prvTask2( void *pvParameters );

/* Handles for the tasks create by main(). */
extern TaskHandle_t xTask1;
	//static TaskHandle_t xTask1 = NULL, xTask2 = NULL;

extern config_t config;
extern scheduler_t scheduler[];
extern int sch_pos;
extern uint8_t sch_size;

extern int cnt;
extern uint8_t dummyAlert;

extern esp2stm_i2c_status1_t 	i2c_rxStatus;
extern esp2stm_i2c_scheduler_t i2c_rxScheduler;


extern int cnt_flow1;
extern int cnt_flow2;
extern UART_HandleTypeDef huart3;
extern uint8_t uart_dma_buf_tx[STATE_DMA_SIZE];
extern uint8_t uart_dma_buf_rx[512];
extern state_t state;

extern RTC_TimeTypeDef sTime;
extern RTC_DateTypeDef sDate;
extern RTC_HandleTypeDef hrtc;


uint8_t send_buf[64];
extern uint16_t ADC_Raw[ADC_CHANNELS_NUM_RAW];  
//extern uint16_t adcData[ADC_CHANNELS_NUM];
extern float adcVoltage[ADC_CHANNELS_NUM];
extern uint8_t adc_complete;
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



osThreadId_t tid_Thread_MsgQueue2;              // thread id 2
void Thread_MsgQueue2 (void *argument);         // thread function 2


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
/* Definitions for txBinSemaphore */
osSemaphoreId_t txBinSemaphoreHandle;
const osSemaphoreAttr_t txBinSemaphore_attributes = {
  .name = "txBinSemaphore"
};
/* Definitions for myEvent01 */
osEventFlagsId_t myEvent01Handle;
const osEventFlagsAttr_t myEvent01_attributes = {
  .name = "myEvent01"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */


void enableZeroCrossACscanExti(){
	__HAL_GPIO_EXTI_CLEAR_IT(zeroCrossLine3_Pin);
	HAL_NVIC_ClearPendingIRQ(EXTI3_IRQn);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}
void ui10toa(uint32_t n, uint8_t s[]){
	int i = 3;
	do {       /* генерируем цифры в обратном порядке */
		s[i--] = n % 10 + '0';   /* берем следующую цифру */
	} while ((n /= 10) > 0);     /* удаляем */
	while(i>=0)
		s[i--] = '0';
 }




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
					uint16_t tempLoc;
					if(scheduler[sch_pos].tempDeltaMode == 0) {
						if(scheduler[sch_pos].channel == 0)
							config.setTempFloor = scheduler[sch_pos].temp != 0 ? scheduler[sch_pos].temp : defaultTempSet0;
						else 
							config.setTempWater = scheduler[sch_pos].temp != 0 ? scheduler[sch_pos].temp : defaultTempSet1;
					} else {
//						config.setTempFloor += scheduler[sch_pos].temp;
						// todo
					}
					if(sch_pos<sch_size-1){ // check next entry if it with the same tima and date but other channel:
						int mask1 = *(uint32_t *)&scheduler[sch_pos] 		& 0xFFFC0000;
						int mask2 = *(uint32_t *)&scheduler[sch_pos+1] 	& 0xFFFC0000;
						if( mask1 == mask2 ){
							if(scheduler[sch_pos+1].channel == 0)
								config.setTempFloor = scheduler[sch_pos+1].temp > 0 ? scheduler[sch_pos+1].temp : defaultTempSet0;// scheduler[sch_pos+1].temp;
							else 
								config.setTempWater = scheduler[sch_pos+1].temp > 0 ? scheduler[sch_pos+1].temp : defaultTempSet1;
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


 
osStatus_t txStatus(){
	osStatus_t sem_stat = osSemaphoreAcquire(txBinSemaphoreHandle,10);
	if(sem_stat == osOK) {
		state.end.crc = usMBCRC16((uint8_t *)&state, sizeof(state_t)-sizeof(end_t));
		HAL_UART_Transmit_DMA(&huart3,(uint8_t *) &state, STATE_DMA_SIZE);
	}
	return sem_stat;
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

  /* Create the semaphores(s) */
  /* creation of txBinSemaphore */
  txBinSemaphoreHandle = osSemaphoreNew(1, 1, &txBinSemaphore_attributes);

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
  myQueue01Handle = osMessageQueueNew (8, 16, &myQueue01_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  myQueue01Handle = osMessageQueueNew (5, sizeof(esp2stm_i2c_status1_t), &myQueue01_attributes);
	
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(StartTask02, NULL, &myTask02_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
/* add threads, ... */
	rtc_conter = RTC_ReadTimeCounter(&hrtc);
	state.duration = 0;
	xTaskCreate( prvTask1, "Task1", 128, NULL, tskIDLE_PRIORITY, &xTask1 );


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
extern IWDG_HandleTypeDef hiwdg;

int hal_busy_cnt = 0;
int sleepD=10;
uint32_t i2cerr = 0;		
uint16_t temp = 0;
		uint32_t hal_err = 0;
		uint16_t Ntc_Tmp_Raw_prew = 0; 
		extern int quickDayChange;
uint8_t str_redraw = 0;
int FSM_CHECK_DEVICES_ERROR_count=0;
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

		switch(state.state_mask.fsmState){
			case FSM_RESTART:
				state.state_mask.fsmState = FSM_RESTART_GPIO_MODULE_CONNECTING;
				break;
			case FSM_RESTART_GPIO_MODULE_CONNECTING:
				if( changeGPIOstate(0, port20gpioAll) == pdFALSE){
					state.state_mask.fsmState = FSM_CHECK_DEVICES_ERROR;
				} else
					state.state_mask.fsmState = FSM_RESTART_RS485_REMOTE_CONNECTING;
				break;
			case FSM_RESTART_RS485_REMOTE_CONNECTING:
// start handshaking process and wait responce from ESP32 side by rs485  or CAN line
				state.state_mask.fsmState = FSM_MAIN_RESTRICT_WORKING;
				break;
			case FSM_MAIN_RESTRICT_WORKING: 
				
			// start ADC measure
				HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Raw, ADC_CHANNELS_NUM_RAW);
				rtc_conter_delta = RTC_ReadTimeCounter(&hrtc) - rtc_conter;
				state.duration++;

				osDelay(100);
				
			// wait to ADC complete before crc is calculated:
				while(adc_complete != 1){
					osDelay(10);// move to	vTaskNotifyGiveFromISR( xTask1, &pxHigherPriorityTaskWoken ); or semaphore
				}
				adc_complete = 0;
				txStatus();
				HAL_UARTEx_ReceiveToIdle_DMA(&huart3, uart_dma_buf_rx, RX_BUFER_MAX_SIZE);
    //disable half transfer interrupt; it is enabled in HAL_UARTEx_ReceiveToIdle_DMA()
				__HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
//				HAL_IWDG_Refresh(&hiwdg);
//				canSend();

				// update screen
				SSD1306_Fill(SSD1306_COLOR_BLACK);
				SSD1306_GotoXY(0, 0);
				switch( msg_by_rs485.status){
					case 0: 				
						SSD1306_Puts2("0", &microsoftSansSerif_12ptFontInfo, SSD1306_COLOR_WHITE);
						break;
					case 1: 				
						SSD1306_Puts2("1", &microsoftSansSerif_12ptFontInfo, SSD1306_COLOR_WHITE);
						break;
					case 2: 				
						SSD1306_Puts2("2", &microsoftSansSerif_12ptFontInfo, SSD1306_COLOR_WHITE);
						break;
				}
				char bufi[4];
				*(uint32_t *)&bufi = 0;
				if(state.temp[9]>=1996)
					state.temp[9] -= 1996;
				else
					state.temp[9] = 0;					
					
				ui10toa(state.temp[9], (uint8_t *)&bufi);
				SSD1306_Puts2(bufi, &microsoftSansSerif_12ptFontInfo, SSD1306_COLOR_WHITE);

				SSD1306_Puts2("offline", &microsoftSansSerif_12ptFontInfo, SSD1306_COLOR_WHITE);
//				SSD1306_Puts2("...", &microsoftSansSerif_12ptFontInfo, SSD1306_COLOR_WHITE);
				for(int a=0;a<str_redraw;a++){
					SSD1306_Puts2(".",&microsoftSansSerif_12ptFontInfo, SSD1306_COLOR_WHITE);
				}
				if(str_redraw++>5)
					str_redraw = 0;
//				if(msg_by_rs485.cmd)
					
				state.gpio_mask.compressor = !(state.gpio_mask.compressor);
				changeGPIOstate(0, 0);

				SSD1306_GotoXY(0, 16);

				state.gpio_mask.compressor ? SSD1306_Putc2big('1',&microsoftSansSerif_12ptFontInfo) : SSD1306_Putc2big('0',&microsoftSansSerif_12ptFontInfo);
				state.gpio_mask.heater1 ? SSD1306_Putc2big('1',&microsoftSansSerif_12ptFontInfo) : SSD1306_Putc2big('0',&microsoftSansSerif_12ptFontInfo);
				state.gpio_mask.heater2 ? SSD1306_Putc2big('1',&microsoftSansSerif_12ptFontInfo) : SSD1306_Putc2big('0',&microsoftSansSerif_12ptFontInfo);
				state.gpio_mask.waterPump ? SSD1306_Putc2big('1',&microsoftSansSerif_12ptFontInfo) : SSD1306_Putc2big('0',&microsoftSansSerif_12ptFontInfo);
				state.gpio_mask.circulPump1 ? SSD1306_Putc2big('1',&microsoftSansSerif_12ptFontInfo) : SSD1306_Putc2big('0',&microsoftSansSerif_12ptFontInfo);
				state.gpio_mask.circulPump2 ? SSD1306_Putc2big('1',&microsoftSansSerif_12ptFontInfo) : SSD1306_Putc2big('0',&microsoftSansSerif_12ptFontInfo);
				state.gpio_mask.CrankcaseHeater ? SSD1306_Putc2big('1',&microsoftSansSerif_12ptFontInfo) : SSD1306_Putc2big('0',&microsoftSansSerif_12ptFontInfo);
				state.gpio_mask.reserved ? SSD1306_Putc2big('1',&microsoftSansSerif_12ptFontInfo) : SSD1306_Putc2big('0',&microsoftSansSerif_12ptFontInfo);
//				SSD1306_Putc2big('1',&microsoftSansSerif_12ptFontInfo);
//				SSD1306_Puts2("1011101111", &microsoftSansSerif_12ptFontInfo, SSD1306_COLOR_WHITE);

				SSD1306_UpdateScreen();
//				state.gpio_mask.circulPump1 = !(state.gpio_mask.circulPump1);
				osEventFlagsWait( myEvent01Handle, FLAGS_MSK1, osFlagsWaitAny, osWaitForever); // wait one second timer tick event

//				osDelay(1000);
				break;
			case FSM_CHECK_DEVICES_ERROR:
				if(FSM_CHECK_DEVICES_ERROR_count++ < 10) //  reboot by watchdog after 10 seconds
					HAL_IWDG_Refresh(&hiwdg);

				osTimerStop(myTimer01Handle);
				for(int a = 0; a < 4; a++){
					HAL_GPIO_TogglePin(LED_Pin_GPIO_Port, LED_Pin_Pin); 
					osDelay(100);
				}
				
				SSD1306_Fill(SSD1306_COLOR_BLACK);
				SSD1306_Puts2(bufi, &microsoftSansSerif_12ptFontInfo, SSD1306_COLOR_WHITE);
				SSD1306_GotoXY(0, 0);
				SSD1306_Puts2("device.err", &microsoftSansSerif_12ptFontInfo, SSD1306_COLOR_WHITE);
				SSD1306_UpdateScreen();
				
				osEventFlagsWait( myEvent01Handle, FLAGS_MSK1, osFlagsWaitAny, osWaitForever); // wait one second timer tick event
		}
//		if(Ntc_Tmp_Raw_prew <20)

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

//		HAL_StatusTypeDef hStat;
//		hStat = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)17<<1, 10,100);
//		if(hStat == HAL_OK){
//			hal_busy_cnt = 0;
//    if(HAL_I2C_Master_Transmit_DMA(&hi2c1, (uint16_t)17<<1, (uint8_t *) &state,STATE_DMA_SIZE)!= HAL_OK){
//      /* Error_Handler() function is called when error occurs. */
//      Error_Handler();
//    }} else if(hStat == HAL_BUSY){
//			hal_busy_cnt++;
//			if(hal_busy_cnt > 5){ // todo
//				HAL_I2C_DeInit(&hi2c1);
//				HAL_I2C_Init(&hi2c1);
//			}
//		} 

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
//	  esp2stm_i2c_status1_t msg;
  osStatus_t status;
 
  while (1) {
    ; // Insert thread code here...
    status = osMessageQueueGet(myQueue01Handle, &msg_by_rs485, NULL, osWaitForever);   // wait for message
    if (status == osOK) {
			rx_cnt_ack++;
			switch(msg_by_rs485.cmd){
				case 't':
					state.setT.set1 = msg_by_rs485.dummy;
					txStatus();
					break;
				case 'T':	
					state.setT.set2 = msg_by_rs485.dummy;
					txStatus();
					break;
				case 'r':// refresh
					txStatus();
					break;
				case 'P':// refresh
					state.state_mask.disabled = msg_by_rs485.dummy;
					txStatus();
					break;
			}
    }
  }
  /* USER CODE END StartTask02 */
}

/* Callback01 function */
void Callback01(void *argument)
{
  /* USER CODE BEGIN Callback01 */
	config.coldSideFlowLPH = (state.flow1 <<3);
	config.coldSideFlowLPH -= (config.coldSideFlowLPH>>4); // -6.25% calibration
	config.hotSideFlowLPH = (state.flow2 <<3);
	state.flow1 = 0;
	state.flow2 = 0;
	HAL_GPIO_TogglePin(LED_Pin_GPIO_Port, LED_Pin_Pin); 
	osEventFlagsSet(myEvent01Handle, FLAGS_MSK1);
  /* USER CODE END Callback01 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void Thread_MsgQueue2 (void *argument) {
}

#define size_of_data 100
uint16_t current_data1[size_of_data];
uint16_t current_data2[size_of_data];
float calc,result;
float current_result, current_previous;
#define offset 2010

static float kalman_filter(float RMS_Value)
{
    float x_k1_k1,x_k_k1;

    static float RMS_OLD_Value=0;

    float Z_k;

    static float P_k1_k1;

    static float Q = 0.01;//Q: Regulation noise, Q increases, dynamic response becomes faster, and convergence stability becomes worse
    static float R = 0.5; //R: Test noise, R increases, dynamic response becomes slower, convergence stability becomes better
    static float Kg = 0;
    static float P_k_k1 = 1;

    float kalman_rms;
    static float kalman_rms_old=0;
    Z_k = RMS_Value;
    x_k1_k1 = kalman_rms_old;

    x_k_k1 = x_k1_k1;
    P_k_k1 = P_k1_k1 + Q;

    Kg = P_k_k1/(P_k_k1 + R);

    kalman_rms = x_k_k1 + Kg * (Z_k - kalman_rms_old);
    P_k1_k1 = (1 - Kg)*P_k_k1;
    P_k_k1 = P_k1_k1;

    RMS_OLD_Value = RMS_Value;
    kalman_rms_old = kalman_rms;

    return kalman_rms;
}

static float calculate_rms(uint16_t * values)
{
	calc=0;
	for (int i=0;i<size_of_data;i++)
	{
		calc=calc+((values[i]-offset) * (values[i]-offset));
	}
	calc=calc/size_of_data;
	calc=sqrt(calc);

	result=kalman_filter(calc);
	result=0.0206*result-0.2;
	current_previous=result;
	return result;
}


 
/* USER CODE END Application */


