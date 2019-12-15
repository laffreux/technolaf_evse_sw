/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "adc.h"
#include "tim.h"
#include "gpio.h"
#include <stdlib.h>
typedef enum states { PILOT_VEHICLE_NOT_DETECTED, PILOT_VEHICLE_PRESENT, PILOT_READY } PILOT_STATE;

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
/* USER CODE BEGIN Variables */
static PILOT_STATE PilotState = PILOT_VEHICLE_NOT_DETECTED;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId PilotHandle;
osThreadId MainTaskHandle;
osMessageQId ChargingStatusHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartTaskPilot(void const * argument);
void StartTaskMainTask(void const * argument);

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

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of ChargingStatus */
  osMessageQDef(ChargingStatus, 16, uint16_t);
  ChargingStatusHandle = osMessageCreate(osMessageQ(ChargingStatus), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Pilot */
  osThreadDef(Pilot, StartTaskPilot, osPriorityNormal, 0, 128);
  PilotHandle = osThreadCreate(osThread(Pilot), NULL);

  /* definition and creation of MainTask */
  osThreadDef(MainTask, StartTaskMainTask, osPriorityIdle, 0, 128);
  MainTaskHandle = osThreadCreate(osThread(MainTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {


	  osDelay(1);


  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTaskPilot */

int cmpfunc (const void * a, const void * b) {
   return ( *(int*)a - *(int*)b );
}

/**
* @brief Function implementing the Pilot thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskPilot */
void StartTaskPilot(void const * argument)
{
  /* USER CODE BEGIN StartTaskPilot */


  const uint32_t SAMPLE_PERIOD = 9;	// ms	(to be exact, it must be a multiple of 1/733)
  const uint32_t STATE_MIN_DELAY = 500 / SAMPLE_PERIOD; // ms
  static const uint32_t BUFFER_SIZE = 16;
  int samples[BUFFER_SIZE];	// circular buffer that contains the adc samples in mV
  int pilot_voltage = 0;
  int filtered = 0;

  PILOT_STATE old_state = -1;
  uint32_t time_elapsed = 0;
  uint32_t samplectr = 0;
  int32_t sum = 0;

  // Adjust the pilot PWM
  adc_Start();

  // init the buffer to the highest voltage (car unplugged)
  for(int i = 0; i < BUFFER_SIZE; i++){
	  samples[i] = 3000;
  }


  /* Infinite loop */
  for(;;)
  {

	osDelay(SAMPLE_PERIOD);
	// wait 500 us

	time_elapsed++;

	// read adc
	pilot_voltage = (int)read_pilot_voltage();

	// take into account only samples which occurs during positive PWM
	if(pilot_voltage > 1400){
		// Measure and apply an IIR filter
		samples[samplectr++ % BUFFER_SIZE] = pilot_voltage;

		// use an average of half of the highest samples
		qsort(samples, BUFFER_SIZE, sizeof(int), cmpfunc);
		sum = 0;
		for(int i = BUFFER_SIZE /2; i < BUFFER_SIZE; i++){
			sum += samples[i];
		}
		filtered = sum / (BUFFER_SIZE /2);
	}

	if(filtered > 1600 && filtered < 2100 && time_elapsed > STATE_MIN_DELAY){
		PilotState = PILOT_READY;

	} else if(filtered >= 2200 && filtered < 2400 && time_elapsed > STATE_MIN_DELAY){
		PilotState = PILOT_VEHICLE_PRESENT;
	} else if(filtered >= 2450 && time_elapsed > STATE_MIN_DELAY) {
		PilotState = PILOT_VEHICLE_NOT_DETECTED;
	}

	if(PilotState != old_state) {
		time_elapsed = 0;
	}
	old_state = PilotState;


  }
  /* USER CODE END StartTaskPilot */
}

/* USER CODE BEGIN Header_StartTaskMainTask */
/**
* @brief Function implementing the MainTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskMainTask */
void StartTaskMainTask(void const * argument)
{
  /* USER CODE BEGIN StartTaskMainTask */
  static uint16_t freectr = 0;
  enum states {IDLE, PRESENT, CHARGING, FAULT};
  uint8_t state = IDLE;
  BOOLEAN blinker_blue = FALSE;
  BOOLEAN blinker_green = FALSE;
  // turn ON D8 ( Power present )
  set_indicator(LED_POWER, TRUE);
  set_contactor(FALSE);
  set_pwm(FALSE, 0.0);
  /* Infinite loop */
  for(;;)
  {

	osDelay(20);
	freectr++;
    // blink blue disco
	if(freectr % 10 == 0){
		set_indicator(LED_DISCO_BLUE, blinker_blue = !blinker_blue);
	}

//	if(HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == GPIO_PIN_SET){
//		set_contactor(TRUE);
//		set_indicator(LED_DISCO_GREEN, TRUE);
//	} else {
//		set_contactor(FALSE);
//		set_indicator(LED_DISCO_GREEN, FALSE);
//	}


    switch(state){
    case IDLE:

    	// turn OFF debug led green
    	set_indicator(LED_DISCO_GREEN, FALSE);
    	// turn OFF contactor
    	set_contactor(FALSE);
    	// turn ON D9 (Ready to charge indicator)
    	set_indicator(LED_READY, TRUE);
    	// turn ON D10 (Connected vehicle indicator)
    	set_indicator(LED_CONNECTED, FALSE);
    	// turn OFF D11 (Vehicle charging indicator)
		set_indicator(LED_CHARGING, FALSE);
    	// turn OFF D12 (Trouble indicator)
		set_indicator(LED_FAULT, FALSE);
    	if(PilotState == PILOT_VEHICLE_PRESENT){
    		state = PRESENT;
    		set_pwm(TRUE, 24.0);
    	}
    	break;
    case PRESENT:

    	set_contactor(TRUE);

    	// turn OFF debug led green
    	set_indicator(LED_DISCO_GREEN, TRUE);
    	// turn OFF contactor

    	// turn ON D9 (Ready to charge indicator)
    	set_indicator(LED_READY, TRUE);
    	// turn ON D10 (Connected vehicle indicator)
    	set_indicator(LED_CONNECTED, TRUE);
    	// turn OFF D11 (Vehicle charging indicator)
		set_indicator(LED_CHARGING, FALSE);
    	// turn OFF D12 (Trouble indicator)
		set_indicator(LED_FAULT, FALSE);


		if(PilotState == PILOT_READY){
    		state = CHARGING;
    	} else if(PilotState == PILOT_VEHICLE_NOT_DETECTED){
    		state = IDLE;
    		set_pwm(FALSE, 0.0);
    	}
    	break;
    case CHARGING:
    	// blink debug led green
    	if(freectr % 20 == 0){
    		set_indicator(LED_DISCO_GREEN, blinker_green = !blinker_green);
    	}

    	// turn ON contactor
    	set_contactor(TRUE);

    	// turn ON D9 (Ready to charge indicator)
    	set_indicator(LED_READY, TRUE);
    	// turn ON D10 (Connected vehicle indicator)
    	set_indicator(LED_CONNECTED, TRUE);
    	// turn OFF D11 (Vehicle charging indicator)
		set_indicator(LED_CHARGING, TRUE);
    	// turn OFF D12 (Trouble indicator)
		set_indicator(LED_FAULT, FALSE);

		if(PilotState == PILOT_VEHICLE_PRESENT){
    		state = PRESENT;
    	} else if(PilotState == PILOT_VEHICLE_NOT_DETECTED){
    		state = IDLE;
    		set_pwm(FALSE, 0.0);
    	}

    	// detect ground fault
    	// detect overcurrent
    	// detect undervoltage
    	// detect overvoltage


    	break;
    case FAULT:
    	// turn OFF debug led green
    	set_indicator(LED_DISCO_GREEN, FALSE);
    	// turn OFF contactor
    	set_contactor(FALSE);
    	// turn ON D9 (Ready to charge indicator)
    	set_indicator(LED_READY, FALSE);
    	// turn ON D10 (Connected vehicle indicator)
    	set_indicator(LED_CONNECTED, FALSE);
    	// turn OFF D11 (Vehicle charging indicator)
		set_indicator(LED_CHARGING, FALSE);
    	// turn OFF D12 (Trouble indicator)
		set_indicator(LED_FAULT, TRUE);
    	break;
    }
  }
  /* USER CODE END StartTaskMainTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
