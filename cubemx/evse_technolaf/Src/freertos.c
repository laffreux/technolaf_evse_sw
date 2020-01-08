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
#include <math.h>
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
static double v_line_rms;
static double EVSECurrent = 26.0;
static uint16_t GFILevel = 0;
static double MeasuredCurrent;
/* USER CODE END Variables */
osThreadId CheckHandle;
osThreadId PilotHandle;
osThreadId MainTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
int cmpfunc (const void * a, const void * b);
/* USER CODE END FunctionPrototypes */

void StartCheckTask(void const * argument);
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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Check */
  osThreadDef(Check, StartCheckTask, osPriorityNormal, 0, 128);
  CheckHandle = osThreadCreate(osThread(Check), NULL);

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

/* USER CODE BEGIN Header_StartCheckTask */
/**
  * @brief  Function implementing the Check thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartCheckTask */
void StartCheckTask(void const * argument)
{

  /* USER CODE BEGIN StartCheckTask */
  int sample_ctr = 0;
  static const uint32_t ADC_BUFFER_SIZE = 32;
  uint16_t line1[ADC_BUFFER_SIZE];	// circular buffer that contains the adc samples in mV
  uint16_t line2[ADC_BUFFER_SIZE];	// circular buffer that contains the adc samples in mV
  uint16_t peak1;
  uint16_t peak2;

  uint16_t current[ADC_BUFFER_SIZE];	// circular buffer that contains the adc samples in mV
  uint16_t peak_current;
  uint16_t gfi[ADC_BUFFER_SIZE];	// circular buffer that contains the adc samples in mV
  uint16_t peak_gfi;

  for(int i = 0; i < ADC_BUFFER_SIZE; i++){
	  line1[i] = 0;
	  line2[i] = 0;
	  gfi[i] = 0;
	  current[i] = 0;
  }

  // ADC sample period

  /* Infinite loop */
  for(;;)
  {
    osDelay(2);

    line1[sample_ctr % ADC_BUFFER_SIZE] = read_line1_voltage();
	line2[sample_ctr % ADC_BUFFER_SIZE] = read_line2_voltage();
	current[sample_ctr % ADC_BUFFER_SIZE] = read_current();
	gfi[sample_ctr % ADC_BUFFER_SIZE] = read_gfi();
	sample_ctr++;

	// find peaks
	peak1 = 0;
	peak2 = 0;
	peak_current = 0;
	peak_gfi = 0;
	for(int i = 0; i < ADC_BUFFER_SIZE; i++){
		if(line1[i] > peak1) {
			peak1 = line1[i];
		}
		if(line2[i] > peak2) {
			peak2 = line2[i];
		}
		if(current[i] > peak_current) {
			peak_current = current[i];
		}
		if(gfi[i] > peak_gfi) {
			peak_gfi = gfi[i];
		}
	}

	// voltage is sum of line1 and line2

	// 500 is to remove the diode voltage drop
	// when input voltage is 0, the capacitor is getting charged by the leakage
	// current of the adc pin (about 0.5 uA) so the voltage measured is about 500 mV
	// so we consider that there is no voltage below 700 mV
	peak1 = (peak1 > 700) ? peak1 - 500 : 0;
	peak2 = (peak2 > 700) ? peak2 - 500 : 0;

	v_line_rms = (double)(peak1 + peak2) / 5.67;
	GFILevel = peak_gfi;

	// current
	MeasuredCurrent =  exp((peak_current + 466.94) / 837.41);




  }
  /* USER CODE END StartCheckTask */
}

/* USER CODE BEGIN Header_StartTaskPilot */



/**
* @brief Function implementing the Pilot thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskPilot */
void StartTaskPilot(void const * argument)
{
  /* USER CODE BEGIN StartTaskPilot */

  // ADC sample period
  const uint32_t SAMPLE_PERIOD = 9;	// ms	(to be exact, it must be a multiple of 1/733)

  // minimum delay before changing the state of the pilot
  const uint32_t STATE_MIN_DELAY = 500 / SAMPLE_PERIOD; // ms / period

  // buffer to store adc samples
  static const uint32_t BUFFER_SIZE = 16;
  int samples[BUFFER_SIZE];	// circular buffer that contains the adc samples in mV
  int pilot_voltage = 0;
  int filtered = 0;

  PILOT_STATE old_state = -1;
  uint32_t time_elapsed = 0;
  uint32_t samplectr = 0;
  int32_t sum = 0;



  // init the buffer to the highest voltage (car unplugged)
  for(int i = 0; i < BUFFER_SIZE; i++){
	  samples[i] = 3000;
  }


  /* Infinite loop */
  for(;;)
  {

	osDelay(SAMPLE_PERIOD);

	time_elapsed++;

	// read adc
	pilot_voltage = (int)read_pilot_voltage();

	// take into account only samples which occurs during positive PWM
	if(pilot_voltage > 1400){
		samples[samplectr++ % BUFFER_SIZE] = pilot_voltage;

		// use an average of half of the highest samples
		qsort(samples, BUFFER_SIZE, sizeof(int), cmpfunc);
		sum = 0;
		for(int i = BUFFER_SIZE /2; i < BUFFER_SIZE; i++){
			sum += samples[i];
		}
		filtered = sum / (BUFFER_SIZE /2);
	}

	// Here we change the state according to the voltage of the
	// Pilot (seen by the MCU)
	// State A => PILOT_VEHICLE_NOT_DETECTED
	// State B => PILOT_VEHICLE_PRESENT
	// State C => PILOT_READY
	if(filtered > 1600 && filtered < 2000 && time_elapsed > STATE_MIN_DELAY){
		PilotState = PILOT_READY;
	} else if(filtered >= 2100 && filtered < 2250 && time_elapsed > STATE_MIN_DELAY){
		PilotState = PILOT_VEHICLE_PRESENT;
	} else if(filtered >= 2350 && time_elapsed > STATE_MIN_DELAY) {
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

  const uint32_t MAIN_TASK_PERIOD = 20;	// ms
  const uint32_t FAULT_PERSISTANCE_DELAY = 25; // multiples of MAIN_TASK_PERIOD

  uint16_t freectr = 0;
  enum states {IDLE, PRESENT, CHARGING, FAULT};
  uint8_t state = IDLE;
  BOOLEAN blinker_blue = FALSE;
  BOOLEAN blinker_green = FALSE;
  uint32_t fault_persistance = 0;
  uint32_t gfi_persistance = 0;

  adc_Start();

  // Read current from dip switches
  uint16_t code = read_dip_switch();
  if(code >= 15){
	  // default current when all dip switches open (or no dip switch installed)
	  EVSECurrent = 28.0;
  } else {
	  EVSECurrent = (code + 1) * 3.2;
  }

  // turn ON D8 ( Power present )
  set_indicator(LED_POWER, TRUE);
  // turn OFF contactor (it should be already off... just to make sure)
  set_contactor(FALSE);
  // No PWM during state A
  set_pwm(FALSE, 0.0);

  /* Infinite loop */
  for(;;)
  {
	// wait 20 ms, to let other tasks running
	osDelay(MAIN_TASK_PERIOD);
	freectr++;
    // blink blue disco led
	if(freectr % 10 == 0){
		set_indicator(LED_DISCO_BLUE, blinker_blue = !blinker_blue);
	}

	// uncomment below to test the contactor with the blue button
	// must comment all "set_contactor" later in the code
//	if(HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == GPIO_PIN_SET){
//		set_contactor(TRUE);
//		set_indicator(LED_DISCO_GREEN, TRUE);
//	} else {
//		set_contactor(FALSE);
//		set_indicator(LED_DISCO_GREEN, FALSE);
//	}


	// Handle ground fault detection
	// the GFI Level must be above the noise floor
	// and this threshold has been tested to detect a 6.6 kohm to ground leak
	if(GFILevel > 300){
		if(gfi_persistance < FAULT_PERSISTANCE_DELAY){
			gfi_persistance++;
		} else {
			state = FAULT;
		}
	} else {
		gfi_persistance = 0;
	}

	// Main state machine
	// IDLE => no car is plugged
	// PRESENT => A car has been plugged in and we now publish the max current through the pilot signal and the contactor is closed
	// CHARGING => The car is pulling current from the station
	// FAULT => A problem has been detected and the contactor is open
	//          To recover, mains has to be switched OFF and ON again
	//   Possible problems are:
	//     - ground fault
	//     - overcurrent
	//     - overvoltage
	//	   - undervoltage (from mains or contactor failed open)
	//     - contactor sticked (voltage has been detected but the contactor should be open)

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

		// check voltage (must be low)
		if(v_line_rms > 80.0){
			fault_persistance++;
		} else {
			fault_persistance = 0;
		}



		if(fault_persistance > FAULT_PERSISTANCE_DELAY){
			state = FAULT;
		} else if(PilotState == PILOT_VEHICLE_PRESENT || PilotState == PILOT_READY){
			fault_persistance = 0;
    		state = PRESENT;
    		set_pwm(TRUE, EVSECurrent);
    	}
    	break;
    case PRESENT:
    	// turn OFF debug led green
    	set_indicator(LED_DISCO_GREEN, TRUE);
    	// turn ON contactor
    	set_contactor(TRUE);
    	// turn ON D9 (Ready to charge indicator)
    	set_indicator(LED_READY, TRUE);
    	// turn ON D10 (Connected vehicle indicator)
    	set_indicator(LED_CONNECTED, TRUE);
    	// turn OFF D11 (Vehicle charging indicator)
		set_indicator(LED_CHARGING, FALSE);
    	// turn OFF D12 (Trouble indicator)
		set_indicator(LED_FAULT, FALSE);

		// check voltage
		if(v_line_rms < 90.0 || v_line_rms > 300.00){
			fault_persistance++;
		} else {
			fault_persistance = 0;
		}

		if(fault_persistance > FAULT_PERSISTANCE_DELAY){
			state = FAULT;
		} else if(PilotState == PILOT_READY){
    		state = CHARGING;
    		fault_persistance = 0;
    	} else if(PilotState == PILOT_VEHICLE_NOT_DETECTED){
    		state = IDLE;
    		set_pwm(FALSE, 0.0);
    		fault_persistance = 0;
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


		// check voltage
		if(v_line_rms < 90.0 || v_line_rms > 300.00 || MeasuredCurrent > EVSECurrent * 1.25){
			fault_persistance++;
		} else {
			fault_persistance = 0;
		}

		if(fault_persistance > FAULT_PERSISTANCE_DELAY){
			state = FAULT;
		} else if(PilotState == PILOT_VEHICLE_PRESENT){
    		state = PRESENT;
    	} else if(PilotState == PILOT_VEHICLE_NOT_DETECTED){
    		state = IDLE;
    		set_pwm(FALSE, 0.0);
    	}

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
int cmpfunc (const void * a, const void * b) {
   return ( *(int*)a - *(int*)b );
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
