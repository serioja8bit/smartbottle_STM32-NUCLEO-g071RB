/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "ds18b20.h"
#include <string.h>
#include <stdio.h>
#include "melody.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_TEMP 25
#define MIN_TEMP 21


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t Menu_Stage_1 = 0;
uint8_t Menu_Stage_2 = 0;
float Flow;
float water;

int dash = 0;
uint8_t led = 1;
uint8_t hour;
uint8_t alarm;

uint8_t menu;

uint8_t sel = 0;
uint8_t time;

/*uint32_t counter = 0;
uint32_t prev_counter = 0;*/
uint16_t value;

float Temperature = 0;
float waterVolume = 2;
char string_buff[10];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void setTimeInterval ();
void setVolume();
void setLed();
void setAlarm(uint8_t);
void getValueADC (uint8_t max_state);
void ssd1306_DrawLedStatus(uint8_t);

void updateMenu()
{
	ssd1306_DrawMenu(sel);
}

void execCommandSelect()
{
	if (Menu_Stage_1 == 1 && Menu_Stage_2 == 1)
	switch(sel)
		{
			case 0:
				setTimeInterval();
			break;

			case 1:
				setLed();
			break;

			case 2:
				setVolume();

			break;

			case 3:
				//exit menu;
				Menu_Stage_1 = 0;
				Menu_Stage_2 = 0;
				menu = 0;
				dash = 0;
				ssd1306_UpdateScreen();
				ssd1306_Fill(Black);
			break;

		}
}

void setTimeInterval (){
	ssd1306_Fill(Black);

			while (1){
				getValueADC(12);
				hour = sel;
				time = hour;
				setAlarm(hour);
				sprintf(string_buff, "%d", hour);
				ssd1306_DrawIntervalStatus(string_buff);
				if(Menu_Stage_2 == 0)
					break;
			}
}

void setVolume(){
		ssd1306_Fill(Black);

		while (1){
			getValueADC(5);
			waterVolume = sel;
			sprintf(string_buff, "%.f", waterVolume);
			ssd1306_DrawVolumeStatus(string_buff);
			if(Menu_Stage_2 == 0)
				break;
		}

}
void setLed(){

	ssd1306_Fill(Black);
			while (1){
				getValueADC(1);
				led = sel;
				ssd1306_DrawLedStatus(led);
				if(Menu_Stage_2 == 0)
					break;
			}
}

void getValueADC (uint8_t max_state){
	uint8_t prescalar = 63 / max_state;
	HAL_ADC_PollForConversion(&hadc1,1000);
	sel = (uint8_t)HAL_ADC_GetValue(&hadc1)/prescalar;
}

void setAlarm(uint8_t x){

	RTC_TimeTypeDef sTime = {0};
	RTC_AlarmTypeDef sAlarm = {0};

	sTime.Hours = 0x0;
	sTime.Minutes = 0x0;
	sTime.Seconds = 0x0;
	sTime.SubSeconds = 0x0;
	sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sTime.StoreOperation = RTC_STOREOPERATION_RESET;
	  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
	  {
	    Error_Handler();
	  }


	  /** Enable the Alarm A
	  */
	  sAlarm.AlarmTime.Hours = 0x0;
	  sAlarm.AlarmTime.Minutes = x;
	  sAlarm.AlarmTime.Seconds = 0x0;
	  sAlarm.AlarmTime.SubSeconds = 0x0;
	  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
	  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
	  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
	  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
	  sAlarm.AlarmDateWeekDay = 0x1;
	  sAlarm.Alarm = RTC_ALARM_A;
	  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
	  {
	    Error_Handler();
	  }

}

void BuzzerPlayNote(int prescalerfornote, int NoteDurationMs)
{
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);

		for(int i = 0; i < NoteDurationMs ; i++)
		{
			//prescaler = 64000000 / (255 * prescalerfornote * 3);

			TIM2->PSC =  prescalerfornote; //change prescaler for have the frequency of our note
			HAL_Delay(1);

		}
		TIM2->PSC=0;
	HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_1);

}

void playBuzzer ()
{

	for(int i = 0 ; i < melodySize; i++ ){
			BuzzerPlayNote(marioMelody[i], marioDuration[i] * 7 );

		}
}




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim6);
  HAL_ADC_Start(&hadc1);
  ssd1306_Init();
  //ssd1306_TestDrawUTM();




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (dash == 0 ){
	 	  	ssd1306_DrawDashboard();
	 	  	dash++;
	 	  }


	 	  if(Menu_Stage_1==1)
	 		  menu = 1;

	 	  while (menu){
	 		  getValueADC(3);
	 		  updateMenu();
	 		  execCommandSelect();

	 	  }

	 	  Temperature = DS18B20_Get_Temperature();

	 	  sprintf(string_buff, "%.1f", Temperature);

	 	  HAL_Delay(50);


	 	  ssd1306_SetCursor(81, 7);

	 	  ssd1306_WriteString(string_buff, Font_7x10,White);

	 	  sprintf(string_buff, "%.2f", waterVolume);
	 	  ssd1306_SetCursor(81, 28);
	 	  ssd1306_WriteString(string_buff, Font_7x10,White);

	 	  sprintf(string_buff, "%d", time);
	 	  ssd1306_SetCursor(81, 48);
	 	  ssd1306_WriteString(string_buff, Font_7x10,White);
	 	  ssd1306_UpdateScreen();

	 	  waterVolume -= water/1000;
	 	  water = 0;





	 	  //fillPercents(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2);

	 	  //draw time on screen



	 	  //readPinState(GPIOA, GPIO_PIN_4);
	 	  //LED change behaivor

	 	 if (led){
	 	 		  if (Temperature > MIN_TEMP && Temperature < MAX_TEMP )
	 	 		  	  {
	 	 		  		  //green
	 	 		  		  HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
	 	 		  		  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
	 	 		  		  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
	 	 		  		  ssd1306_DrawTempIcon(2);
	 	 		  	  }
	 	 		  	  else if(Temperature >= MAX_TEMP )
	 	 		  	  {
	 	 		  		  //red
	 	 		  		  HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
	 	 		  		  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
	 	 		  		  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
	 	 		  		  ssd1306_DrawTempIcon(0);

	 	 		  	  }
	 	 		  	  else
	 	 		  	  {
	 	 		  		//blue
	 	 		  		  HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
	 	 		  		  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
	 	 		  		  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
	 	 		  		  ssd1306_DrawTempIcon(1);
	 	 		  	  }
	 	 	  }
	 	 	  else{
	 	 		  HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
	 	 		  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
	 	 		  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
	 	 		  ssd1306_DrawTempIcon(3);
	 	 	  }

	 	  if(alarm){
	 		  playBuzzer();
	 		  setAlarm(hour);
	 		  //resetTime();
	 		  alarm = 0;
	 	  }
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin){



	if (GPIO_Pin == BUTTON_Pin){
		if(Menu_Stage_1 == 0 && Menu_Stage_2 == 0){
			Menu_Stage_1 = 1;
			return;
		}
	    if(Menu_Stage_1 == 1 && Menu_Stage_2 == 0){
			Menu_Stage_2 = 1;
			return;
		}
		if(Menu_Stage_1 == 1 && Menu_Stage_2 == 1){
			Menu_Stage_2 = 0;
			return;
		}
	}



}



void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{

	alarm = 1;

}


/* USER CODE END 4 */

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
