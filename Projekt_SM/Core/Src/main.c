/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "eth.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "servo.h"
#include "math.h"
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

//Sinus voltage amplitude and its 100 samples
double A = 1.0;
uint16_t sine[] = {
#include "sinus.txt"
};
int i = 0;

//Potentiometer
uint32_t pot_value = 0;
uint32_t pot_read_from_DMA = 0;

//Sensor
uint32_t sensor_RMS = 0;
const int BUFFER_SIZE = 100;
uint32_t sensor_values_buffer[100];
int current_sample = 0;
uint32_t sensor_read_from_DMA = 0;
int calculateRMS = 0;


//Variable used to handle lcd and servo with callback periods
int ifDisplay = 0;
int ifServo = 0;

//Initialize the lcd display
Lcd_PortType ports[] = {
	D4_GPIO_Port, D5_GPIO_Port, D6_GPIO_Port, D7_GPIO_Port
};
Lcd_PinType pins[] = {D4_Pin, D5_Pin, D6_Pin, D7_Pin};
//Handler for lcd
Lcd_HandleTypeDef my_lcd;

//Variables for the servo
int angle = 0;
servo_t servo1;

//Variables for UART
#define MESSAGE_MAX_SIZE 8
char receivedMessage[MESSAGE_MAX_SIZE] = {0};
char savedMessage[MESSAGE_MAX_SIZE] = {0};
char ledColor = 0;
int ifUART = 0;
int UART_status = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

//Scale samples values to set amplitude
void Set_Sinus_Values()
{
	for(int j = 0; j < 100; j++) {
		sine[j] = (float)sine[j] * A * 2.0 / 3.3;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM2) { //If interrupt comes from timer 2
		ifDisplay = 1;
	}
	if(htim->Instance == TIM3) { //If interrupt comes from timer 3
		ifServo=1;
	}
	if(htim->Instance == TIM4){ //If interrupt comes from timer 4
		//HAL_GPIO_WritePin(_GPIO_Port, _Pin, GPIO_PIN_SET);
		i += 1;
		if (i >= 50) {
			i = 0;
		}
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, sine[2*i]);
		//HAL_GPIO_WritePin(_GPIO_Port, _Pin, GPIO_PIN_RESET);
	}
	if(htim->Instance == TIM5){ //If interrupt comes from timer 5
		pot_value = pot_read_from_DMA;
	}
	if(htim->Instance == TIM6){ //If interrupt comes from timer 6
		if(current_sample < BUFFER_SIZE) {
			sensor_values_buffer[current_sample] = sensor_read_from_DMA;
			current_sample++;
		}
		else {
			calculateRMS = 1;
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//Listening again
	UART_status = HAL_UART_Receive_IT(&huart6, (uint8_t*)receivedMessage, 7);

	//Save received data
	for (int i = 0; i < MESSAGE_MAX_SIZE; i++) {
		savedMessage[i] = receivedMessage[i];
	}
	receivedMessage[0] = '\0';

	//Set flag to action
	ifUART = 1;

}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_DAC_Init();
  MX_ADC2_Init();
  MX_TIM5_Init();
  MX_USART6_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  //Start tim2 to write data from joystick to the lcd display
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);

  //Create lcd
  my_lcd = Lcd_create(ports, pins, RS_GPIO_Port, RS_Pin, Enable_GPIO_Port, Enable_Pin, LCD_4_BIT_MODE);

  //Initialize micro servo sg90
  Servo_Init(&servo1, &htim1, TIM_CHANNEL_4);

  //Sinus
  Set_Sinus_Values();
  HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
  HAL_TIM_Base_Start_IT(&htim4);

  //Interrupts for DMA adc1 (potentiometer)
  HAL_TIM_Base_Start_IT(&htim5);

  //Interrupts for DMA adc2 (sensor)
  HAL_TIM_Base_Start_IT(&htim6);

  //Start ADC conversion from the potentiometer and sensor
  HAL_ADC_Start_DMA(&hadc1, &pot_read_from_DMA, 1);
  HAL_ADC_Start_DMA(&hadc2, &sensor_read_from_DMA, 1);

  //UART listening
  UART_status = HAL_UART_Receive_IT(&huart6, (uint8_t*)receivedMessage, 7);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(ifDisplay == 1) {
		  Lcd_cursor(&my_lcd, 0, 0);
		  Lcd_string(&my_lcd, "                ");
		  Lcd_cursor(&my_lcd, 0, 0);
		  Lcd_string(&my_lcd, "Sens: ");
		  Lcd_int(&my_lcd, sensor_RMS);
		  Lcd_cursor(&my_lcd, 1, 0);
		  Lcd_string(&my_lcd, "                ");
		  Lcd_cursor(&my_lcd, 1, 0);
		  Lcd_string(&my_lcd, "Pot: ");
		  Lcd_int(&my_lcd, pot_value);
		  ifDisplay = 0;
	  }
	  if(ifServo)
	  {
		  angle = pot_value * 150 / 4095 + 30;
		  Servo_SetAngle(&servo1, angle);
	  }
	  if(calculateRMS) {
			int squares_sum = 0;
			for (int i = 0; i < BUFFER_SIZE; i++) {
				squares_sum += pow(sensor_values_buffer[i], 2);
			}
			sensor_RMS = sqrt(squares_sum / 100.0);
			current_sample = 0;
			calculateRMS = 0;
	  }

	  //LEDs' control by UART
	  if(ifUART) {
		  if (!strcmp(savedMessage, "LD_R_ON") || !strcmp(savedMessage, "LD_G_ON") || !strcmp(savedMessage, "LD_B_ON")) {
			  sscanf((char*)savedMessage, "LD_%c_ON ", &ledColor);
			  switch(ledColor) {
			  case 'G':
				  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
			  case 'B':
				  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
			  case 'R':
				  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
			  }
			  savedMessage[0] = '\0';
		  }
		  else if (!strcmp(savedMessage, "LD_R_OFF") || !strcmp(savedMessage, "LD_G_OFF") || !strcmp(savedMessage, "LD_B_OFF")) {
			  sscanf((char*)savedMessage, "LD_%c_OFF", &ledColor);
			  switch(ledColor) {
			  case 'G':
				HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
			  case 'B':
				HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
			  case 'R':
				HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
			  }
			  savedMessage[0] = '\0';
		  }

		  ifUART = 0;
	  }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 108;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_USART6
                              |RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 96;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV4;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Usart6ClockSelection = RCC_USART6CLKSOURCE_PCLK2;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLLSAIP;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
