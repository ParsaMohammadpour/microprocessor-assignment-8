/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
enum State{
	BEFOR_START,
	CHECKING,
	OK,
	WARNING,
	DANGER,
	COOLING,
	TURN_OFF
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
////////////////////////////////////////////////////// LED //////////////////////////////////////////////////////////
#define LED_port GPIOB
#define green GPIO_PIN_3
#define orange GPIO_PIN_7
#define red GPIO_PIN_6

////////////////////////////////////////////////////// left 7-Segment //////////////////////////////////////////////////////////
#define left_port GPIOA

#define la GPIO_PIN_2
#define lb GPIO_PIN_3
#define lc GPIO_PIN_4
#define ld GPIO_PIN_5
#define le GPIO_PIN_6
#define lf GPIO_PIN_7
#define lg GPIO_PIN_8

////////////////////////////////////////////////////// right 7-Segment //////////////////////////////////////////////////////////
#define right_port GPIOB

#define ra GPIO_PIN_8
#define rb GPIO_PIN_9
#define rc GPIO_PIN_10
#define rd GPIO_PIN_12
#define re GPIO_PIN_13
#define rf GPIO_PIN_14
#define rg GPIO_PIN_15

////////////////////////////////////////////////////// MASK //////////////////////////////////////////////////////////
#define SET1(x) (1ul << x)
#define SET0(x) (~SET1(x))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static volatile uint16_t temprature = 0;
static volatile int tempCounter = 0;
static volatile enum State status = BEFOR_START;
static volatile uint16_t externalSig = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void printString(char* s);
void printNumber(uint32_t input);
void newLine(void);
void setLeftDigit(int number);
void setRightDigit(int number);
void set7Segment(int number);
uint16_t getTemprature(void);
uint16_t getExternalValue(void);
void turnOffLEDs(void);
void stopBlinking(void);
void setDAC(void);
void newTempratureHandler(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void OK_state(void);
void WARNING_state(void);
void DANGER_state(void);
void TURN_OF_state(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void printString(char* s){
    HAL_UART_Transmit(&huart1,(uint8_t*) s , strlen(s), HAL_MAX_DELAY);
}
void printNumber(uint32_t input){
  char number[10]; 
  sprintf(number, "%u", input);
  HAL_UART_Transmit(&huart1,(uint8_t*) number , 10, HAL_MAX_DELAY);
}
void newLine(void){
	char* s ="\n\r";
	HAL_UART_Transmit(&huart1,(uint8_t*) s , 2, HAL_MAX_DELAY);
}
void setLeftDigit(int number){
	switch(number){
		case -1:
			HAL_GPIO_WritePin(left_port, la, 0);
			HAL_GPIO_WritePin(left_port, lb, 0);
			HAL_GPIO_WritePin(left_port, lc, 0);
			HAL_GPIO_WritePin(left_port, ld, 0);
			HAL_GPIO_WritePin(left_port, le, 0);
			HAL_GPIO_WritePin(left_port, lf, 0);
			HAL_GPIO_WritePin(left_port, lg, 0);
		break;
		case 0: 
			HAL_GPIO_WritePin(left_port, la, 1);
			HAL_GPIO_WritePin(left_port, lb, 1);
			HAL_GPIO_WritePin(left_port, lc, 1);
			HAL_GPIO_WritePin(left_port, ld, 1);
			HAL_GPIO_WritePin(left_port, le, 1);
			HAL_GPIO_WritePin(left_port, lf, 1);
			HAL_GPIO_WritePin(left_port, lg, 0);
		break;
		case 1: 
			HAL_GPIO_WritePin(left_port, la, 0);
			HAL_GPIO_WritePin(left_port, lb, 1);
			HAL_GPIO_WritePin(left_port, lc, 1);
			HAL_GPIO_WritePin(left_port, ld, 0);
			HAL_GPIO_WritePin(left_port, le, 0);
			HAL_GPIO_WritePin(left_port, lf, 0);
			HAL_GPIO_WritePin(left_port, lg, 0);
		break;
		case 2:
			HAL_GPIO_WritePin(left_port, la, 1);
			HAL_GPIO_WritePin(left_port, lb, 1);
			HAL_GPIO_WritePin(left_port, lc, 0);
			HAL_GPIO_WritePin(left_port, ld, 1);
			HAL_GPIO_WritePin(left_port, le, 1);
			HAL_GPIO_WritePin(left_port, lf, 0);
			HAL_GPIO_WritePin(left_port, lg, 1);
		break;
		case 3:
			HAL_GPIO_WritePin(left_port, la, 1);
			HAL_GPIO_WritePin(left_port, lb, 1);
			HAL_GPIO_WritePin(left_port, lc, 1);
			HAL_GPIO_WritePin(left_port, ld, 1);
			HAL_GPIO_WritePin(left_port, le, 0);
			HAL_GPIO_WritePin(left_port, lf, 0);
			HAL_GPIO_WritePin(left_port, lg, 1);
		break;
		case 4:
			HAL_GPIO_WritePin(left_port, la, 0);
			HAL_GPIO_WritePin(left_port, lb, 1);
			HAL_GPIO_WritePin(left_port, lc, 1);
			HAL_GPIO_WritePin(left_port, ld, 0);
			HAL_GPIO_WritePin(left_port, le, 0);
			HAL_GPIO_WritePin(left_port, lf, 1);
			HAL_GPIO_WritePin(left_port, lg, 1);
		break;
		case 5:
			HAL_GPIO_WritePin(left_port, la, 1);
			HAL_GPIO_WritePin(left_port, lb, 0);
			HAL_GPIO_WritePin(left_port, lc, 1);
			HAL_GPIO_WritePin(left_port, ld, 1);
			HAL_GPIO_WritePin(left_port, le, 0);
			HAL_GPIO_WritePin(left_port, lf, 1);
			HAL_GPIO_WritePin(left_port, lg, 1);
		break;
		case 6:
			HAL_GPIO_WritePin(left_port, la, 1);
			HAL_GPIO_WritePin(left_port, lb, 0);
			HAL_GPIO_WritePin(left_port, lc, 1);
			HAL_GPIO_WritePin(left_port, ld, 1);
			HAL_GPIO_WritePin(left_port, le, 1);
			HAL_GPIO_WritePin(left_port, lf, 1);
			HAL_GPIO_WritePin(left_port, lg, 1);
		break;
		case 7:
			HAL_GPIO_WritePin(left_port, la, 1);
			HAL_GPIO_WritePin(left_port, lb, 1);
			HAL_GPIO_WritePin(left_port, lc, 1);
			HAL_GPIO_WritePin(left_port, ld, 0);
			HAL_GPIO_WritePin(left_port, le, 0);
			HAL_GPIO_WritePin(left_port, lf, 0);
			HAL_GPIO_WritePin(left_port, lg, 0);
		break;
		case 8:
			HAL_GPIO_WritePin(left_port, la, 1);
			HAL_GPIO_WritePin(left_port, lb, 1);
			HAL_GPIO_WritePin(left_port, lc, 1);
			HAL_GPIO_WritePin(left_port, ld, 1);
			HAL_GPIO_WritePin(left_port, le, 1);
			HAL_GPIO_WritePin(left_port, lf, 1);
			HAL_GPIO_WritePin(left_port, lg, 1);
		break;
		case 9:
			HAL_GPIO_WritePin(left_port, la, 1);
			HAL_GPIO_WritePin(left_port, lb, 1);
			HAL_GPIO_WritePin(left_port, lc, 1);
			HAL_GPIO_WritePin(left_port, ld, 1);
			HAL_GPIO_WritePin(left_port, le, 0);
			HAL_GPIO_WritePin(left_port, lf, 1);
			HAL_GPIO_WritePin(left_port, lg, 1);
		break;
	}
}
void setRightDigit(int number){
	switch(number){
		case -1:
			HAL_GPIO_WritePin(right_port, ra, 0);
			HAL_GPIO_WritePin(right_port, rb, 0);
			HAL_GPIO_WritePin(right_port, rc, 0);
			HAL_GPIO_WritePin(right_port, rd, 0);
			HAL_GPIO_WritePin(right_port, re, 0);
			HAL_GPIO_WritePin(right_port, rf, 0);
			HAL_GPIO_WritePin(right_port, rg, 0);
		break;
		case 0: 
			HAL_GPIO_WritePin(right_port, ra, 1);
			HAL_GPIO_WritePin(right_port, rb, 1);
			HAL_GPIO_WritePin(right_port, rc, 1);
			HAL_GPIO_WritePin(right_port, rd, 1);
			HAL_GPIO_WritePin(right_port, re, 1);
			HAL_GPIO_WritePin(right_port, rf, 1);
			HAL_GPIO_WritePin(right_port, rg, 0);
		break;
		case 1: 
			HAL_GPIO_WritePin(right_port, ra, 0);
			HAL_GPIO_WritePin(right_port, rb, 1);
			HAL_GPIO_WritePin(right_port, rc, 1);
			HAL_GPIO_WritePin(right_port, rd, 0);
			HAL_GPIO_WritePin(right_port, re, 0);
			HAL_GPIO_WritePin(right_port, rf, 0);
			HAL_GPIO_WritePin(right_port, rg, 0);
		break;
		case 2:
			HAL_GPIO_WritePin(right_port, ra, 1);
			HAL_GPIO_WritePin(right_port, rb, 1);
			HAL_GPIO_WritePin(right_port, rc, 0);
			HAL_GPIO_WritePin(right_port, rd, 1);
			HAL_GPIO_WritePin(right_port, re, 1);
			HAL_GPIO_WritePin(right_port, rf, 0);
			HAL_GPIO_WritePin(right_port, rg, 1);
		break;
		case 3:
			HAL_GPIO_WritePin(right_port, ra, 1);
			HAL_GPIO_WritePin(right_port, rb, 1);
			HAL_GPIO_WritePin(right_port, rc, 1);
			HAL_GPIO_WritePin(right_port, rd, 1);
			HAL_GPIO_WritePin(right_port, re, 0);
			HAL_GPIO_WritePin(right_port, rf, 0);
			HAL_GPIO_WritePin(right_port, rg, 1);
		break;
		case 4:
			HAL_GPIO_WritePin(right_port, ra, 0);
			HAL_GPIO_WritePin(right_port, rb, 1);
			HAL_GPIO_WritePin(right_port, rc, 1);
			HAL_GPIO_WritePin(right_port, rd, 0);
			HAL_GPIO_WritePin(right_port, re, 0);
			HAL_GPIO_WritePin(right_port, rf, 1);
			HAL_GPIO_WritePin(right_port, rg, 1);
		break;
		case 5:
			HAL_GPIO_WritePin(right_port, ra, 1);
			HAL_GPIO_WritePin(right_port, rb, 0);
			HAL_GPIO_WritePin(right_port, rc, 1);
			HAL_GPIO_WritePin(right_port, rd, 1);
			HAL_GPIO_WritePin(right_port, re, 0);
			HAL_GPIO_WritePin(right_port, rf, 1);
			HAL_GPIO_WritePin(right_port, rg, 1);
		break;
		case 6:
			HAL_GPIO_WritePin(right_port, ra, 1);
			HAL_GPIO_WritePin(right_port, rb, 0);
			HAL_GPIO_WritePin(right_port, rc, 1);
			HAL_GPIO_WritePin(right_port, rd, 1);
			HAL_GPIO_WritePin(right_port, re, 1);
			HAL_GPIO_WritePin(right_port, rf, 1);
			HAL_GPIO_WritePin(right_port, rg, 1);
		break;
		case 7:
			HAL_GPIO_WritePin(right_port, ra, 1);
			HAL_GPIO_WritePin(right_port, rb, 1);
			HAL_GPIO_WritePin(right_port, rc, 1);
			HAL_GPIO_WritePin(right_port, rd, 0);
			HAL_GPIO_WritePin(right_port, re, 0);
			HAL_GPIO_WritePin(right_port, rf, 0);
			HAL_GPIO_WritePin(right_port, rg, 0);
		break;
		case 8:
			HAL_GPIO_WritePin(right_port, ra, 1);
			HAL_GPIO_WritePin(right_port, rb, 1);
			HAL_GPIO_WritePin(right_port, rc, 1);
			HAL_GPIO_WritePin(right_port, rd, 1);
			HAL_GPIO_WritePin(right_port, re, 1);
			HAL_GPIO_WritePin(right_port, rf, 1);
			HAL_GPIO_WritePin(right_port, rg, 1);
		break;
		case 9:
			HAL_GPIO_WritePin(right_port, ra, 1);
			HAL_GPIO_WritePin(right_port, rb, 1);
			HAL_GPIO_WritePin(right_port, rc, 1);
			HAL_GPIO_WritePin(right_port, rd, 1);
			HAL_GPIO_WritePin(right_port, re, 0);
			HAL_GPIO_WritePin(right_port, rf, 1);
			HAL_GPIO_WritePin(right_port, rg, 1);
		break;
	}
}
void set7Segment(int number){
	if(number == -1){
		setLeftDigit(-1);
		setRightDigit(-1);
		return;
	}
	setLeftDigit(number / 10);
	setRightDigit(number % 10);
}


uint16_t getTemprature(void){
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	uint16_t temp = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	double res = (double)temp;
	res /= 4095;
	res *= 3.3;
	res *= 100;
	temp = res;
	return temp;
}
uint16_t getExternalValue(void){
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	uint16_t temp = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	return temp;
}
void turnOffLEDs(void){
	HAL_GPIO_WritePin(LED_port, orange, 0);
	HAL_GPIO_WritePin(LED_port, green, 0);
	HAL_GPIO_WritePin(LED_port, red, 0);
}
void stopBlinking(void){
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
}
void setDAC(void){
	uint16_t signal = getExternalValue();
	signal = 4095 - signal;
	// the ADC_16 input is 16 bit, but our number is 12 bit. So we should convert our number to 16 bit number.
	// and for that, we simplified this conversion and only multiply it by 4.(or in other way, shift it 4 bit to left)
	signal <<= 4; 
	GPIOC -> ODR = signal;
}
void newTempratureHandler(void){
	printString("new Temprature Handler");
	newLine();
	if(temprature < 35){
		printString("OK status");
		newLine();
		OK_state();
	}else if(temprature < 46){
		printString("WARNING state");
		newLine();
		WARNING_state();
	}else {
		printString("DANGER status");
		newLine();
		DANGER_state();
	}
}
void OK_state(void){
	if(status != OK){
		turnOffLEDs();
		stopBlinking();
		HAL_GPIO_WritePin(LED_port, green, 1);
		status = OK;
	}
	//else => if we come to this part, it means our status was OK befor too. So we don't need to change it.
}
void WARNING_state(void){
	turnOffLEDs();
	stopBlinking();
	HAL_GPIO_WritePin(LED_port, orange, 1);
	status = WARNING;
}
void TURN_OFF_state(void){
	printString("Turn Off");
	newLine();
	stopBlinking();
	turnOffLEDs();
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	set7Segment(-1);
	HAL_Delay(500);
	stopBlinking();
	turnOffLEDs();
	status = TURN_OFF;
}
void DANGER_state(void){
	status = DANGER;
	stopBlinking();
	turnOffLEDs();
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	volatile int isPressed = 0;
	printString("Waiting for cooling button to be pressed");
	newLine();
	// This part is copied from stm32f4xx_hal.c file. because we want to wait if the cooling button is pressed 
	// during the this time.(deadline is about 500)
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	uint32_t tickstart = HAL_GetTick();
  uint32_t wait = 500; // our dedline is 500ms here.

  /* Add a freq to guarantee minimum wait */
  if (wait < HAL_MAX_DELAY)
  {
    wait += (uint32_t)(uwTickFreq);
  }

  while((HAL_GetTick() - tickstart) < wait)
  {
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 1) // my code
			isPressed = 1;                             // my code
  }
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	if(isPressed == 1){
		printString("Cooling Button was pressed");
		status = COOLING;
		stopBlinking();
		turnOffLEDs();
		HAL_GPIO_WritePin(LED_port, red, 1);
		for(int i = 0; i < 10; i++){
			tempCounter = 0;
			temprature = 0;
			HAL_TIM_Base_Start_IT(&htim2);
			while(tempCounter == 0)
				;
			while(tempCounter!= 0)
				;
			HAL_TIM_Base_Stop_IT(&htim2);
		}
		printString("Temprature:");
		newLine();
		printNumber(temprature);
		newLine();
		if(temprature < 35){
			OK_state();
		}else if(temprature < 46){
			WARNING_state();
		}else{
			printString("Temprature is still high");
			newLine();
			TURN_OFF_state();
		}
	}else{
		printString("Cooling Button wasn't pressed");
		newLine();
		TURN_OFF_state();
	}
}
/************************************************************** EXTERNAL INTRRUPT ******************************************************/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin & SET1(0)){
		if(status == BEFOR_START){
			status = CHECKING;
			stopBlinking();
			turnOffLEDs();
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
		}
	}else if(GPIO_Pin & SET1(2)){
		if(status == TURN_OFF){
			status = CHECKING;
			stopBlinking();
			turnOffLEDs();
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
		}
	}
}
void TIM2_IRQHandler(void){
	if(tempCounter == 0)
		temprature = 0;
	uint16_t a = getTemprature();
	temprature += a;
	tempCounter++;
	printString("Temprature Timer Intrrupt");
	newLine();
	printNumber(a);
	newLine();
	if(tempCounter >= 10){
		tempCounter = 0;
		temprature /= 10;
		set7Segment(temprature);
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
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
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
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	HAL_Delay(350); // wait for proteus to initialize (specialy for the thermometer to generate the appropriate voltage)
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(status == CHECKING){
			tempCounter = 0;
			temprature = 0;
			HAL_TIM_Base_Start_IT(&htim2);// activing tim2_irqHandler
			while(tempCounter == 0) // we wait until we know that we have started finding the temprature.
				;
			while(tempCounter != 0) // wait until we find the temprature for the first time
				;
			HAL_TIM_Base_Stop_IT(&htim2); // disabling tim2_irqHandler
			newTempratureHandler();
		}else if(status == OK || status == WARNING){
			tempCounter = 0;
			temprature = 0;
			HAL_TIM_Base_Start_IT(&htim2);// activing tim2_irqHandler
			while(tempCounter == 0){ // we wait until we know that we have started finding the temprature.
				HAL_TIM_Base_Stop_IT(&htim2);
				setDAC();
				HAL_TIM_Base_Start_IT(&htim2);
			}
			while(tempCounter != 0){ // wait until we find the temprature for the first time
				HAL_TIM_Base_Stop_IT(&htim2);
				setDAC();
				HAL_TIM_Base_Start_IT(&htim2);
			}
			HAL_TIM_Base_Stop_IT(&htim2); // disabling tim2_irqHandler
			newTempratureHandler();
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
