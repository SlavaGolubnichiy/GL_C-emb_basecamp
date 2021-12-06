/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

// todo

// implement internal temperature sensor read
// implement external temperature sensor read

// todo end.



/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdbool.h>

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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;

TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */



static const uint8_t led_states_num = 5;

const uint8_t get_led_states_num(void)
{
	return led_states_num;
}

uint16_t adc1_ar[2];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC3_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



uint32_t min(uint32_t val1, uint32_t val2)
{
	return (val1 < val2) ? val1 : val2;
}

uint32_t max(uint32_t val1, uint32_t val2)
{
	return (val1 > val2) ? val1 : val2;
}

uint16_t saturate_uint16(int32_t val, uint16_t min_lim, uint16_t max_lim)
{
	bool below_min = val < (int32_t)min_lim;
	bool above_max = val > (int32_t)max_lim;
	return (below_min * min_lim + (!below_min && !above_max) * val + above_max * max_lim);
}

double normalize(uint16_t src, uint16_t src_range_min, uint16_t src_range_size)
{
	if (src_range_size)
	{
		return 0.0 + (double)(src - src_range_min) / (double)src_range_size;
	}
	else
	{
		return 1;
	}
}

uint16_t map(int16_t src, uint16_t src_range_min, uint16_t src_range_size, uint16_t res_range_min, uint16_t res_range_size)
{
	double normalized = normalize(src, src_range_min, src_range_size);	// normalized to 0-1 range (src_min = 0.0, src_max = 1.0, >src_max = >1.0)
	uint32_t mapped = (uint32_t) ((double)res_range_min + normalized * (double)res_range_size);
	return mapped;
}





void ADC1_Select_CH9()	// external temp. sensor channel
{
	ADC_ChannelConfTypeDef sConfig = {0};
	
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	*/
	sConfig.Channel = ADC_CHANNEL_9;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

void ADC1_Select_CHTemp()	// internal temp. sensor channel
{
	ADC_ChannelConfTypeDef sConfig = {0};
	
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	*/
	sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
	sConfig.Rank = 2;
	sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}






void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	uint16_t adc_value_min = 0;
	uint16_t adc_value_max = 4095;
	uint32_t adc3_value = 0;
	
	if(hadc->Instance == ADC3)
	{
		// blue led - potentiometer input
		adc3_value = HAL_ADC_GetValue(&hadc3);
		TIM4->CCR4 = (uint32_t) map(adc3_value, adc_value_min, adc_value_max, 0, 10);
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
  MX_TIM4_Init();
  MX_ADC3_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

	// note: HARDWARE SETUP

    // pin_D12 = LED green	- TIM_CH_1
    // pin_D13 = LED orange	- TIM_CH_2
    // pin_D14 = LED red	- TIM_CH_3
    // pin_D15 = LED blue	- TIM_CH_4
      
    // pin_A15 	= button SWT2 (center)	- INT_SWT2
    // pin_C6	= button SWT4 (up)		- INT_SWT4
    // pin_C8	= button SWT5 (down)	- INT_SWT5
    // pin_C9	= button SWT3 (left)	- INT_SWT3
    // pin_C11 	= button SWT1 (right)	- INT_SWT1

  	// pin_GND 	= potentiometer.pin_left
  	// pin_3V	= potentiometer.pin_right
  	// pin_A3	= potentiometer.pin_middle
  
  	// note: SOFTWARE SETUP

  	// files with user-code:
  	// - $ProjectDir/Core/Src/main.h
  	// - $ProjectDir/Core/Src/main.c
  	// - $ProjectDir/Core/Src/stm32f4xx_it.c
  
  	// xxx !!!
  	// * When re-generated code:
    // - in $ProjectDir/Core/Src/main.c -> MX_ADC1_Init() -> hadc1.Init.NbrOfConversion = 2; 	
  	//		=> 	change value to 1 (manually after regenerate !);
  	
  	// note: adc with interrupts config
  	// * When setting up adc with interrupt mode, set 
  	// 		ADCx -> Parameter settings -> Sampling Time = 480 cyles, otherwise
  	//		interrupts may not work
  
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  HAL_TIM_Base_Start(&htim4);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);		// enable led_green
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);		// enable led_orange
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);		// enable led_blue
  //HAL_ADC_Start_IT(&hadc1);
  //HAL_ADC_Start_IT(&hadc2);
  HAL_ADC_Start_IT(&hadc3);
  
  // caculate and set initial CCR value (initial duty cycle)
  uint32_t sys_clk_hz = HAL_RCC_GetSysClockFreq();
  const uint16_t CCR_min = 1;
  uint16_t CCR_max = (uint16_t) (sys_clk_hz / TIM4->PSC);
  uint32_t target_CCR = (uint32_t) map(0, 0, 100, CCR_min, CCR_max);
  TIM4->CCR1 = target_CCR;
  TIM4->CCR2 = target_CCR;
  TIM4->CCR4 = target_CCR;
  
  TIM4->ARR = 7;	// result pwm frequency = 55.56 Hz
  
  // ADC3_CH3 variables - potentiometer [single-channel polling]
  uint16_t adc_value_min = 0;
  uint16_t adc_value_max = 4095;
  uint32_t adc3_value = 0;
  volatile HAL_StatusTypeDef adc3_poll_result;

  // ADC1_CH9 - external temp. sensor variables
  const uint16_t ext_temp_sensor_adc1_value_min = 1800;	// (approx.) value of room temperature
  const uint16_t ext_temp_sensor_adc1_value_max = 4095;	// max value
  
  // ADC1_CHTemp - internal temp. sensor variables
  const uint16_t int_temp_sensor_adc1_value_min = 1800;	// (approx.) value of room temperature
  const uint16_t int_temp_sensor_adc1_value_max = 4095;	// max value
  
  const uint16_t ext_temp_sensor_adc1_value_min_inverted = 1650;
  const uint16_t int_temp_sensor_adc1_value_min_inverted = 1650;
  
  
  
	while (1)
	{
		
		// ADC3_CH3 - potentiometer [single-channel polling]
		/*
		adc3_poll_result = HAL_ADC_PollForConversion(&hadc3, 1);
		if (adc3_poll_result == HAL_OK)
		{
			adc3_value = HAL_ADC_GetValue(&hadc3);
		}
		else
		{
			HAL_ADC_Start(&hadc3);
		}
		TIM4->CCR4 = (uint32_t) map(adc3_value, adc_value_min, adc_value_max, 0, 10);
		*/
		
		
		
		// ADC1_CH9 - external temp. sensor [multi-channel polling]
		ADC1_Select_CH9();
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 1000);
		adc1_ar[0] = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
		// green led -ext. temp. sensor
		TIM4->CCR1 = (uint32_t) map
		(
			adc_value_max - adc1_ar[0],		// inverting for more brightness when temp. rises
			ext_temp_sensor_adc1_value_min_inverted, 
			adc_value_max,
			0, 
			10
		);
		
		// ADC1_CHTemp - internal temp. sensor	[multi-channel polling]
		ADC1_Select_CHTemp();
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 1000);
		adc1_ar[1] = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
		// orange led - int. temp. sensor
		TIM4->CCR2 = (uint32_t) map
		(
			adc_value_max - adc1_ar[1],		// inverting for more brightness when temp. rises
			int_temp_sensor_adc1_value_min_inverted, 
			adc_value_max, 
			0, 
			10
		);
		
		HAL_Delay(10);
		
		
		
		
		
		
		
		
		
		
		
		// adc values test
		if (adc3_value < 1000)
		{
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_0);
		}
		else if (adc3_value < 2000)
		{
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_1);
		}
		else if (adc3_value < 3000)
		{
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_3);
		}
		else
		{
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_4);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 2;
  sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 16000;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 499;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 100;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC6 PC8 PC9 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD1 PD3 PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
