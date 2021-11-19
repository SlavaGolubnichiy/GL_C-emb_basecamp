/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

#define PWM_FREQ_CALC_PLUS_5KHZ 1

#if defined(PWM_FREQ_CALC_PLUS_5KHZ)
	#define PWM_FREQ_CALC_MUL2 !PWM_FREQ_CALC_PLUS_5KHZ
#else
	#define PWM_FREQ_CALC_MUL2 1
#endif


/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

static Led output_led = led_red;

Led get_output_led(void)
{
	return output_led;
}

uint32_t current_freq = 1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
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

// do not use 0 or UINT16_MAX as limits
uint32_t saturate(uint32_t val, uint32_t min_limit, uint32_t max_limit)
{
    return min(max(val, min_limit), max_limit);
}




/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim4;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  HAL_RCC_NMI_IRQHandler();
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

	// pin_C6, pin_C8, pin_C9 interrupts handler
	
	GPIO_PinState pin_c6_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6);
	GPIO_PinState pin_c8_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8);
	GPIO_PinState pin_c9_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9);
	
	if (!pin_c6_state) // SWT4
	{
		#if PWM_FREQ_CALC_MUL2
		{
			// freq *2 (ARR/2 = period/2)
			uint16_t pwm_period_delta = (TIM4->ARR & 0x0000FFFF) * 0.05;
			pwm_period_delta = (pwm_period_delta > 0) ? pwm_period_delta : 1;
			TIM4->ARR = (TIM4->ARR >> 1) + 1; 	// period /2
		}
		#else
		{
			// freq +5 kHz
			
			// res_freq = result timer interrupt frequency in Hz
			// F0 		= sys_clk_freq_hz
			// psc0 	= perfect_psc
			// PSC 		= timer->Prescaler value
			// ARR 		= timer->Auto_Reload_Register value
			
			// Goal: Calculate ARR value to change PWM frequency on 5 kHz.
			// Solution:
			
			// BASE FORMULA: res_freq = F0 / (PSC+1) / (ARR+1);				*1*
			
			// Changing ARR only, I need min_res_freq to be 1 Hz.
			// Minimum freq. is reaced when ARR is ARR_MAX (2^16-1). 		*2*
			
			// From *1* and *2*:
			// min_res_freq = F0 / (psc0 + 1) / (ARR_MAX+1)
			// psc0 = F0 / min_res_freq / (ARR_MAX+1) - 1;
			// psc0 = F0 / 1 / (2^16-1+1) - 1;								[1]
			
			// Calculating max_res_freq using psc0.
			// max_res_freq is reached when ARR is ARR_MIN (0).				*3*
			
			// From *1* and *4*:
			// max_freq = F0 / (psc0+1) / (ARR_MIN+1);
			// max_freq = F0 / (psc0+1) / (0+1);							[2]
			
			// max_res_freq := floor(max_freq, step=5001);					[3]
			// for example: floor(65573, 5001) = 65001.
			
			// So, we have res_freq range: from min_res_freq to max_res_freq.
			
			// Calculating ARR value to get new current_freq
			// new current_freq є { min_res_freq, min_res_freq+5kHz, ... , max_res_freq }.
			// new_current_freq = saturate(current_freq+5000, min_res_freq, max_res_freq);
			
			// From *1*:
			// ARR = F0 / (psc0+1) / res_freq;
			// new_ARR = F0 / (psc0+1) / current_freq;						[4]
			
			// Done!
			
			uint32_t min_res_freq = 1;
			uint16_t ARR_MAX = 0xFFFF;	// 2^16-1
			uint16_t ARR_MIN = 0;
			uint32_t freq_delta = 100;

			uint32_t F0 = HAL_RCC_GetSysClockFreq();
			uint32_t psc0 = F0 / min_res_freq / ((uint32_t)ARR_MAX + 1) - 1;	// [1]
			uint32_t max_freq = F0 / (psc0+1) / ((uint32_t)ARR_MIN+1);		// [2]
			uint32_t max_res_freq;
			if ((max_freq % 10000) < 5001)	// floor(max_freq, 5001);
			{
				max_res_freq = max_freq - (max_freq % 10000) + 1;
			}
			else
			{
				max_res_freq = max_freq - (max_freq % 10000) + 5001;	// 0x1389 = 5001 	(5001 Hz delta)
			}
  
			current_freq = saturate(current_freq+freq_delta, min_res_freq, max_res_freq);
			//uint16_t arr = F0 / (psc0+1) / current_freq;
			TIM4->ARR = F0 / (psc0+1) / current_freq;
			
		}
		#endif
	}
	if (!pin_c8_state) // SWT5
	{
		#if PWM_FREQ_CALC_MUL2
			// freq /2 (ARR*2 = period*2)
			uint16_t pwm_period_delta = (TIM4->ARR & 0x0000FFFF) * 0.05;
			pwm_period_delta = (pwm_period_delta > 0) ? pwm_period_delta : 1;
			TIM4->ARR = (TIM4->ARR << 1) + 1; 	// period *2
		#else
			// freq -5 kHz
			uint32_t min_res_freq = 1;
			uint16_t ARR_MAX = 0xFFFF;	// 2^16-1
			uint16_t ARR_MIN = 0;
			uint32_t freq_delta = 100;

			uint32_t F0 = HAL_RCC_GetSysClockFreq();
			uint32_t psc0 = F0 / min_res_freq / ((uint32_t)ARR_MAX + 1) - 1;	// [1]
			uint32_t max_freq = F0 / (psc0+1) / ((uint32_t)ARR_MIN+1);			// [2]
			uint32_t max_res_freq;
			if ((max_freq % 10000) < 5001)	// floor(max_freq, 5001);
			{
				max_res_freq = max_freq - (max_freq % 10000) + 1;
			}
			else
			{
				max_res_freq = max_freq - (max_freq % 10000) + 5001;	// 0x1389 = 5001 	(5001 Hz delta)
			}
		  
			current_freq = saturate(current_freq-freq_delta, min_res_freq, max_res_freq);
			//uint16_t arr = F0 / (psc0+1) / current_freq;
			TIM4->ARR = F0 / (psc0+1) / current_freq;
		#endif
	}
	if (!pin_c9_state) // SWT3
	{
		// duty cycle -5 %
		uint16_t pwm_duty_cycle_delta = TIM4->CCR1 * 0.05;	// ok if all TIM4->CCRx are equal all the time
		pwm_duty_cycle_delta = (pwm_duty_cycle_delta > 0) ? pwm_duty_cycle_delta : 1;	// ok if all TIM4->CCRx are equal all the time
		uint16_t pwm_duty_cycle_new_value = saturate(TIM4->CCR1 - pwm_duty_cycle_delta, 1, UINT16_MAX-1);
		
		TIM4->CCR1 = pwm_duty_cycle_new_value; 	// duty cycle -1 %
		TIM4->CCR2 = pwm_duty_cycle_new_value;
		TIM4->CCR3 = pwm_duty_cycle_new_value;
		TIM4->CCR4 = pwm_duty_cycle_new_value;
	}
	else
	{
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_9);
	}
	
	
	
  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

	// pin_A15, pin_C11 interrupts handler
		
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_RESET)	// SWT2
	{
		// when pressed - change pwm output (pd12, pd13, pd14, pd15 or none)
		
		// to check LEDx <-> TIM4_CH_x correspondence - see Core/Src/main.c/note
		switch(output_led)
		{
			case no_led:
				HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);	// switch to led_red
				break;
			case led_red:
				HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
				HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);	// switch to led_orange
				break;
			case led_orange:
				HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
				HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);	// switch to led_green
				break;
			case led_green:
				HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
				HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);	// switch to led_blue
				break;
			case led_blue:
				HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);	// switch to no_led
				HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
				HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
				HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
				break;
			default:
				output_led = led_red;
		}
		
		output_led = ++output_led % get_led_states_num();
		
	}
	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11) == GPIO_PIN_RESET)	// SWT1
	{
		// duty cycle +5 %
		
		// ok if all TIM4->CCRx are equal all the time
		uint16_t pwm_duty_cycle_delta = TIM4->CCR1 * 0.05;
		pwm_duty_cycle_delta = (pwm_duty_cycle_delta > 0) ? pwm_duty_cycle_delta : 1;
		uint16_t pwm_duty_cycle_new_value = saturate(TIM4->CCR1 + pwm_duty_cycle_delta, 1, UINT16_MAX-1);
		
		TIM4->CCR1 = pwm_duty_cycle_new_value; 	// duty cycle -1 %
		TIM4->CCR2 = pwm_duty_cycle_new_value;
		TIM4->CCR3 = pwm_duty_cycle_new_value;
		TIM4->CCR4 = pwm_duty_cycle_new_value;
	}
	
	
  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
