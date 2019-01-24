/**
******************************************************************************
* @file    low-power.c
* @author  ST
* @version V1.0.0
* @date    05-December-2018
* @brief   MCU Low Power API
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*   1. Redistributions of source code must retain the above copyright notice,
*      this list of conditions and the following disclaimer.
*   2. Redistributions in binary form must reproduce the above copyright notice,
*      this list of conditions and the following disclaimer in the documentation
*      and/or other materials provided with the distribution.
*   3. Neither the name of STMicroelectronics nor the names of its contributors
*      may be used to endorse or promote products derived from this software
*      without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
*/

/** @defgroup Low_power
* @{
*/

/** @addtogroup Low_power
* @{
*/


/* Includes -----------------------------------------------------------------*/
#include "main.h"
#include "cube_hal.h"
#include "dev/radio.h"
#include "radio-driver.h"
#include "low-power.h"
/*---------------------------------------------------------------------------*/
#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else /*!DEBUG*/
#define PRINTF(...)
#endif /*DEBUG*/

//Enable CHECK_CLK_F if you want to validate the argument to LP_sysclk_config()
#define CHECK_CLK_F 0
#if CHECK_CLK_F
#define VALIDATE_CLK_F(a) do{                                               \
  if (!((a) == 65 || (a) == 131 || (a) == 262 || (a) == 524                 \
      || (a) == 1048  || (a) == 2097  || (a) == 4194                        \
	  || (a) == 12000 || (a) == 16000  || (a) == 24000  || (a) == 32000 )) {\
	    printf("Error, unsupported clock frequency: %d\r\n", (a));          \
	    while(1);                                                           \
  }                                                                         \
} while (0)
#else /*!CHECK_CLK_F*/
#define VALIDATE_CLK_F(a) {}
#endif /*CHECK_CLK_F*/

#if MCU_LOW_POWER
/*---------------------------------------------------------------------------*/
/* Static local functions prototypes                                         */
static void _lp_clock_adjust(clock_time_t howmany);
static void _lp_sysclk_exit_stop(void);
static void _lp_peripheral_enable(void);
static void _lp_peripheral_disable(void);
/*---------------------------------------------------------------------------*/
/* Static local variables                                                    */
static uint32_t exti_imr_backup;
static uint32_t gpioa_moder, gpiob_moder, gpioc_moder;
static uint32_t gpioa_otyper, gpiob_otyper, gpioc_otyper;
static uint32_t gpioa_ospeedr, gpiob_ospeedr, gpioc_ospeedr;
static uint32_t gpioa_pupdr, gpiob_pupdr, gpioc_pupdr;
/*---------------------------------------------------------------------------*/
/* Extern variables                                                          */
extern RTC_HandleTypeDef RtcHandle;
extern volatile unsigned long seconds;
extern volatile clock_time_t ticks;
/*---------------------------------------------------------------------------*/
/* Public (global) variables                                                 */
int LP_interrupt_from_radio=0;
/*---------------------------------------------------------------------------*/
/* Public functions implementation                                           */
/*---------------------------------------------------------------------------*/
/**
* @brief  Let the system go in STOP mode for MCU and Standby mode for the radio.
* @param  stop_seconds the specified time in seconds for the system to stop.
* @retval None.
*/
void LP_enter_stop_mode(int stop_seconds)
{
	int stop_time =0;
#if DEBUG
	int fail=0;
#endif /*DEBUG*/
	printf("Entering STOP mode for about %d seconds.\r\n", stop_seconds);

	/*Put the radio in Standby*/
	subGHz_radio_driver.off();
	/*De Init all the peripheral to enter STOP mode*/
	_lp_peripheral_enable();

	HAL_PWREx_EnableUltraLowPower();
	HAL_PWREx_EnableFastWakeUp();

	exti_imr_backup = EXTI->IMR;
	EXTI->IMR=0x00000000;

	/* Disable Prefetch Buffer */
	__HAL_FLASH_PREFETCH_BUFFER_DISABLE();
	/* Disable 64-bit Access */
	__HAL_FLASH_ACC64_DISABLE();
	/* Disable FLASH during Sleep  */
	__HAL_FLASH_SLEEP_POWERDOWN_ENABLE();
	HAL_PWR_DisablePVD();
	HAL_SuspendTick();

	stop_time = stop_seconds * LP_WAKEUP_TIME_CONVERSION;
	while(stop_time>0){
		/* Disable Wake-up timer */
		if (HAL_OK != HAL_RTCEx_DeactivateWakeUpTimer(&RtcHandle)){
#if DEBUG
			fail = 1;
#endif /*DEBUG*/
			goto _fail;
		}

		/* Clear PWR wake up Flag */
		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

		/* Clear RTC Wake Up timer Flag */
		__HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&RtcHandle, RTC_FLAG_WUTF);

		/* Enable Wake-up timer */
		if(stop_time<=65535){
			if (HAL_OK != HAL_RTCEx_SetWakeUpTimer_IT(&RtcHandle, stop_time, RTC_WAKEUPCLOCK_RTCCLK_DIV16)) {
#if DEBUG
				fail = 2;
#endif /*DEBUG*/
				goto _fail;
			}
		}
		else{
			if (HAL_OK != HAL_RTCEx_SetWakeUpTimer_IT(&RtcHandle, 0xffff , RTC_WAKEUPCLOCK_RTCCLK_DIV16)){
#if DEBUG
				fail = 3;
#endif/*DEBUG*/
				goto _fail;
			}
		}
		/*## Enter Stop Mode #######################################################*/
		HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
		stop_time-=65535;
	}


_fail:
	PRINTF("Enter stop mode (fail code = %d)\n", fail);
}
/*---------------------------------------------------------------------------*/
/**
* @brief  Exit from STOP mode for MCU and Standby mode for the radio.
* @param  wakeup_radio boolean to decide if to wake up the radio when exiting the stop mode.
* @retval None.
*/
void LP_exit_stop_mode(int wakeup_radio)
{
	/* Configures system clock after wake-up from STOP: enable HSI, PLL and select
	  PLL as system clock source (HSI and PLL are disabled in STOP mode) */
	_lp_sysclk_exit_stop();

	/* Disable Wake-up timer */
	if(HAL_RTCEx_DeactivateWakeUpTimer(&RtcHandle) != HAL_OK){
		/* Initialization Error */
		Error_Handler();
	}

	HAL_ResumeTick();
	EXTI->IMR = exti_imr_backup;
	_lp_peripheral_disable();

	if (wakeup_radio){
		subGHz_radio_driver.on();
	}
	HAL_PWREx_DisableUltraLowPower();

	__HAL_FLASH_PREFETCH_BUFFER_ENABLE();
	/* Enable 64-bit Access */
	__HAL_FLASH_ACC64_ENABLE();
	/* Clear Wake Up Flag */
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	/* Enable PVD */
	HAL_PWR_EnablePVD();

	PRINTF("Exit stop mode\n");
}
/*---------------------------------------------------------------------------*/
/**
* @brief  Let the system go in SLEEP mode for MCU, with reduced clock frequency.
* @param  None.
* @retval None.
*/
void LP_enter_sleep_mode(void)
{
#if LP_PERIPHERAL_IN_SLEEP
	_lp_peripheral_enable();
#endif /*LP_PERIPHERAL_IN_SLEEP*/
	HAL_SuspendTick();

    HAL_NVIC_DisableIRQ((IRQn_Type)EXTI0_IRQn);
	HAL_NVIC_DisableIRQ((IRQn_Type)TIM2_IRQn);
	HAL_NVIC_DisableIRQ((IRQn_Type)SysTick_IRQn);
	HAL_PWREx_EnableFastWakeUp();
	__HAL_FLASH_PREFETCH_BUFFER_DISABLE();
	__HAL_FLASH_ACC64_DISABLE();
	__HAL_FLASH_SLEEP_POWERDOWN_ENABLE();
	HAL_RCC_DeInit();

	/*Reducing the Clock Frequency for max power saving */
	LP_sysclk_config(SLEEP_CLOCK_FREQUENCY);
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
}
/*---------------------------------------------------------------------------*/
/**
* @brief  Exit from SLEEP mode for MCU and restore USER_CLOCK_FREQUENCY.
* @param  None.
* @retval None.
*/
void LP_exit_sleep_mode(void)
{
	/* This function must be called from the main idle loop, after LP_enter_sleep_mode(),
	 * and from the HAL_GPIO_EXTI_Callback(), in order to set system clock frequency back to
	 * USER_CLOCK_FREQUENCY before calling the radio irq callback.
	 * */
	if (LP_interrupt_from_radio==0 && HAL_RCC_GetSysClockFreq()==SLEEP_CLOCK_FREQUENCY_HZ){
		LP_sysclk_config(USER_CLOCK_FREQUENCY);
#if LP_PERIPHERAL_IN_SLEEP
		_lp_peripheral_disable();
#endif /*LP_PERIPHERAL_IN_SLEEP*/
		HAL_ResumeTick();
		/*We need to compensate clock exiting from sleep, how many tickes depends on
		 * USER_CLOCK_FREQUENCY and on the used toolchain (__WFI() implementations differ) */
		_lp_clock_adjust(COMPENSATE_TICKS);

		HAL_PWREx_DisableUltraLowPower();
		__HAL_FLASH_PREFETCH_BUFFER_ENABLE();
		__HAL_FLASH_ACC64_ENABLE();
		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
		HAL_PWR_EnablePVD();
		HAL_NVIC_EnableIRQ((IRQn_Type)EXTI0_IRQn);
		HAL_NVIC_EnableIRQ((IRQn_Type)TIM2_IRQn);
		HAL_NVIC_EnableIRQ((IRQn_Type)SysTick_IRQn);
	}
}
/*---------------------------------------------------------------------------*/
/**
  * @brief  Low Power System Clock Configuration.
  * @param  clk_freq  the desired clock frequency in KHz.
  * @retval None.
  */
void LP_sysclk_config (unsigned int clk_freq)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  VALIDATE_CLK_F(clk_freq);

  if (clk_freq >= 1048)
  {
	 F_CPU = clk_freq * 1000;
  }
  else
  {
	  	switch(clk_freq)
	  	{
	  		case 65:
	  			F_CPU =65536;
	  			break;
	  		case 131:
	  			F_CPU =131072;
	  			break;
	  		case 262:
	  			F_CPU =262144;
	  			break;
	  		case 524:
	  			F_CPU =524288;
	  			break;
	  	}
  }

  if (clk_freq >= 12000)
  {
  	// HSI Configurations
  	//32MHz PLLMUL = 6 and PLLDIV = 3
  	//24MHz PLLMUL = 6 and PLLDIV = 4
  	//16MHz PLLMUL = 3 and PLLDIV = 3
  	//12MHz PLLMUL = 3 and PLLDIV = 4

  	/* Set Voltage scale */
  	// Range1 (V_DD range limited to 1.71V - 3.6V), with the CPU running at up to 32MHz
  	// Range2 (full V_DD range), with a maximum CPU frequency of 16MHz
  	// Range3 (full V_DD range), with a maximum frequency limited to 4MHz
  	// generated only with the multispeed internal RTC oscillator clock source

  	/* Enable HSE Oscillator and Activate PLL with HSE as source */
  	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  	RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
  	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;

  	switch(clk_freq)
  	{
  		case 32000:
  			RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  			RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  			if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  			{
  				Error_Handler();
  			}
  			__PWR_CLK_ENABLE();
  			__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  			break;
  	  	case 24000:
  	  		RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  	  		RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV4;
  	  		if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  	  		{
  	  			Error_Handler();
  	  		}
  	  		__PWR_CLK_ENABLE();
  	  		__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  	  		break;
  	  	case 16000:
  	  		RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
  	  		RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  	  		if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  	  		{
  	  			Error_Handler();
  	  		}
  	  		__PWR_CLK_ENABLE();
  	  		__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  	  		break;
  	  	case 12000:
  	  		RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
  	  		RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV4;
  	  		if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  	  		{
  	  			Error_Handler();
  	  		}
  	  		__PWR_CLK_ENABLE();
      	    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
      	    break;
  	}

  	/* Poll VOSF bit of in PWR_CSR. Wait until it is reset to 0 */
  	while (__HAL_PWR_GET_FLAG(PWR_FLAG_VOS) != RESET) {};

  	/* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
		clocks dividers */
  	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);

  	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;

  	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  	{
  		Error_Handler();
  	}
  }
  else if (clk_freq <= 4194)
  {
  	// MSIConfigurations
  	//RCC_ICSCR_MSIRANGE_0 /*!< MSI = 65.536 KHz
  	//RCC_ICSCR_MSIRANGE_1 /*!< MSI = 131.072 KHz
  	//RCC_ICSCR_MSIRANGE_2 /*!< MSI = 262.144 KH
  	//RCC_ICSCR_MSIRANGE_3 /*!< MSI = 524.288 KHz
  	//RCC_ICSCR_MSIRANGE_4 /*!< MSI = 1.048 MHz
  	//RCC_ICSCR_MSIRANGE_5 /*!< MSI = 2.097 MHz
  	// RCC_ICSCR_MSIRANGE_6 /*!< MSI = 4.194 MHz the system works well, under
  	//this frequency it doesn't work

  	// to use MSI, you have to you have to set NUCLEO_I2C_EXPBD_SPEED = 100000
  	// in Drivers/BSP/X-Nucleo_IKS01Ax/x_nucleo_iks01ax.h

  	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  	RCC_OscInitStruct.MSICalibrationValue = 0;

  	switch(clk_freq){
  		case 4194:
  			RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  			break;
  		case 2097:
  			RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  			break;
  		case 1048:
  			RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_4;
  			break;
  		case 524:
  			RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_3;
  			break;
  		case 262:
  			RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_2;
  			break;
  		case 131:
  			RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_2;
  			break;
  		case 65:
  			RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_2;
  			break;
  	}

  	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;

  	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  	{
  		Error_Handler();
  	}

  	/* Set Voltage scale */
  	// Range1 (V_DD range limited to 1.71V - 3.6V), with the CPU running at up to 32MHz
  	// Range2 (full V_DD range), with a maximum CPU frequency of 16MHz
  	// Range3 (full V_DD range), with a maximum frequency limited to 4MHz
  	// generated only with the multispeed internal RTC oscillator clock source
  	__PWR_CLK_ENABLE();
  	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;

  	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  	{
  		Error_Handler();
  	}
  	if(clk_freq==65)
  	{
  		/* Set MSI range to 0 */
  		__HAL_RCC_MSI_RANGE_CONFIG(RCC_MSIRANGE_0);
  	}
  	if(clk_freq==131)
	{
  		/* Set MSI range to 1 */
  		__HAL_RCC_MSI_RANGE_CONFIG(RCC_MSIRANGE_1);
  	}
  }
}
/*---------------------------------------------------------------------------*/
/* Static Local Functions                                                    */
/*---------------------------------------------------------------------------*/
/**
  * @brief  Reset the system clock back to normal configuration after STOP mode.
  * @param  None.
  * @retval None.
  */
static void _lp_sysclk_exit_stop(void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;
	uint32_t pFLatency = 0;

	/* Get the Oscillators configuration according to the internal RCC registers */
	HAL_RCC_GetOscConfig(&RCC_OscInitStruct);

	/* After wake-up from STOP reconfigure the system clock: Enable HSI and PLL */
	if (USER_CLOCK_FREQUENCY > 4194)
	{
		RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
		RCC_OscInitStruct.HSIState = RCC_HSI_ON;
		RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
		RCC_OscInitStruct.HSICalibrationValue = 0x10;
	}
	else
	{
	  	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
	  	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	  	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	  	RCC_OscInitStruct.MSICalibrationValue = 0;

	}
	if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/* Get the Clocks configuration according to the internal RCC registers */
	HAL_RCC_GetClockConfig(&RCC_ClkInitStruct, &pFLatency);

	/* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
	if (USER_CLOCK_FREQUENCY > 4194)
	{
		RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	}
	else
	{
		RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
	}

	if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, pFLatency) != HAL_OK)
	{
		Error_Handler();
	}
}
/*---------------------------------------------------------------------------*/
/**
  * @brief  Configure peripheral GPIO to enter STOP mode.
  * @param  None.
  * @retval None.
  */
static void _lp_peripheral_enable(void)
{
	/*	Gpio configuration:
	*
	* 	X-Nucleo-S2868A1 expansion board uses the following pin connected to the STM32L152RE:
	* 	SDN: 		PA8
	* 	CSN:		PA1
	* 	MISO:		PA6
	* 	MOSI:		PA7
	* 	SCLK		PB3
	*
	* 	They are configured by the radio driver but during low power mode (sleep/stop) they need to be configured in the correct state
	* 	in order to be sure that the MCU goes in sleep state, the radio is turned off (standby) and the EEPROM keeps a stable state:
	* 	PA8(SDN):		output + nopull + storing a '0'
	* 	PA1(CSN):		output + nopull + storing a '1' (this is the default value)
	* 	PA6(MISO):		analog + nopull
	* 	PA7(MOSI):		output + nopull + storing a '0'
	* 	PB3(CLK):		output + nopull + storing a '0'
	*
	* 	SPIGPIO3 and LED are set to ANALOG NOPULL since they are not critical
	*
	*	Other GPIO used are:
	*	PA5:		Nucleo Green Led
	*	PA0:		TIM2 (used by contiki timer)
	*	PC13:		Nucleo Push Button
	*	PA2:		TX for USART2
	*	PA3:		RX for USART2
	*
	* */

	GPIO_InitTypeDef GPIO_InitStructure= {0};

	/*Save the previous configuration*/
	gpioa_moder = GPIOA->MODER;
	gpiob_moder = GPIOB->MODER;
	gpioc_moder = GPIOC->MODER;
	gpioa_otyper = GPIOA->OTYPER;
	gpiob_otyper = GPIOB->OTYPER;
	gpioc_otyper = GPIOC->OTYPER;
	gpioa_ospeedr = GPIOA->OSPEEDR;
	gpiob_ospeedr = GPIOB->OSPEEDR;
	gpioc_ospeedr = GPIOC->OSPEEDR;
	gpioa_pupdr = GPIOA->PUPDR;
	gpiob_pupdr = GPIOB->PUPDR;
	gpioc_pupdr = GPIOC->PUPDR;

	/*GPIOC configuration*/
	__HAL_RCC_GPIOC_CLK_ENABLE();
	GPIO_InitStructure.Mode=GPIO_MODE_ANALOG;
	GPIO_InitStructure.Pull=GPIO_NOPULL;
	GPIO_InitStructure.Pin=GPIO_PIN_All;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
	__HAL_RCC_GPIOC_CLK_DISABLE();

	/*GPIOA conf */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitStructure.Mode=GPIO_MODE_ANALOG;
	GPIO_InitStructure.Pull=GPIO_NOPULL;
	GPIO_InitStructure.Speed=GPIO_SPEED_HIGH;
	GPIO_InitStructure.Pin=GPIO_PIN_All & ~GPIO_PIN_7 & ~GPIO_PIN_8 & ~GPIO_PIN_1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*Critical pin: PA8 PA7 PA1*/
	GPIO_InitStructure.Mode=GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull=GPIO_NOPULL;
	GPIO_InitStructure.Speed=GPIO_SPEED_HIGH;
	GPIO_InitStructure.Pin=GPIO_PIN_8 | GPIO_PIN_7 | GPIO_PIN_1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
	__HAL_RCC_GPIOA_CLK_DISABLE();

	/*GPIOB conf */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitStructure.Mode=GPIO_MODE_ANALOG;
	GPIO_InitStructure.Pull=GPIO_NOPULL;
	GPIO_InitStructure.Speed=GPIO_SPEED_HIGH;
	GPIO_InitStructure.Alternate=0x00;
	GPIO_InitStructure.Pin=GPIO_PIN_All & ~GPIO_PIN_3;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

	/*Critical pin: PB3 */
	GPIO_InitStructure.Mode=GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull=GPIO_NOPULL;
	GPIO_InitStructure.Speed=GPIO_SPEED_HIGH;
	GPIO_InitStructure.Pin=GPIO_PIN_3;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
	__HAL_RCC_GPIOB_CLK_DISABLE();
}
/*---------------------------------------------------------------------------*/
/**
  * @brief  Configure peripheral GPIO to exit STOP mode.
  * @param  None.
  * @retval None.
  */
static void _lp_peripheral_disable(void)
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	GPIOA->MODER = gpioa_moder;
	GPIOB->MODER = gpiob_moder;
	GPIOC->MODER = gpioc_moder;
	GPIOA->OTYPER = gpioa_otyper;
	GPIOB->OTYPER = gpiob_otyper;
	GPIOC->OTYPER = gpioc_otyper;
	GPIOA->OSPEEDR = gpioa_ospeedr;
	GPIOB->OSPEEDR = gpiob_ospeedr;
	GPIOC->OSPEEDR = gpioc_ospeedr;
	GPIOA->PUPDR = gpioa_pupdr;
	GPIOB->PUPDR = gpiob_pupdr;
	GPIOC->PUPDR = gpioc_pupdr;
}

/*---------------------------------------------------------------------------*/
/**
  * @brief  Adjust the system clock (after a reduced frequency sleep) by a given amount of ticks.
  * @param  howmany the number of ticks the clock has to be incremented.
  * @retval None.
  */
static void _lp_clock_adjust(clock_time_t howmany)
{
	/*See low-power.h for an explanation on the ticks value to pass to this function.*/
	/*Add seconds*/
	seconds += howmany/CLOCK_SECOND;
	/* Handle tick overflow */
	if(((ticks % CLOCK_SECOND) + (howmany % CLOCK_SECOND)) >= CLOCK_SECOND){
		seconds++;
	}
	/*Add ticks*/
	ticks+=howmany;
}
/*---------------------------------------------------------------------------*/
#endif /*MCU_LOW_POWER*/
/**
* @}
*/

/**
* @}
*/
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
