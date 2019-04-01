/**
******************************************************************************
* @file    cube_hal_l1.c
* @author  CL
* @version V1.0.0
* @date    03-November-2014
* @brief   
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
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

/* Includes ------------------------------------------------------------------*/
#include "cube_hal.h"
#include "main.h"
//#include "hw-config.h"
/** @defgroup cube_hal_l1
  * @ingroup Udp_sender
  * @{
  */

/** @addtogroup cube_hal_l1
  * @ingroup Udp_sender
  * @{
  */

#define I2C_SPEEDCLOCK   100000
#define I2C_DUTYCYCLE    I2C_DUTYCYCLE_2

/* I2C handler declaration */
I2C_HandleTypeDef I2cHandle;

RTC_HandleTypeDef RtcHandle;

extern uint8_t RxCounter;
extern uint8_t aRxBuffer[6];
/**
  * @brief  Configure the RTC peripheral by selecting the clock source.
  * @param  None
  * @retval None
  */
void RTC_Config(void)
{
  /*##-1- Configure the RTC peripheral #######################################*/
  RtcHandle.Instance = RTC;

  /* Configure RTC prescaler and RTC data registers */
  /* RTC configured as follow:
  - Hour Format    = Format 12
  - Asynch Prediv  = Value according to source clock
  - Synch Prediv   = Value according to source clock
  - OutPut         = Output Disable
  - OutPutPolarity = High Polarity
  - OutPutType     = Open Drain */
  RtcHandle.Init.HourFormat = RTC_HOURFORMAT_12;
  RtcHandle.Init.AsynchPrediv = RTC_ASYNCH_PREDIV;
  RtcHandle.Init.SynchPrediv = RTC_SYNCH_PREDIV;
  RtcHandle.Init.OutPut = RTC_OUTPUT_DISABLE;
  RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  RtcHandle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;

  if(HAL_RTC_Init(&RtcHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}


/**
 * @brief  Configures the current time and date.
 * @param  None
 * @retval None
 */
void RTC_TimeStampConfig(void)
{
    RTC_DateTypeDef sdatestructure;
    RTC_TimeTypeDef stimestructure;

    /*##-3- Configure the Date #################################################*/
    /* Set Date: Tuesday February 18th 2014 */
    sdatestructure.Year = 0x14;
    sdatestructure.Month = RTC_MONTH_FEBRUARY;
    sdatestructure.Date = 0x18;
    sdatestructure.WeekDay = RTC_WEEKDAY_TUESDAY;

    if(HAL_RTC_SetDate(&RtcHandle,&sdatestructure,FORMAT_BCD) != HAL_OK)
    {
        /* Initialization Error */
        Error_Handler();
    }

    /*##-4- Configure the Time #################################################*/
    /* Set Time: 08:10:00 */
    stimestructure.Hours = 0x08;
    stimestructure.Minutes = 0x10;
    stimestructure.Seconds = 0x00;
    stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
    stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
    stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

    if(HAL_RTC_SetTime(&RtcHandle,&stimestructure,FORMAT_BCD) != HAL_OK)
    {
        /* Initialization Error */
        Error_Handler();
    }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 32000000
  *            HCLK(Hz)                       = 32000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            HSI Frequency(Hz)              = 16000000
  *            PLL_MUL                        = 4
  *            PLL_DIV                        = 2
  *            Flash Latency(WS)              = 1
  *            Main regulator output voltage  = Scale1 mode
  * @param  None
  * @retval None
  */

void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  /* Enable HSE Oscillator and Activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Set Voltage scale1 as MCU will run at 32MHz */
  __PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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


/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{
	/*	Deactivate unused GPIO pin */
	GPIO_InitTypeDef GPIO_InitStructure= {0};
	/* GPIO Ports Clock Enable */
	__GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
	__GPIOC_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();
	/* W.B.C: D, E, F, G and H GPIOs are set as ANALOG in NOPULL with clock disabled since
	   they are not used by the application. (NUCLEO + S2LP)*/
	//MGR: check if it is ok for SPIRIT1, also check that this code is modified by us...

	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();

#if 1
	GPIO_InitStructure.Mode=GPIO_MODE_ANALOG;
	GPIO_InitStructure.Pull=GPIO_NOPULL;
	GPIO_InitStructure.Pin=GPIO_PIN_All;

	HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
	HAL_GPIO_Init(GPIOE, &GPIO_InitStructure);
	HAL_GPIO_Init(GPIOF, &GPIO_InitStructure);
	HAL_GPIO_Init(GPIOG, &GPIO_InitStructure);
	HAL_GPIO_Init(GPIOH, &GPIO_InitStructure);

	__HAL_RCC_GPIOD_CLK_DISABLE();
	__HAL_RCC_GPIOE_CLK_DISABLE();
	__HAL_RCC_GPIOF_CLK_DISABLE();
	__HAL_RCC_GPIOG_CLK_DISABLE();
	__HAL_RCC_GPIOH_CLK_DISABLE();


	/* Disable unused peripherals */
	__HAL_RCC_CRC_CLK_DISABLE();

	__HAL_RCC_DMA2_CLK_DISABLE();

	__HAL_RCC_TIM3_CLK_DISABLE();
	__HAL_RCC_TIM4_CLK_DISABLE();
	__HAL_RCC_TIM5_CLK_DISABLE();
	__HAL_RCC_TIM6_CLK_DISABLE();
	__HAL_RCC_TIM7_CLK_DISABLE();
	__HAL_RCC_TIM9_CLK_DISABLE();
	__HAL_RCC_TIM10_CLK_DISABLE();
	__HAL_RCC_TIM11_CLK_DISABLE();
	__HAL_RCC_LCD_CLK_DISABLE();
	__HAL_RCC_WWDG_CLK_DISABLE();

	__HAL_RCC_SPI2_CLK_DISABLE();
	__HAL_RCC_SPI3_CLK_DISABLE();
	__HAL_RCC_I2C1_CLK_DISABLE();
	__HAL_RCC_I2C2_CLK_DISABLE();

	__HAL_RCC_COMP_CLK_DISABLE();
#endif  
  
  GPIO_InitTypeDef GPIO_InitStruct;
     /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();
  __HAL_RCC_TIM5_CLK_ENABLE();
  
 // __HAL_RCC_GPIOB_CLK_ENABLE();
 // __HAL_RCC_I2C1_CLK_ENABLE();
  __TIM2_CLK_ENABLE();
  __TIM3_CLK_ENABLE();
  __HAL_RCC_WWDG_CLK_ENABLE();
 
    /*Configure GPIO pin : PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode =GPIO_MODE_IT_FALLING;;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    
   /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn); 
  
  
  /* 8-bit mcu wake up pin */
  

}
void MX_MCU_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct= {0}; 
  
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  
  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  //GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Pull = GPIO_PULLUP;                       //jas modify
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  /*Configure GPIO pin Output Level */
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);   //jas mark
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);       //jas add   
  
  #if 0
   /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  //GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Pull = GPIO_NOPULL;                      
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);      
  #endif
  
  
  
}


/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1)
  {
	  BSP_LED_Toggle(LED2);
	  HAL_Delay(250);
  }
}

void MX_I2C_Init(void)
{
    /*##-1- Configure the I2C peripheral ######################################*/
  I2cHandle.Instance              = I2Cx;
  I2cHandle.Init.ClockSpeed       = I2C_SPEEDCLOCK;
  I2cHandle.Init.DutyCycle        = I2C_DUTYCYCLE;
  I2cHandle.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
  I2cHandle.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
  I2cHandle.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
  I2cHandle.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;  
  I2cHandle.Init.OwnAddress1      = I2C_ADDRESS;
  I2cHandle.Init.OwnAddress2      = 0xFF;
  
  if(HAL_I2C_Init(&I2cHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
#if 0
  RxCounter++;
  /*##-2- Put I2C peripheral in reception process ###########################*/  
  if(HAL_I2C_Slave_Receive_DMA(&I2cHandle, (uint8_t *)aRxBuffer, 6) != HAL_OK)
  {
    /* Transfer error in reception process */
    Error_Handler();
  }
#endif  
  /*##-3- Wait for the end of the transfer ###################################*/  
  /*  Before starting a new communication transfer, you need to check the current   
      state of the peripheral; if it? busy you need to wait for the end of current
      transfer before starting a new one.
      For simplicity reasons, this example is just waiting till the end of the
      transfer, but application may perform other tasks while transfer operation
      is ongoing. */
#if 0
  while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY)
  {
  }
#endif  
  
  
}  


/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
