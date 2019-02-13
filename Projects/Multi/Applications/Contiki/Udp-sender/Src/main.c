/**
******************************************************************************
* @file    main.c
* @author  System LAB
* @version V1.0.0
* @date    17-June-2015
* @brief   STM32 Cube HAL Init Source file (formerly main.c as per STM32Cube)
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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "main.h"
#include "cube_hal.h"
#include "radio_shield_config.h"
#include "radio-driver.h"
#include "process.h"
#include "contiki-st-radio-main.h"
#include "low-power.h"

/** @defgroup Udp_sender
* @{
*/

/** @addtogroup Udp_sender
* @{
*/


/* Private function prototypes -----------------------------------------------*/
static void _print_fw_info(void);
#if MCU_LOW_POWER && RADIO_USES_CONTIKIMAC
int sender_check_connection(); //implemented in unicast-sender.c
#endif /*MCU_LOW_POWER && RADIO_USES_CONTIKIMAC*/
/*----------------------------------------------------------------------------*/
extern UART_HandleTypeDef UartHandle;
/*----------------------------------------------------------------------------*/
#if LP_PERIPHERAL_IN_SLEEP
int from_sleep = 0;
#endif /*LP_PERIPHERAL_IN_SLEEP*/
/*----------------------------------------------------------------------------*/

/**
* @brief  main()
* @param  None
* @retval None
*/
int main()
{
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
  
  HAL_EnableDBGStopMode();
  
  MX_GPIO_Init();
  USARTConfig();
  
  BUZZERConfig();
  
  
#if MCU_LOW_POWER
  if (USER_CLOCK_FREQUENCY != DEFAULT_CLOCK_FREQUENCY) {
	  HAL_RCC_DeInit();
	  LP_sysclk_config(USER_CLOCK_FREQUENCY);
	  MX_GPIO_Init();
	  HAL_UART_DeInit(&UartHandle);
	  USARTConfig();
  }
#endif /*MCU_LOW_POWER*/
  
  /* Initialize LEDs */
  BSP_LED_Init(LED2);
  BSP_LED_Init(LED_ALARM);
 
  RadioShieldLedInit(RADIO_SHIELD_LED);
  
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
  
  /* Initialize RTC */
  RTC_Config();
  RTC_TimeStampConfig();
  _print_fw_info();
  
  Stack_6LoWPAN_Init();

  while(1) {
    int r = 0;
#if MCU_LOW_POWER
    LP_interrupt_from_radio=0;
#endif /*MCU_LOW_POWER*/
    do {
      r = process_run();
      // add watch dog timer here
      
    } while(r > 0);
#if MCU_LOW_POWER
#  if RADIO_USES_CONTIKIMAC
    /*This setting is still experimental. For the sender we
     * check for servreghack service to speed up the connection*/
    if (sender_check_connection() && !contikimac_CCA_in_progrss){
    	for (int k=0;k<CONSECUTIVE_SLEEPS; k++) {
#    if LP_PERIPHERAL_IN_SLEEP
    		from_sleep = 1;
#    endif /*LP_PERIPHERAL_IN_SLEEP*/
    	    LP_enter_sleep_mode();
    	    LP_exit_sleep_mode();
    	}
    	contikimac_CCA_in_progrss = 1;
    }
#  else /*!RADIO_USES_CONTIKIMAC*/
#    if LP_PERIPHERAL_IN_SLEEP
    from_sleep = 1;
#    endif /*LP_PERIPHERAL_IN_SLEEP*/
    LP_enter_sleep_mode();
    LP_exit_sleep_mode();
#  endif /*RADIO_USES_CONTIKIMAC (and MCU_LOW_POWER) */
#endif /*MCU_LOW_POWER*/
  }
  
}
/*----------------------------------------------------------------------------*/
static void _print_fw_info(void)
{
  /*Compiler, HAL and firmware info:*/
  printf("\t(HAL %ld.%ld.%ld_%ld)\r\n"
	       "\tCompiled %s %s"
#if defined (__IAR_SYSTEMS_ICC__)
	           " (IAR)\r\n\n",
#elif defined (__CC_ARM)
	           " (KEIL)\r\n\n",
#elif defined (__GNUC__)
	           " (openstm32)\r\n\n",
#endif
   HAL_GetHalVersion() >>24,
   (HAL_GetHalVersion() >>16)&0xFF,
   (HAL_GetHalVersion() >> 8)&0xFF,
   HAL_GetHalVersion()      &0xFF,
   __DATE__,__TIME__);
}
/*----------------------------------------------------------------------------*/
/**
* @}
*/

/**
* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
