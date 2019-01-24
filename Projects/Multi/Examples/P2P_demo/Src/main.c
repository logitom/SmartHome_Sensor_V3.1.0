/**
******************************************************************************
* @file    main.c 
* @author  CLAB
* @version V1.0.0
@date    10-Sep-2016
* @brief   Main program body
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
#include "radio_shield_config.h"
#include "radio_appli.h"
#include "p2p_lib.h"

/** @addtogroup USER
* @{
*/

/* Private typedef -----------------------------------------------------------*/ 
/* Private define ------------------------------------------------------------*/
#define TX_BUFFER_SIZE   20
#define RX_BUFFER_SIZE   96

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t TxLength = TX_BUFFER_SIZE;
uint8_t RxLength = 0;
uint8_t aTransmitBuffer[TX_BUFFER_SIZE] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,\
  16,17,18,19,20};
uint8_t aReceiveBuffer[RX_BUFFER_SIZE] = {0x00};

/* Private function prototypes -----------------------------------------------*/

#if defined(USE_LOW_POWER_MODE)
static void SystemPower_Config(void);
#endif
int main(void);

void Clock_Enable(void);

/* Private functions ---------------------------------------------------------*/
/**
* @brief  Main program.
* @param  None
* @retval None
*/
int main(void)
{
  /* MCU Configuration----------------------------------------------------------*/
  
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  
  Clock_Enable();
  
  /* Configure the system clock */
 #if defined(X_NUCLEO_IDS01A4) || defined(X_NUCLEO_IDS01A5)
 SystemClock_Config();
#endif
  
#if defined(MCU_SLEEP_MODE) 
  {
    HAL_EnableDBGSleepMode();
  }
#endif 
#if defined(MCU_STOP_MODE) 
  {
    HAL_EnableDBGStopMode();
  }
#endif 
  
  /* Initialize LEDs*/
#if defined(X_NUCLEO_IDS01A4) || defined(X_NUCLEO_IDS01A5)
  RadioShieldLedInit(RADIO_SHIELD_LED);
#endif
  
  BSP_LED_Init(LED2);
  HAL_Radio_Init();
  
  /* Initialize Buttons*/
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);
  
  P2P_Init();
  
  while (1)
  { 
    /* Data communication start */   
    P2P_Process(aTransmitBuffer, TxLength, aReceiveBuffer, RxLength);
     
  } 
}


void Clock_Enable(void)
{
  __GPIOA_CLK_ENABLE();  
  __GPIOB_CLK_ENABLE();  
  __GPIOC_CLK_ENABLE();  
  __GPIOD_CLK_ENABLE();  
}



#if defined(USE_LOW_POWER_MODE)
/**
* @brief  System Power Configuration
*         The system Power is configured as follow : 
*            + Regulator in LP mode
*            + VREFINT OFF, with fast wakeup enabled
*            + HSI as SysClk after Wake Up
*            + No IWDG
*            + Wakeup using EXTI Line (Key Button PC.13)
* @param  None
* @retval None
*/
static void SystemPower_Config(void)
{
  SystemPower_Configuration();
}
#endif

#ifdef USE_FULL_ASSERT

/**
* @brief Reports the name of the source file and the source line number
* where the assert_param error has occurred.
* @param file: pointer to the source file name
* @param line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
}

#endif

/**
* @}
*/ 

/**
* @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
