/**
* @file    wmbus_tim.c
* @author  VMA division - AMS/System Lab - NOIDA
* @version V2.4.3
* @date    14-Feb-2018
* @brief   This file contains routine for Timer. It is an implementation of
*          the timer interface specified by the header file wmbus_tim.h
*          of wM-BUS_Library_2013.
* @details
*
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
* TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
* DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
* FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* THIS SOURCE CODE IS PROTECTED BY A LICENSE.
* FOR MORE INFORMATION PLEASE CAREFULLY READ THE LICENSE AGREEMENT FILE LOCATED
* IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
*
* <h2><center>&copy; COPYRIGHT 2018 STMicroelectronics</center></h2>
*/


#include "wmbus_tim.h"


#ifdef USE_STM32F4XX_NUCLEO
#include "stm32f4xx_hal_tim.h"
#endif

#ifdef USE_STM32L1XX_NUCLEO
#include "stm32l1xx_hal_tim.h"
#endif

#ifdef USE_STM32L0XX_NUCLEO
#include "stm32l0xx_hal_tim.h"
#endif

/**
* @addtogroup Application Application
* @{
*/

/**
* @addtogroup wmbus_user_application    WMBus User Application
* @{
*/

/**
* @addtogroup wmbus_tim    WMBus Timer Implementation
* @brief An implementation of the timer interface.
* @details See the file <i>WMBUS_Library_2013/wmbus_tim.h</i> for more details.
* @{
*/


/**
* @defgroup wmbus_tim_Private_Variables      WMBus Timer Private Variables
* @{
*/

static volatile uint32_t NOverflows=0;

/**
*@}
*/


/**
* @defgroup wmbus_tim_Private_Defines        WMBus Timer Private Defines
* @{
*/

#define PROTOCOL_TIMER                          TIM2
#define PROTOCOL_TIMER_ISR                      TIM2_IRQHandler
#define PROTOCOL_TIMER_IRQN                     TIM2_IRQn
#define PROTOCOL_TIMER_CLOCK_ENABLE()           __HAL_RCC_TIM2_CLK_ENABLE()

/* TIM handle declaration */
TIM_HandleTypeDef    TimHandle;

/**
*@}
*/


/**
* @defgroup wmbus_tim_Exported_Functions         WMBus Timer Exported Functions
* @{
*/

/**
* @brief  Timer Interrupt Handler.
* @param  None
* @retval None
*/
void PROTOCOL_TIMER_ISR(void)   
{

  HAL_TIM_IRQHandler(&TimHandle);
}

/**
* @brief  Counts the Number of overflow.
* @param  None
* @retval None
*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == PROTOCOL_TIMER)
  {
      NOverflows++;
  }
}

/**
* @brief  Initialize Timer.
* @param  None
* @retval None
*/
void WMBus_TimInit(void)  
{

  /* Set TIMx instance */
  TimHandle.Instance = PROTOCOL_TIMER;

  /* Initialize TIMx peripheral as follows:
       + Period = 0xFFFF - 1
       + Prescaler = (SystemCoreClock/3200) - 1
       + ClockDivision = 0
       + Counter direction = Up
  */
  TimHandle.Init.Period            = 0xFFFF;
  TimHandle.Init.Prescaler         = 3200-1;
  TimHandle.Init.ClockDivision     = 0;
  TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;

  HAL_TIM_Base_DeInit(&TimHandle);
  
  if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
  {
    /* Initialization Error */
    /*Error_Handler();*/
  }
  
  __HAL_TIM_CLEAR_FLAG(&TimHandle, TIM_FLAG_UPDATE);
  
  /*##-2- Start the TIM Base generation in interrupt mode ####################*/
  /* Start Channel1 */
  if (HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK)
  {
    /* Starting Error */
    /*Error_Handler();*/
  }

}

/**
* @brief  Get Time Stamp.
* @param  None
* @retval Returns the current value of the Timer-Counter Register
*/
uint32_t WMBus_TimGetTimestamp(void)
{
  uint32_t tStamp,tCounter,tNow;
  tStamp = NOverflows*0x10000;
  tCounter = __HAL_TIM_GET_COUNTER(&TimHandle);
  tNow = tStamp + tCounter;
  return (tNow);
}


/**
* @brief  Calculates the Time difference from a given time Stamp.
* @param  t0: Initial Time Stamp
* @retval Returns the Time Difference.
*/
uint32_t WMBus_TimDiffFromNow(uint32_t t0)
{
  volatile uint32_t now=WMBus_TimGetTimestamp();
  
  if(now<t0)
  {
    /*
    case in which the current now is after the 32 bits max
             t0       2^32|0        now 
    ---------|------------|----------|---- 
    */
    return(0xffffffff-t0+now);
  }
  
  /* else return the simple difference */
  return(now-t0);
}


/**
* @brief  Inserts a time delay from a Initial Time Stamp
* @param  InitialTimeStamp: Initial Time Stamp 
* @param  ReqTimeDelay: The required Time Delay.
* @retval 0/1: Returns 1 When a delay is given else returns 0.
*/
uint8_t TimeDelay(uint32_t InitialTimeStamp,uint32_t ReqTimeDelay)
{
  volatile uint32_t timeDiff;
  int32_t tmp;
  
  timeDiff = WMBus_TimDiffFromNow(InitialTimeStamp);
  tmp = ReqTimeDelay - timeDiff;                                                 
  
  if(tmp>0)                                                                      
  {
    /*Return FALSE*/
    return 0;
  }
  else
  {
    /* Return TRUE*/
    return 1;
  }
}


/**
  * @brief TIM MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  * @param htim: TIM handle pointer
  * @retval None
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
  /*Enable peripherals and GPIO Clocks */
  
  /* TIMx Peripheral clock enable */
  PROTOCOL_TIMER_CLOCK_ENABLE();
  
  /*Configure the NVIC for TIMx */
  /* Set the TIMx priority */
  HAL_NVIC_SetPriority(PROTOCOL_TIMER_IRQN, 2, 0);

  /* Enable the TIMx global Interrupt */
  HAL_NVIC_EnableIRQ(PROTOCOL_TIMER_IRQN);
}


/**
  * @brief Resets the timer
  *        This function RESET the counter of TIMER to ZERO
  * @param None
  * @retval None
  */
void WMBus_TimReset(void)
{
  
  NOverflows = 0;
__HAL_TIM_SET_COUNTER(&TimHandle, 0);
  
}

/**
  * @brief Resets the timer if n Overflow has occured
  *        This function RESET the counter after n Overflow
  * @param None
  * @retval None
  */
void WMBus_TimResetNOverflow(uint8_t numOverflow)
{
  if(numOverflow >= NOverflows)
  {
    NOverflows = 0;
    __HAL_TIM_SET_COUNTER(&TimHandle, 0);
  }
}

/**
*@}
*/

/**
*@}
*/

/**
*@}
*/

/**
*@}
*/


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
