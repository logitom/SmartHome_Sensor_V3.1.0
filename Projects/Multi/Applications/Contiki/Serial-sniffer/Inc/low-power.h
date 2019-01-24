/**
******************************************************************************
* @file    low-power.h
* @author  ST
* @version V1.0.0
* @date    05-December-2018
* @brief   MCU Low Power API include file
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

/** @addtogroup Low_power
* @{
*/
#ifndef _LOW_POWER_H_
#define _LOW_POWER_H_
#include "platform-conf.h" /*For DEFAULT_CLOCK_FREQUENCY*/

#if defined(X_NUCLEO_S2868A1) || defined(X_NUCLEO_S2915A1)
#define S2LP_RADIO
#elif defined(X_NUCLEO_IDS01A4) || defined(X_NUCLEO_IDS01A5)
#define SPIRIT1_RADIO
#define RADIO_USES_LONG_PREAMBLE 0
#endif /*RADIO TYPE*/

#ifndef MCU_LOW_POWER
#define MCU_LOW_POWER 1  //Default setting is Low Power
#endif /*!defined MCU_LOW_POWER*/

#ifndef RADIO_LOW_POWER
#define RADIO_LOW_POWER 1 //Default setting is Low Power
#endif /*!defined RADIO_LOW_POWER*/

#if RADIO_LOW_POWER == 0
#define RADIO_USES_SNIFF_MODE 0
#define RADIO_USES_CONTIKIMAC 0
#else /*RADIO_LOW_POWER ==1*/
/*CHOOSE EITHER RADIO_USES_SNIFF_MODE OR RADIO_USES_CONTIKIMAC:*/
#define RADIO_USES_SNIFF_MODE 1
#define RADIO_USES_CONTIKIMAC 0
#if RADIO_USES_SNIFF_MODE  //Default validated setting
#undef RADIO_USES_LONG_PREAMBLE
#define RADIO_USES_LONG_PREAMBLE 1
#elif RADIO_USES_CONTIKIMAC /*ContikiMAC is still experimental*/
#define RADIO_USES_LONG_PREAMBLE 0
#endif /*RADIO_USES_SNIFF_MODE || RADIO_USES_CONTIKIMAC */
#endif /*RADIO_LOW_POWER == 0*/

#if RADIO_USES_CONTIKIMAC && RADIO_USES_SNIFF_MODE
#error "Choose at most one betwween SNIFF and ContikiMAC"
#endif /*RADIO_USES_CONTIKIMAC && RADIO_USES_SNIFF_MODE*/

#ifdef SPIRIT1_RADIO
#if RADIO_USES_CONTIKIMAC || RADIO_USES_SNIFF_MODE || MCU_LOW_POWER
#error "Low Power modes implemented only for L1 MCU and S2-LP Radio."
#endif /*Any low power */
#endif /*SPIRIT1_RADIO*/


#ifdef USE_STM32L1XX_NUCLEO
#if MCU_LOW_POWER
/*  RTC Wakeup Interrupt Generation:
		  Wakeup Time Base = (RTC_WAKEUPCLOCK_RTCCLK_DIV /(LSI))
		  Wakeup Time = Wakeup Time Base * WakeUpCounter
					  = (RTC_WAKEUPCLOCK_RTCCLK_DIV /(LSI)) * WakeUpCounter
		  ==> WakeUpCounter = Wakeup Time / Wakeup Time Base
			RTC_WAKEUPCLOCK_RTCCLK_DIV = RTCCLK_Div16 = 16
			Wakeup Time Base = 16 /(37KHz)
			Wakeup Time Conversion (to multiply by required seconds) = 1 / Wakeup Time Base = 37KHz / 16
 */
#define LP_WAKEUP_TIME_CONVERSION (LSI_VALUE/16)

/*This is the frequency the core is set to when is Sleep mode*/
#define LOWEST_USER_CLOCK_FREQUENCY 4194
#define SLEEP_CLOCK_FREQUENCY_HZ 65536
#define SLEEP_CLOCK_FREQUENCY (SLEEP_CLOCK_FREQUENCY_HZ/1000) //65 KHz

#if RADIO_USES_CONTIKIMAC
#warning "Support for ContikiMAC together with MCU Sleep mode is still experimental."
extern int contikimac_CCA_in_progrss;
#define USER_CLOCK_FREQUENCY DEFAULT_CLOCK_FREQUENCY //Only working setting in this case
#define CONSECUTIVE_SLEEPS 2
#else //SNIFF MODE
/*Choose a Clock Frequency that can be the default one or a lower one for a lower consumption. 4MHz is supported at the moment*/
//#define USER_CLOCK_FREQUENCY DEFAULT_CLOCK_FREQUENCY
#define USER_CLOCK_FREQUENCY LOWEST_USER_CLOCK_FREQUENCY
#endif /*RADIO_USES_CONTIKIMAC*/

#if USER_CLOCK_FREQUENCY == DEFAULT_CLOCK_FREQUENCY
/* Compensate Ticks to adjust the clock after SLEEP depend on the system frequency but also on the compiler
 * (__WFI() implementations differ).
 * Adjust these values to your application (you can check that application level timers are ok).
 * If a timer expires too quickly, the COMPENSATE_TICKS value must be lowered, and vice versa.
 * */
#if defined (__IAR_SYSTEMS_ICC__)
#define COMPENSATE_TICKS 22
#elif defined (__CC_ARM)
#define COMPENSATE_TICKS 35
#elif defined (__GNUC__)
#define COMPENSATE_TICKS 65
#endif /*Toolhain*/
#elif USER_CLOCK_FREQUENCY == LOWEST_USER_CLOCK_FREQUENCY
#if defined (__IAR_SYSTEMS_ICC__)
#define COMPENSATE_TICKS 17
#elif defined (__CC_ARM)
#define COMPENSATE_TICKS 27
#elif defined (__GNUC__)
#define COMPENSATE_TICKS 48
#endif /*Toolchain*/
#else
#error "The selected USER_CLOCK_FREQUENCY value needs to be validated: chosse proper COMPENSATE_TICKS number."
#endif /*USER_CLOCK_FREQUENCY == DEFAULT_CLOCK_FREQUENCY*/

/* Public functions prototypes */
void LP_enter_stop_mode(int stop_seconds);
void LP_exit_stop_mode(int wakeup_radio);
void LP_enter_sleep_mode(void);
void LP_exit_sleep_mode (void);
void LP_sysclk_config(unsigned int clk_freq);

#else /*!MCU_LOW_POWER*/
#define USER_CLOCK_FREQUENCY DEFAULT_CLOCK_FREQUENCY
#endif /*MCU_LOW_POWER*/

extern int LP_interrupt_from_radio;
#endif /*USE_STM32L1XX_NUCLEO*/

#ifdef USE_STM32F4XX_NUCLEO
#if RADIO_USES_CONTIKIMAC || RADIO_USES_SNIFF_MODE || MCU_LOW_POWER
#error "Low Power modes implemented only for L1 MCU and S2-LP Radio."
#endif /*Any low power */

#define USER_CLOCK_FREQUENCY DEFAULT_CLOCK_FREQUENCY
#endif /*USE_STM32F4XX_NUCLEO*/

#if  RADIO_USES_CONTIKIMAC
//Currently we enable this only for ContikiMAC case. It gives an improvement also for SNIFF mode but it is not stable, @TODO.
#define LP_PERIPHERAL_IN_SLEEP 1
#else
#define LP_PERIPHERAL_IN_SLEEP 0
#endif /*RADIO_USES_CONTIKIMAC*/

extern unsigned long F_CPU;
#endif /*_LOW_POWER_H_*/

/**
* @}
*/
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
