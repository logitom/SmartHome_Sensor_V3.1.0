/**
******************************************************************************
* @file    radio_spi.h
* @author  Central Labs
* @version V1.0.1
* @date    10-Mar-2016
* @brief   This file contains all the functions prototypes for SPI .
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


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RADIO_SPI_H
#define __RADIO_SPI_H
#ifdef __cplusplus
extern "C" {
#endif
  
/* Includes ------------------------------------------------------------------*/
#include "cube_hal.h"
#include "radio_gpio.h"

  
/**
 * @addtogroup BSP
 * @{
 */ 
  
/* Exported types ------------------------------------------------------------*/
   
/* Exported constants --------------------------------------------------------*/
  
  
/* Exported macro ------------------------------------------------------------*/
  /* Define for SPIRIT1 board  */  
// #if !defined (USE_SPIRIT1_DEFAULT)
// #define USE_SPIRIT1_DEFAULT
//#endif
  
/* SPIRIT1_Spi_config */ 
/* SPI1 */  
  
#define RADIO_SPI                                 SPI1
#define RADIO_SPI_CLK_ENABLE()                  __SPI1_CLK_ENABLE()
#define RADIO_SPI_CLK_DISABLE()                 __SPI1_CLK_DISABLE()

/* Defines for MISO pin*/
#define RADIO_SPI_MISO_PORT                      GPIOA
#define RADIO_SPI_MISO_PIN                       GPIO_PIN_6 
#if defined(USE_STM32F4XX_NUCLEO)||(USE_STM32L1XX_NUCLEO)
#define RADIO_SPI_MISO_AF                       GPIO_AF5_SPI1
#endif
#if defined(USE_STM32L0XX_NUCLEO)
#define RADIO_SPI_MISO_AF                       GPIO_AF0_SPI1
#endif
#define RADIO_SPI_MISO_CLK_ENABLE()            __GPIOA_CLK_ENABLE()
#define RADIO_SPI_MISO_CLK_DISABLE()           __GPIOA_CLK_DISABLE() 
  
/* Defines for MOSI pin*/  
#define RADIO_SPI_MOSI_PORT                      GPIOA
#define RADIO_SPI_MOSI_PIN                       GPIO_PIN_7 
#if defined(USE_STM32F4XX_NUCLEO)||(USE_STM32L1XX_NUCLEO)
#define RADIO_SPI_MOSI_AF                       GPIO_AF5_SPI1
#endif
#if defined(USE_STM32L0XX_NUCLEO)
#define RADIO_SPI_MOSI_AF                       GPIO_AF0_SPI1
#endif

#define RADIO_SPI_MOSI_CLK_ENABLE()            __GPIOA_CLK_ENABLE()
#define RADIO_SPI_MOSI_CLK_DISABLE()           __GPIOA_CLK_DISABLE()   
  

/* Defines for SCLK pin */  
#define RADIO_SPI_SCK_PORT                      GPIOB
#define RADIO_SPI_SCK_PIN                       GPIO_PIN_3
  #if defined(USE_STM32F4XX_NUCLEO)||(USE_STM32L1XX_NUCLEO)
#define RADIO_SPI_SCK_AF                       GPIO_AF5_SPI1
#endif
#if defined(USE_STM32L0XX_NUCLEO)
#define RADIO_SPI_SCK_AF                       GPIO_AF0_SPI1
#endif
#define RADIO_SPI_SCLK_CLK_ENABLE()            __GPIOB_CLK_ENABLE()
#define RADIO_SPI_SCLK_CLK_DISABLE()           __GPIOB_CLK_DISABLE()

 /* Defines for chip select pin */  
#define RADIO_SPI_CS_PORT                        GPIOA
#define RADIO_SPI_CS_PIN                         GPIO_PIN_1
#define RADIO_SPI_CS_CLK_ENABLE()            __GPIOA_CLK_ENABLE()
#define RADIO_SPI_CS_CLK_DISABLE()           __GPIOA_CLK_DISABLE()
  
#define RadioSpiCSLow()        HAL_GPIO_WritePin(RADIO_SPI_CS_PORT, RADIO_SPI_CS_PIN, GPIO_PIN_RESET)
#define RadioSpiCSHigh()       HAL_GPIO_WritePin(RADIO_SPI_CS_PORT, RADIO_SPI_CS_PIN, GPIO_PIN_SET)

  

   
#define EEPROM_SPI_PERIPH_NB                  SPI1
#define EEPROM_SPI_PERIPH_RCC                 __SPI1_CLK_ENABLE
  

/* Defines for MOSI pin*/
#define EEPROM_SPI_PERIPH_MOSI_PORT           GPIOA
#define EEPROM_SPI_PERIPH_MOSI_PIN            GPIO_PIN_7
#if defined(USE_STM32F4XX_NUCLEO)||(USE_STM32L1XX_NUCLEO)
#define EEPROM_SPI_PERIPH_MOSI_AF                       GPIO_AF5_SPI1
#endif
#if defined(USE_STM32L0XX_NUCLEO)
#define EEPROM_SPI_PERIPH_MOSI_AF                       GPIO_AF0_SPI1
#endif

#define EEPROM_SPI_PERIPH_MOSI_RCC            __GPIOA_CLK_ENABLE
#define EEPROM_SPI_PERIPH_MOSI_RCC_SOURCE     GPIO_PinSource7

/* Defines for MISO pin */
#define EEPROM_SPI_PERIPH_MISO_PORT           GPIOA
#define EEPROM_SPI_PERIPH_MISO_PIN            GPIO_PIN_6
  
#if defined(USE_STM32F4XX_NUCLEO)||(USE_STM32L1XX_NUCLEO)
#define EEPROM_SPI_PERIPH_MISO_AF                       GPIO_AF5_SPI1
#endif
#if defined(USE_STM32L0XX_NUCLEO)
#define EEPROM_SPI_PERIPH_MISO_AF                       GPIO_AF0_SPI1
#endif
  
#define EEPROM_SPI_PERIPH_MISO_RCC            __GPIOA_CLK_ENABLE
#define EEPROM_SPI_PERIPH_MISO_RCC_SOURCE     GPIO_PinSource6

/* Defines for SCLK pin */
#define EEPROM_SPI_PERIPH_SCLK_PORT           GPIOB
#define EEPROM_SPI_PERIPH_SCLK_PIN            GPIO_PIN_3 

#if defined(USE_STM32F4XX_NUCLEO)||(USE_STM32L1XX_NUCLEO)
#define EEPROM_SPI_PERIPH_SCLK_AF                       GPIO_AF5_SPI1
#endif
#if defined(USE_STM32L0XX_NUCLEO)
#define EEPROM_SPI_PERIPH_SCLK_AF                       GPIO_AF0_SPI1
#endif
  
#define EEPROM_SPI_PERIPH_SCLK_RCC            __GPIOB_CLK_ENABLE
#define EEPROM_SPI_PERIPH_SCLK_RCC_SOURCE     GPIO_PinSource3

 
/* Defines for EEPROM chip select pin (PA9 for STEVAL-FKI868V1 and PB4 for X-NUCLEO-S2868A1)*/
#if 0 //indar
#define EEPROM_SPI_PERIPH_CS_PORT             GPIOA
#define EEPROM_SPI_PERIPH_CS_PIN              GPIO_PIN_9
#define EEPROM_SPI_PERIPH_CS_RCC              __GPIOA_CLK_ENABLE
#define EEPROM_SPI_PERIPH_CS_RCC_SOURCE       GPIO_PinSource9
#else
#define EEPROM_SPI_PERIPH_CS_PORT             GPIOB
#define EEPROM_SPI_PERIPH_CS_PIN              GPIO_PIN_4
#define EEPROM_SPI_PERIPH_CS_RCC              __GPIOB_CLK_ENABLE
#define EEPROM_SPI_PERIPH_CS_RCC_SOURCE       GPIO_PinSource4

#endif
 
/* Maximum Timeout values for flags waiting loops. These timeouts are not based
   on accurate values, they just guarantee that the application will not remain
   stuck if the SPI communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...) */    
#define RADIO_SPI_TIMEOUT_MAX                   ((uint32_t)1000)

/* SPIRIT1_Spi_config_Private_Defines */  
#ifdef STM32F401xE
#define CS_TO_SCLK_DELAY     0x0200
#define CLK_TO_CS_DELAY      0x0200
#endif

#ifdef STM32L152xE
#define CS_TO_SCLK_DELAY     0x0100
#define CLK_TO_CS_DELAY      0x0100
#endif

#ifdef STM32L053xx
#define CS_TO_SCLK_DELAY     0x0100
#define CLK_TO_CS_DELAY      0x0100
#endif
  
#define SPI_ENTER_CRITICAL()          RadioGpioInterruptCmd(RADIO_GPIO_IRQ,0x04,0x04,DISABLE);//kg
#define SPI_EXIT_CRITICAL()           RadioGpioInterruptCmd(RADIO_GPIO_IRQ,0x04,0x04,ENABLE); 

#ifdef __cplusplus
}
#endif
#endif /*__RADIO_SPI_H */

/**
* @}
*/

/**
* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
