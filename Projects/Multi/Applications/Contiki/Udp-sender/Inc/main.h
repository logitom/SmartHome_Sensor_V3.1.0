/**
******************************************************************************
* @file    main.h
* @author  MCD Application Team
* @version V1.0.0
*
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
*
* Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
* You may not use this file except in compliance with the License.
* You may obtain a copy of the License at:
*
*        http://www.st.com/software_license_agreement_liberty_v2
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*
******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

  /* Includes ------------------------------------------------------------------*/
#ifdef USE_STM32F4XX_NUCLEO
#include "stm32f4xx_hal.h"
#include "stm32f4xx_nucleo.h"
#endif

#ifdef USE_STM32L1XX_NUCLEO
#include "stm32l1xx_hal.h"
#include "stm32l1xx_nucleo.h"
#endif

#include "platform-conf.h"



  /* Exported types ------------------------------------------------------------*/
  /* Exported constants --------------------------------------------------------*/

  /* Uncomment to enable the adaquate RTC Clock Source */
#ifdef USE_STM32F4XX_NUCLEO

#define RTC_CLOCK_SOURCE_LSI


#ifdef RTC_CLOCK_SOURCE_LSI
  #define RTC_ASYNCH_PREDIV  0x7F
  #define RTC_SYNCH_PREDIV   0x0130
#endif

#ifdef RTC_CLOCK_SOURCE_LSE
  #define RTC_ASYNCH_PREDIV  0x7F
  #define RTC_SYNCH_PREDIV   0x00FF
#endif

#endif

#ifdef USE_STM32L1XX_NUCLEO
#define RTC_CLOCK_SOURCE_LSI

#ifdef RTC_CLOCK_SOURCE_LSI
  #define RTC_ASYNCH_PREDIV  0x7F
  #define RTC_SYNCH_PREDIV   0x0130
#endif

#ifdef RTC_CLOCK_SOURCE_LSE
  #define RTC_ASYNCH_PREDIV  0x7F
  #define RTC_SYNCH_PREDIV   0x00FF
#endif

#endif
#define I2Cx                            I2C1
#define I2Cx_CLK_ENABLE()               __HAL_RCC_I2C1_CLK_ENABLE()
#define I2Cx_SDA_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define I2Cx_SCL_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE() 
#define I2Cx_DMA_CLK_ENABLE()           __HAL_RCC_DMA1_CLK_ENABLE()

#define I2Cx_FORCE_RESET()              __HAL_RCC_I2C1_FORCE_RESET()
#define I2Cx_RELEASE_RESET()            __HAL_RCC_I2C1_RELEASE_RESET()

/* Definition for I2Cx Pins */
#define I2Cx_SCL_PIN                    GPIO_PIN_8
#define I2Cx_SCL_GPIO_PORT              GPIOB
#define I2Cx_SDA_PIN                    GPIO_PIN_9
#define I2Cx_SDA_GPIO_PORT              GPIOB
#define I2Cx_SCL_SDA_AF                 GPIO_AF4_I2C1

/* Definition for I2Cx's NVIC */
#define I2Cx_EV_IRQn                    I2C1_EV_IRQn
#define I2Cx_ER_IRQn                    I2C1_ER_IRQn
#define I2Cx_EV_IRQHandler              I2C1_EV_IRQHandler
#define I2Cx_ER_IRQHandler              I2C1_ER_IRQHandler

/* Definition for I2Cx's DMA */
#define I2Cx_DMA                        DMA1   
#define I2Cx_DMA_INSTANCE_TX            DMA1_Channel6
#define I2Cx_DMA_INSTANCE_RX            DMA1_Channel7

/* Definition for I2Cx's DMA NVIC */
#define I2Cx_DMA_TX_IRQn                DMA1_Channel6_IRQn
#define I2Cx_DMA_RX_IRQn                DMA1_Channel7_IRQn
#define I2Cx_DMA_TX_IRQHandler          DMA1_Channel6_IRQHandler
#define I2Cx_DMA_RX_IRQHandler          DMA1_Channel7_IRQHandler

/* Size of Transmission buffer */
#define TXBUFFERSIZE                      (COUNTOF(aTxBuffer) - 1)
/* Size of Reception buffer */
#define RXBUFFERSIZE                      TXBUFFERSIZE

/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/* Exported functions ------------------------------------------------------- */


struct sensor_pkt
{
    uint8_t cmd;
    uint8_t device_type;
    uint8_t alarm_status;
    uint8_t index;
    uint8_t status;
    uint8_t sensor_type;
    uint8_t sensor_data;
    uint8_t battery;
};



void Sent_Testing_Data(void);

#ifdef __cplusplus
}
#endif


#endif /* __MAIN */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

