/******************************************************************************/
/**
 * @file    uart.h
 * @author  Lyle Bainbridge
 *
 * @section LICENSE
 *
 * COPYRIGHT 2014 STMicroelectronics
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
 * @section DESCRIPTION
 *
 * Driver for the STM32L USART1 peripheral.
 */

#ifndef __UART_H__
#define __UART_H__

/*******************************************************************************
 * Include Files
 */
#include <stdint.h>
#include "fifo.h"


#ifdef ENABLE_VCOM
#define PRINTF(...)             printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

   
#ifdef IAR
#include <yfuns.h>
#else
   //#define size_t int
#define _LLIO_STDIN  0
#define _LLIO_STDOUT 1
#define _LLIO_STDERR 2
#define _LLIO_ERROR ((size_t)-1) /* For __read and __write. */
#endif
   
#ifdef STM32L053xx  
#define  TERMINAL_BUFF_SIZE                     600 //kg
#else
#define  TERMINAL_BUFF_SIZE                   	600                   
#endif

#define NUCLEO_UARTx_RX_QUEUE_SIZE              (1224)
//#define NUCLEO_UARTx_TX_QUEUE_SIZE              (3*1024)  


#define RECEIVE_QUEUE_SIZE              NUCLEO_UARTx_RX_QUEUE_SIZE



#define MAX_ECHO_BUFFER_SIZE            100
#define RX_FIFO_SIZE                    256
#define TX_FIFO_SIZE                    256
                                  
#define USARTx                           USART2
   
#define UART_TX_PORT                     GPIOA
#define USARTx_TX_PIN                    GPIO_PIN_2                
#define USARTx_TX_GPIO_PORT              GPIOA                       
#define USARTx_TX_GPIO_CLK               RCC_AHBPeriph_GPIOA
#define USARTx_TX_SOURCE                 GPIO_PinSource2

#ifdef USE_STM32F4XX_NUCLEO
#define USARTx_TX_AF                     GPIO_AF7_USART2
#endif

#ifdef USE_STM32L1XX_NUCLEO
#define USARTx_TX_AF                     GPIO_AF7_USART2
#endif

#ifdef USE_STM32L0XX_NUCLEO
#define USARTx_TX_AF                     GPIO_AF4_USART2
#endif


#define UART_RX_PORT                     GPIOA
#define USARTx_RX_PIN                    GPIO_PIN_3                
#define USARTx_RX_GPIO_PORT              GPIOA                    
#define USARTx_RX_GPIO_CLK               RCC_AHBPeriph_GPIOA
#define USARTx_RX_SOURCE                 GPIO_PinSource3

#ifdef USE_STM32F4XX_NUCLEO
#define USARTx_RX_AF                     GPIO_AF7_USART2
#endif

#ifdef USE_STM32L1XX_NUCLEO
#define USARTx_RX_AF                     GPIO_AF7_USART2
#endif

#ifdef USE_STM32L0XX_NUCLEO
#define USARTx_RX_AF                     GPIO_AF4_USART2
#endif

   
   
#define USARTx_IRQn                      USART2_IRQn 
#define USARTx_IRQHandler                USART2_IRQHandler
   



#ifdef USE_STM32F4XX_NUCLEO
/* Definition for USARTx's DMA */
#define USARTx_TX_DMA_CHANNEL             DMA1_Stream6
#define USARTx_RX_DMA_CHANNEL             DMA1_Stream5
   
#define USARTx_DMA_CHANNEL                DMA_CHANNEL_4   

/* Definition for USARTx's NVIC */
#define USARTx_DMA_TX_IRQn                DMA1_Stream6_IRQn
#define USARTx_DMA_RX_IRQn                DMA1_Stream5_IRQn
#define USARTx_DMA_TX_IRQHandler          DMA1_Stream6_IRQHandler
#define USARTx_DMA_RX_IRQHandler          DMA1_Stream5_IRQHandler

#endif

#ifdef USE_STM32L1XX_NUCLEO
/* Definition for USARTx's DMA */
#define USARTx_TX_DMA_CHANNEL             DMA1_Channel7
#define USARTx_RX_DMA_CHANNEL             DMA1_Channel6

/* Definition for USARTx's NVIC */
#define USARTx_DMA_TX_IRQn                DMA1_Channel7_IRQn
#define USARTx_DMA_RX_IRQn                DMA1_Channel6_IRQn
#define USARTx_DMA_TX_IRQHandler          DMA1_Channel7_IRQHandler
#define USARTx_DMA_RX_IRQHandler          DMA1_Channel6_IRQHandler

#endif

#ifdef USE_STM32L0XX_NUCLEO
/* Definition for USARTx's DMA */
#define USARTx_TX_DMA_CHANNEL             DMA1_Channel4
#define USARTx_RX_DMA_CHANNEL             DMA1_Channel5

/* Definition for USARTx's NVIC */
#define USARTx_DMA_TX_IRQn                DMA1_Channel4_5_6_7_IRQn
#define USARTx_DMA_RX_IRQn                DMA1_Channel4_5_6_7_IRQn
#define USARTx_DMA_TX_RX_IRQHandler       DMA1_Channel4_5_6_7_IRQHandler

#endif




#define DMAx_CLK_ENABLE()                __HAL_RCC_DMA1_CLK_ENABLE() 

#define USARTx_FORCE_RESET()             __HAL_RCC_USART2_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __HAL_RCC_USART2_RELEASE_RESET()
   
/*******************************************************************************
 * Functions
 */
fifo_t* uart_get_rx_fifo(void);
void uart_init(uint32_t baud_rate);
void uart_set_baud_rate(uint32_t baud_rate);
void uart_sendDMA(uint8_t* p_buf, uint16_t len);
void USART1_HandlerUser(void);
void SendTimeStamp(void);
void Error_Handler(void);
void SendDataToTerminal(void);
#endif
