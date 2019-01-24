/******************************************************************************/
/**
 * @file    uart.c
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

/*******************************************************************************
 * Include Files
 */

#include <stdio.h>
#include <string.h>
#include "fifo.h"
#include "uart.h"
#include "wmbus_link.h"
#include "command.h"

#ifdef USE_STM32F4XX_NUCLEO
#include <stm32f4xx.h>
#include "stm32f4xx_hal_uart.h" 
#endif

#ifdef USE_STM32L1XX_NUCLEO
#include <stm32l1xx.h>
#include "stm32l1xx_hal_uart.h" 
#endif

#ifdef USE_STM32L0XX_NUCLEO
#include <stm32l0xx.h>
#include "stm32l0xx_hal_uart.h" 
#endif

   
/*******************************************************************************
 * Definitions
 */


/*******************************************************************************
 * Global Variables
 */
UART_HandleTypeDef Huart2;
fifo_t*    g_p_uart_rx_fifo = 0;
__IO ITStatus UartReady = RESET;
uint8_t RxBuffer[MAX_ECHO_BUFFER_SIZE];
uint8_t SingleByte;
uint16_t NumDataRecvd = 0;
uint8_t RespRcvdFlag = 0;
uint8_t g_rx_fifo_data[RX_FIFO_SIZE];
fifo_t g_rx_fifo;
fifo_t g_tx_fifo;
uint8_t g_rx_fifo_data[RX_FIFO_SIZE];
uint8_t g_tx_fifo_data[TX_FIFO_SIZE];

static uint8_t rxQ[RECEIVE_QUEUE_SIZE];
static uint16_t rxHead = 0;
static uint16_t rxTail = 0;
volatile uint16_t rxUsed = 0;


uint8_t TerminalFifoBuffer[TERMINAL_BUFF_SIZE];
fifo_t  TerminalFifo;


/*******************************************************************************
 * Exported Variables
 */

extern fifo_t g_rx_fifo;
extern uint32_t SysTickTimeStamp;
extern fifo_t g_rx_fifo;
extern fifo_t g_tx_fifo;
/******************************/

/** RX frame buffer size */
//extern uint8_t RcvdFrameBuff[];
//extern uint8_t TransmitFrameBuff[];
//extern uint8_t RcvdFrameFlag , TransmitFrameFlag;
//extern uint16_t RcvdFrameBuffSize , TransmitFrameBuffSize;
extern volatile LL_State_t LL_State;
//extern uint8_t TempDataBuff[];
/*******************************************************************************/
/******************************************************************************/
/**
 * @brief Iniialize the UART and set to the specified baud rate.
 *
 * @param baud_rate  The UART baud rate to set.
 */
void uart_init(uint32_t baud_rate)
{
  static DMA_HandleTypeDef hdma_tx;
  static DMA_HandleTypeDef hdma_rx;
  
  GPIO_InitTypeDef GPIO_InitStruct;
  
  
  if(HAL_UART_DeInit(&Huart2) != HAL_OK)
  {
    Error_Handler();
  }  
  
  HAL_UART_MspDeInit(&Huart2);
  
  /* Peripheral clock enable */
  __HAL_RCC_USART2_CLK_ENABLE();
  
  /* Enable DMA clock */
  DMAx_CLK_ENABLE();
  
  /**USART2 GPIO Configuration    
  PA2     ------> USART2_TX
  PA3     ------> USART2_RX 
  */
  GPIO_InitStruct.Pin = USARTx_TX_PIN | USARTx_RX_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = USARTx_TX_AF;
  HAL_GPIO_Init(UART_TX_PORT, &GPIO_InitStruct);
  
  
  Huart2.Instance = USARTx;
  Huart2.Init.BaudRate = baud_rate;
  Huart2.Init.WordLength = UART_WORDLENGTH_8B;
  Huart2.Init.StopBits = UART_STOPBITS_1;
  Huart2.Init.Parity = UART_PARITY_NONE;
  Huart2.Init.Mode = UART_MODE_TX_RX;
  Huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  Huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  
  if (HAL_UART_Init(&Huart2) != HAL_OK)
  {
    Error_Handler();
  }
  
  
  /*##-3- Configure the DMA ##################################################*/
  /* Configure the DMA handler for Transmission process */
  hdma_tx.Instance                 = USARTx_TX_DMA_CHANNEL;
  
#ifdef USE_STM32L0XX_NUCLEO
  hdma_tx.Init.Request           = DMA_REQUEST_4;
#endif
  
#ifdef USE_STM32F4XX_NUCLEO
  hdma_tx.Init.Channel           = USARTx_DMA_CHANNEL;
#endif

  hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
  hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  hdma_tx.Init.Mode                = DMA_NORMAL;
  hdma_tx.Init.Priority            = DMA_PRIORITY_LOW;
  
  HAL_DMA_Init(&hdma_tx);
  
  /* Associate the initialized DMA handle to the UART handle */
  __HAL_LINKDMA(&Huart2, hdmatx, hdma_tx);
  
  /* Configure the DMA handler for reception process */
  hdma_rx.Instance                 = USARTx_RX_DMA_CHANNEL;
  
#ifdef USE_STM32L0XX_NUCLEO
  hdma_rx.Init.Request           = DMA_REQUEST_4;
#endif
  
#ifdef USE_STM32F4XX_NUCLEO
  hdma_rx.Init.Channel           = USARTx_DMA_CHANNEL;
#endif
  hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  hdma_rx.Init.Mode                = DMA_NORMAL;
  hdma_rx.Init.Priority            = DMA_PRIORITY_HIGH;
  
  HAL_DMA_Init(&hdma_rx);

  /* Associate the initialized DMA handle to the the UART handle */
  __HAL_LINKDMA(&Huart2, hdmarx, hdma_rx);
    
  /*##-4- Configure the NVIC for DMA #########################################*/
  /* NVIC configuration for DMA transfer complete interrupt (USART2_TX) */
  HAL_NVIC_SetPriority(USARTx_DMA_TX_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USARTx_DMA_TX_IRQn);
    
  /* NVIC configuration for DMA transfer complete interrupt (USART2_RX) */
  HAL_NVIC_SetPriority(USARTx_DMA_RX_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USARTx_DMA_RX_IRQn);
  
    /* NVIC for USART2 */
  HAL_NVIC_SetPriority(USARTx_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(USARTx_IRQn);
  
  
  /***********  Put UART peripheral in reception process  ********/  
  if(HAL_UART_Receive_DMA(&Huart2, &SingleByte, 1) != HAL_OK)
  {
    Error_Handler();
  }
  
  
  g_rx_fifo.p_data = g_rx_fifo_data;
  g_rx_fifo.size = RX_FIFO_SIZE;
  fifo_init(&g_rx_fifo);
  
  g_tx_fifo.p_data = g_tx_fifo_data;
  g_tx_fifo.size = TX_FIFO_SIZE;
  fifo_init(&g_tx_fifo);
  
  g_p_uart_rx_fifo = uart_get_rx_fifo();
  
  
  TerminalFifo.p_data = TerminalFifoBuffer;                                     
  TerminalFifo.size = TERMINAL_BUFF_SIZE;
  fifo_init(&TerminalFifo);
}



/**
  * @brief UART MSP De-Initialization
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO, DMA and NVIC configuration to their default state
  * @param Huart2: UART handle pointer
  * @retval None
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef *Huart2)
{

  /*##-1- Reset peripherals ##################################################*/
  USARTx_FORCE_RESET();
  USARTx_RELEASE_RESET();

  /*##-2- Disable peripherals and GPIO Clocks #################################*/
  /* Configure USARTx Tx as alternate function  */
  HAL_GPIO_DeInit(USARTx_TX_GPIO_PORT, USARTx_TX_PIN);
  /* Configure USARTx Rx as alternate function  */
  HAL_GPIO_DeInit(USARTx_RX_GPIO_PORT, USARTx_RX_PIN);
   
  /*##-3- Disable the DMA #####################################################*/
  /* De-Initialize the DMA channel associated to reception process */
  if(Huart2->hdmarx != 0)
  {
    HAL_DMA_DeInit(Huart2->hdmarx);
  }
  /* De-Initialize the DMA channel associated to transmission process */
  if(Huart2->hdmatx != 0)
  {
    HAL_DMA_DeInit(Huart2->hdmatx);
  }  
  
  /*##-4- Disable the NVIC for DMA ###########################################*/
  HAL_NVIC_DisableIRQ(USARTx_DMA_TX_IRQn);
  HAL_NVIC_DisableIRQ(USARTx_DMA_RX_IRQn);
}



/******************************************************************************/
/**
 * @brief Set the UART to the specified baud rate.
 *
 * This function is called to change the baud rate after the UART is already
 * initialized.
 *
 * @param baud_rate  The UART baud rate to set.
 */
void uart_set_baud_rate(uint32_t baud_rate)
{
  Huart2.Instance = USARTx;
  Huart2.Init.BaudRate = baud_rate;
  Huart2.Init.WordLength = UART_WORDLENGTH_8B;
  Huart2.Init.StopBits = UART_STOPBITS_1;
  Huart2.Init.Parity = UART_PARITY_NONE;
  Huart2.Init.Mode = UART_MODE_TX_RX;
  Huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  Huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  
  if (HAL_UART_Init(&Huart2) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* NVIC for USART2 */
  HAL_NVIC_SetPriority(USARTx_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(USARTx_IRQn);
  
  /***********  Put UART peripheral in reception process  ********/  
  if(HAL_UART_Receive_IT(&Huart2, &SingleByte, 1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief Send the specified buffer to the UART.
 *
 * This function is called to send a variable length buffer of data to the
 * UART.  The UART must already have been initialized.  The function does not
 * return until all of the data has been transmitted.
 *
 * @param buf  A pointer to the buffer to send to the UART.
 * @param len  The length of the buffer to be sent to the UART.
 */
void uart_sendDMA(uint8_t* p_buf, uint16_t len)
{    
  if(HAL_UART_Transmit_DMA(&Huart2, (uint8_t *)p_buf, len) != HAL_OK)
  {
    Error_Handler();
  }
  
  do
  {
  }
  while (UartReady != SET);
  
  UartReady = RESET;
}

/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle. 
  * @note   This example shows a simple way to report end of DMA Tx transfer,and 
  *         you can add your own implementation. 
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: trasfer complete*/
  UartReady = SET;

}

/**
* @brief  This function is UART Interrupt Handler.
* @param  None
* @retval None
*/
void USARTx_IRQHandler(void)
{
  HAL_UART_IRQHandler(&Huart2);
}


/**
  * @brief  Rx Transfer completed callbacks.
  * @param  Huart2: Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *Huart2)
{
  RxBuffer[NumDataRecvd] = SingleByte;
  NumDataRecvd++;
  
  if(HAL_UART_Receive_DMA(Huart2, &SingleByte , 1) != HAL_OK)
  {
    Error_Handler();
  }
  
  
  fifo_queue(&g_rx_fifo, SingleByte);
  
  if(SingleByte == '\n')
  {
    RespRcvdFlag = 1;
    NumDataRecvd = 0;
  }
}


/**
  * @brief  Sends the wM-Bus Packet o the Terminal.
  * @param  None.
  * @retval None.
  */
void SendDataToTerminal(void)
{
  if(LL_State == LL_SM_IDLE)
  {
    if(TerminalFifo.count)
    {
      uart_sendDMA(TerminalFifo.p_data, TerminalFifo.count);
      fifo_init(&TerminalFifo);
    }
  }
}


/**
 * @brief Get a pointer to the UART receive fifo.
 *
 * @return Pointer to the UART receive fifo.
 */
fifo_t* uart_get_rx_fifo(void)
{
    return &g_rx_fifo;
}


/**
 * @brief Send the timestamp to the Terminal.
 *
 * @return None.
 */
void SendTimeStamp(void)
{
  char timeStampStr[10];
  
  sprintf(timeStampStr,"%lu",SysTickTimeStamp);
  
  uart_sendDMA((uint8_t*)timeStampStr, strlen(timeStampStr));
}


// IAR Standard library hook for serial output
size_t __write(int handle, const unsigned char * buffer, size_t size)
{
  
  /* This template only writes to "standard out" and "standard err",
   * for all other file handles it returns failure. */
  if (handle != _LLIO_STDOUT && handle != _LLIO_STDERR) {
    return _LLIO_ERROR;
  }
  if (buffer == 0) {
    uint8_t c=0;
    uart_sendDMA(&c, 1/*, 100*/);
    return 0;
  }
  
  
  uart_sendDMA((uint8_t*)buffer, size/*, 300*/);

  return size;
}

/*kg
size_t fflush (int handle)
{
  return __write(_LLIO_STDOUT, NULL, 0);
}
*/


// IAR Standard library hook for serial input
size_t __read(int handle, unsigned char * buffer, size_t size)
{
  int nChars = 0;
  
  /* This template only reads from "standard in", for all other file
  * handles it returns failure. */
  if (handle != _LLIO_STDIN)
  {
    return _LLIO_ERROR;
  }
  
  if(Huart2.hdmarx)
  {
#ifdef USE_STM32F4XX_NUCLEO
    rxHead=(RECEIVE_QUEUE_SIZE-Huart2.hdmarx->Instance->NDTR)%RECEIVE_QUEUE_SIZE;
#else
    rxHead=(RECEIVE_QUEUE_SIZE-Huart2.hdmarx->Instance->CNDTR)%RECEIVE_QUEUE_SIZE;
#endif
    
    
    if(rxHead>=rxTail)
      rxUsed=rxHead-rxTail;
    else
      rxUsed=RECEIVE_QUEUE_SIZE-rxTail+rxHead;
    
  }
  
  for(nChars = 0; (rxUsed>0) && (nChars < size); nChars++) {
    *buffer++ = rxQ[rxTail];
    rxTail = (rxTail+1) % RECEIVE_QUEUE_SIZE;
    rxUsed--;
  }
  
  return nChars;
}

void enqueueRxChars(unsigned char * buffer, uint16_t size)
{
  while (( size > 0 ) && (rxUsed < (RECEIVE_QUEUE_SIZE-1))) {
      rxQ[rxHead] = *buffer++;
      rxHead = (rxHead+1) % RECEIVE_QUEUE_SIZE;
      rxUsed++;
      size--;
  }
}

uint8_t __io_getcharNonBlocking(uint8_t *data)
{
  if (__read(_LLIO_STDIN,data,1))
    return 1;
  else
    return 0;
}

void __io_putchar( char c )
{
  __write(_LLIO_STDOUT, (unsigned char *)&c, 1);
}

int __io_getchar()
{
  unsigned char c;
  __read(_LLIO_STDIN, &c, 1);
  return (int)(c);
}

void __io_flush( void )
{
  __write(_LLIO_STDOUT, NULL, 0);
}

uint8_t append_to_buf(void)
{
  uint8_t c;
  
  HAL_UART_Receive_DMA(&Huart2, &c, 1/*, 100*/);
  rxQ[rxHead] = c;
  rxHead=(rxHead+1)%RECEIVE_QUEUE_SIZE;
  rxUsed++;
  
  if(c=='\n' || c=='\r')
    return 1;
  
  return 0;
  
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}
