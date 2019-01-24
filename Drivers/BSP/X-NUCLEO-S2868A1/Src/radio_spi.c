/**
* @file    SDK_EVAL_Spi_Driver.c
* @author  High End Analog & RF BU - AMS
* @version 3.2.0
* @date    April 01, 2013
* @brief   This file provides all the low level SPI API to access to the device using a software watchdog timer to avoid stuck situation.
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
* <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
*
*/

/* Includes ------------------------------------------------------------------*/

#include "radio_spi.h"
#include "cube_hal.h"
#include "MCU_Interface.h"
#include "S2LP_Config.h"


/** @addtogroup SDK_EVAL_NUCLEO
* @{
*/


/** @addtogroup SDK_EVAL_Spi                    SDK EVAL Spi
* @brief SPI functions implementation.
* @details This file implements the SPI interface functions. Please see the MCU_Interface.h file of the device library for more details.
* @{
*/


/** @defgroup SPI_Private_Defines
* @{
*/


/** @defgroup SPI_Headers
* @{
*/
#define HEADER_WRITE_MASK     0x00 /*!< Write mask for header byte*/
#define HEADER_READ_MASK      0x01 /*!< Read mask for header byte*/
#define HEADER_ADDRESS_MASK   0x00 /*!< Address mask for header byte*/
#define HEADER_COMMAND_MASK   0x80 /*!< Command mask for header byte*/

#define LINEAR_FIFO_ADDRESS 0xFF  /*!< Linear FIFO address*/


/**
* @}
*/


/** @defgroup SDK_EVAL_Spi_Peripheral_Gpio
* @{
*/



/**
* @}
*/

/**
* @}
*/


/** @defgroup SPI_Private_Macros
* @{
*/
#define BUILT_HEADER(add_comm, w_r) (add_comm | w_r)  /*!< macro to build the header byte*/
#define WRITE_HEADER    BUILT_HEADER(HEADER_ADDRESS_MASK, HEADER_WRITE_MASK) /*!< macro to build the write header byte*/
#define READ_HEADER     BUILT_HEADER(HEADER_ADDRESS_MASK, HEADER_READ_MASK)  /*!< macro to build the read header byte*/
#define COMMAND_HEADER  BUILT_HEADER(HEADER_COMMAND_MASK, HEADER_WRITE_MASK) /*!< macro to build the command header byte*/


/**
* @}
*/



/** @defgroup SPI_Private_Variables
* @{
*/

//static SPI_HandleTypeDef SpiHandle;
SPI_HandleTypeDef SpiHandle;
/**
* @}
*/


/** @defgroup SPI_Private_FunctionPrototypes
* @{
*/


/**
* @brief  Deinitializes the SPI.
* @param  None
* @retval None
*/
void RadioSpiDeinit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  
  /* Enable SPI periph and SCLK, MOSI, MISO and CS GPIO clocks */
  RADIO_SPI_CLK_ENABLE();
  RADIO_SPI_CS_CLK_ENABLE();
  RADIO_SPI_SCLK_CLK_ENABLE();
  RADIO_SPI_MISO_CLK_ENABLE();
  RADIO_SPI_MOSI_CLK_ENABLE();
  
  
  /* Configure the AF for MOSI, MISO and SCLK GPIO pins*/
  GPIO_InitStructure.Pin       = RADIO_SPI_SCK_PIN;
  GPIO_InitStructure.Mode      = GPIO_MODE_OUTPUT_PP;//GPIO_MODE_INPUT;
  GPIO_InitStructure.Pull      = GPIO_NOPULL;
  GPIO_InitStructure.Speed     = GPIO_SPEED_HIGH;
  
  HAL_GPIO_Init(RADIO_SPI_SCK_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.Pin = RADIO_SPI_MISO_PIN;
  
  HAL_GPIO_Init(RADIO_SPI_MISO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.Pin = RADIO_SPI_MOSI_PIN;
  
  HAL_GPIO_Init(RADIO_SPI_MOSI_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.Pin = RADIO_SPI_CS_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
  HAL_GPIO_Init(RADIO_SPI_CS_PORT, &GPIO_InitStructure);
  HAL_GPIO_WritePin(RADIO_SPI_MOSI_PORT,RADIO_SPI_MOSI_PIN,GPIO_PIN_SET);
  HAL_GPIO_WritePin(RADIO_SPI_MISO_PORT,RADIO_SPI_MISO_PIN,GPIO_PIN_SET);
  
  /* Configure SPI peripheral */
  if(HAL_SPI_GetState(&SpiHandle) == HAL_SPI_STATE_RESET)
  {  
    /* Set the SPI parameters */
    SpiHandle.Instance               = RADIO_SPI;
    SpiHandle.Init.Mode              = SPI_MODE_MASTER;
    SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    
    SpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
    SpiHandle.Init.CLKPhase          = SPI_PHASE_1EDGE;
    SpiHandle.Init.CLKPolarity       = SPI_POLARITY_LOW;
    SpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLED;
    SpiHandle.Init.CRCPolynomial     = 7;
    SpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
    SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    SpiHandle.Init.NSS               = SPI_NSS_SOFT;
    SpiHandle.Init.TIMode            = SPI_TIMODE_DISABLED;
    
    if(HAL_SPI_DeInit(&SpiHandle) != HAL_OK) {
      return;
    } 
    __HAL_SPI_DISABLE(&SpiHandle);    
  }
  
}


/**
* @brief  Write single or multiple registers.
* @param  baudrate_prescaler: baudrate prescaler to be used
*               Must be one of the @ref SPI_BaudRate_Prescaler .
* @retval None
*/
void RadioSpiSetBaudrate(uint32_t baudrate_prescaler)
{
  __HAL_SPI_DISABLE(&SpiHandle);    
  
  SpiHandle.Init.BaudRatePrescaler = baudrate_prescaler;
  
  if(HAL_SPI_Init(&SpiHandle) != HAL_OK) {
    return;
  } 
  __HAL_SPI_ENABLE(&SpiHandle);    
  
}

/**
* @brief  Initializes the SPI.
* @param  None
* @retval None
*/
void RadioSpiInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  /* Enable SPI periph and SCLK, MOSI, MISO and CS GPIO clocks */
  RADIO_SPI_CLK_ENABLE();
  RADIO_SPI_CS_CLK_ENABLE();
  RADIO_SPI_SCLK_CLK_ENABLE();
  RADIO_SPI_MISO_CLK_ENABLE();
  RADIO_SPI_MOSI_CLK_ENABLE();
  
  
  /* Configure the AF for MOSI, MISO and SCLK GPIO pins*/
  GPIO_InitStructure.Pin       = RADIO_SPI_SCK_PIN;
  GPIO_InitStructure.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Pull      = GPIO_PULLUP;
  GPIO_InitStructure.Speed     = GPIO_SPEED_HIGH;
  GPIO_InitStructure.Alternate = RADIO_SPI_SCK_AF;
  HAL_GPIO_Init(RADIO_SPI_SCK_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.Pin = RADIO_SPI_MISO_PIN;
  GPIO_InitStructure.Alternate = RADIO_SPI_MISO_AF;
  HAL_GPIO_Init(RADIO_SPI_MISO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.Pin = RADIO_SPI_MOSI_PIN;
  GPIO_InitStructure.Alternate = RADIO_SPI_MOSI_AF;
  HAL_GPIO_Init(RADIO_SPI_MOSI_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.Pin = RADIO_SPI_CS_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(RADIO_SPI_CS_PORT, &GPIO_InitStructure);
  
  
  /* Configure SPI peripheral */
  if(HAL_SPI_GetState(&SpiHandle) == HAL_SPI_STATE_RESET)
  {  
    /* Set the SPI parameters */
    SpiHandle.Instance               = RADIO_SPI;
    SpiHandle.Init.Mode              = SPI_MODE_MASTER;
    SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    
    SpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
    SpiHandle.Init.CLKPhase          = SPI_PHASE_1EDGE;
    SpiHandle.Init.CLKPolarity       = SPI_POLARITY_LOW;
    SpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLED;
    SpiHandle.Init.CRCPolynomial     = 7;
    SpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
    SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    SpiHandle.Init.NSS               = SPI_NSS_SOFT;
    SpiHandle.Init.TIMode            = SPI_TIMODE_DISABLED;
    
    if(HAL_SPI_Init(&SpiHandle) != HAL_OK) {
      while(1);
    } 
    __HAL_SPI_ENABLE(&SpiHandle);    
  }
  
  RadioSpiCSHigh();
}


/**
* @brief  Write single or multiple registers.
* @param  cRegAddress: base register's address to be write
* @param  cNbBytes: number of registers and bytes to be write
* @param  pcBuffer: pointer to the buffer of values have to be written into registers
* @retval Device status
*/
StatusBytes RadioSpiWriteRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer)
{
  uint8_t tx_buff[2]={WRITE_HEADER,cRegAddress};
  uint8_t rx_buff[255];
  StatusBytes status;
  
  SPI_ENTER_CRITICAL();
  
  /* Puts the SPI chip select low to start the transaction */
  RadioSpiCSLow();
  
  HAL_SPI_TransmitReceive(&SpiHandle, tx_buff, rx_buff, 2, 1000);
  HAL_SPI_TransmitReceive(&SpiHandle, pcBuffer, &rx_buff[2], cNbBytes, 1000);
  
  /* Puts the SPI chip select high to end the transaction */
  RadioSpiCSHigh();
  
  SPI_EXIT_CRITICAL();
  
  ((uint8_t*)&status)[1]=rx_buff[0];
  ((uint8_t*)&status)[0]=rx_buff[1];
  
  return status;
  
}

/**
* @brief  Read single or multiple registers.
* @param  cRegAddress: base register's address to be read
* @param  cNbBytes: number of registers and bytes to be read
* @param  pcBuffer: pointer to the buffer of registers' values read
* @retval Device status
*/
StatusBytes RadioSpiReadRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer)
{
  uint8_t tx_buff[255]={READ_HEADER,cRegAddress};
  uint8_t rx_buff[2];
  StatusBytes status;
  
  SPI_ENTER_CRITICAL();
  RadioSpiCSLow();
  HAL_SPI_TransmitReceive(&SpiHandle, tx_buff, rx_buff, 2, 1000);
  HAL_SPI_TransmitReceive(&SpiHandle, tx_buff, pcBuffer, cNbBytes, 1000);
  RadioSpiCSHigh();
  SPI_EXIT_CRITICAL();
  
  ((uint8_t*)&status)[1]=rx_buff[0];
  ((uint8_t*)&status)[0]=rx_buff[1];  
  
  return status;
}

/**
* @brief  Send a command
* @param  cCommandCode: command code to be sent
* @retval Device status
*/
StatusBytes RadioSpiCommandStrobes(uint8_t cCommandCode)
{
  uint8_t tx_buff[2]={COMMAND_HEADER,cCommandCode};
  uint8_t rx_buff[2];
  StatusBytes status;
  
  SPI_ENTER_CRITICAL();
  RadioSpiCSLow();
  HAL_SPI_TransmitReceive(&SpiHandle, tx_buff, rx_buff, 2, 1000);
  RadioSpiCSHigh();
  SPI_EXIT_CRITICAL();
  
  ((uint8_t*)&status)[1]=rx_buff[0];
  ((uint8_t*)&status)[0]=rx_buff[1];
  
  return status;
}


/**
* @brief  Write data into TX FIFO.
* @param  cNbBytes: number of bytes to be written into TX FIFO
* @param  pcBuffer: pointer to data to write
* @retval Device status
*/
StatusBytes RadioSpiWriteFifo(uint8_t cNbBytes, uint8_t* pcBuffer)
{
  uint8_t tx_buff[2]={WRITE_HEADER,LINEAR_FIFO_ADDRESS};
  uint8_t rx_buff[130];
  StatusBytes status;
  
  SPI_ENTER_CRITICAL();
  RadioSpiCSLow();
  HAL_SPI_TransmitReceive(&SpiHandle, tx_buff, rx_buff, 2, 1000);
  HAL_SPI_TransmitReceive(&SpiHandle, pcBuffer, &rx_buff[2], cNbBytes, 1000);
  RadioSpiCSHigh();
  SPI_EXIT_CRITICAL();
  
  ((uint8_t*)&status)[1]=rx_buff[0];
  ((uint8_t*)&status)[0]=rx_buff[1];
  
  return status;
}


/**
* @brief  Read data from RX FIFO.
* @param  cNbBytes: number of bytes to read from RX FIFO
* @param  pcBuffer: pointer to data read from RX FIFO
* @retval Device status
*/
StatusBytes RadioSpiReadFifo(uint8_t cNbBytes, uint8_t* pcBuffer)
{
  uint8_t tx_buff[130]={READ_HEADER,LINEAR_FIFO_ADDRESS};
  uint8_t rx_buff[2];
  StatusBytes status;
  
  SPI_ENTER_CRITICAL();
  RadioSpiCSLow();
  HAL_SPI_TransmitReceive(&SpiHandle, tx_buff, rx_buff, 2, 1000);
  HAL_SPI_TransmitReceive(&SpiHandle, tx_buff, pcBuffer, cNbBytes, 1000);
  RadioSpiCSHigh();
  SPI_EXIT_CRITICAL();
  
  ((uint8_t*)&status)[1]=rx_buff[0];
  ((uint8_t*)&status)[0]=rx_buff[1];
  
  
  return status;
  
}

SPI_HandleTypeDef* RadioSpiGetStruct(void)
{
  return &SpiHandle;
}

/**
* @}
*/


/**
* @}
*/


/**
* @}
*/



/******************* (C) COPYRIGHT 2013 STMicroelectronics *****END OF FILE****/
