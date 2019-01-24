/**
* @file    user_config.h
* @author  System Lab - NOIDA
* @version V1.0.0
* @date    14-Mar-2018
* @brief  This file contains the user configurable MACROs  
* @details
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USER_CONFIG_H
#define __USER_CONFIG_H
/* Includes ------------------------------------------------------------------*/

#include "string.h"

/********************WMBUS Sub-Mode Configuration******************************/
#define DEVICE_WMBUS_MODE               S2_MODE                               
#define DEVICE_WMBUS_CHANNEL            CHANNEL_5
#define WMBUS_FRAME_FORMAT              FRAME_FORMAT_A
#define TX_OUTPUT_POWER                 (10)/*Between [-3.0 ,+27.0] if the RF 
                                               board has an external PA. 
                                               Otherwise between[-30.0 ,+13.0]*/

/***********************************User Settings******************************/
#define ENABLE_CONTINUOUS_TX
              
#ifdef ENABLE_CONTINUOUS_TX
#define CONTINUOUS_TX_TIME_INTERVAL     4000  /*In MiliSecond*/
#endif                                                

#define UART_BAUD_RATE                 115200



/**************************Generic User defines********************************/
#define ENCR_MODE_0             0x00 /*No Encryption*/
#define ENCR_MODE_1             0x01 /*Encryption mode not Supported*/
#define ENCR_MODE_2             0x02 /*Encryption mode not Supported*/
#define ENCR_MODE_3             0x03 /*Encryption mode not Supported*/
#define ENCR_MODE_4             0x04 /*Encryption mode not Supported*/
#define ENCR_MODE_5             0x05 /*Encryption mode Supported : 
                                        AES encryption with CBC; 
                                        initialisation vector is not zero*/
#define ENCR_MODE_6             0x06 /*Encryption mode not Supported*/
#define ENCR_MODE_7             0x07 /*Encryption mode not Supported*/
#define ENCR_MODE_15            0x0F /*Encryption mode not Supported*/

#define ENCKEY_LEN              16   /*16 bytes Encryption Key is used*/

#define ENCR_BLOCK_SIZE         16   /*Encrypted block size*/

#define METER_DB_INDEX          0x01 /*Index to get the device selfinfo
                                        from the meter database Index 0 
                                        is reserved for Conc*/
#define INIT_VECTOR_SIZE        16
#define MAX_DATA_LEN            255

                                          
#ifdef DEVICE_TYPE_METER

//#define ENCRPTION_ENABLE

#ifdef  ENCRPTION_ENABLE
  #define ENC_CONFIG_WORD   0x0551                                        
#endif

#ifndef  ENCRPTION_ENABLE    
  #define ENC_CONFIG_WORD    0x00                                       
#endif                                        
     
#endif


#ifndef DEVICE_TYPE_METER
 #define ENC_CONFIG_WORD   0x0551
#endif

/*************************Device Type Identification***************************/
                                  
                                                                             
#define OTHER_DEVICE                    0x00
#define OIL_METER                       0x01
#define ELECTRICITY_METER               0x02
#define GAS_METER                       0x03
#define HEAT_METER                      0x04
#define STEAM_METER                     0x05
#define WARM_WATER_METER                0x06
#define WATER_METER                     0x07
#define HEAT_COST_ALLOCATOR             0x08
#define COMPRESSED_AIR                  0x09
#define UNKNOWN_DEVICE                  0x0F
#define HOT_WATER_METER                 0x15
#define COLD_WATER_METER                0x16
#define PRESSURE_METER                  0x18
#define AD_CONVERTER                    0x19
#define SMOKE_DETECTOR                  0x1A
#define ROOM_SENSOR                     0x1B
#define GAS_DETECTOR                    0x1C
#define BREAKER                         0x20
#define VALVE                           0x21
#define CUSTOMER_UNIT                   0x25
#define WASTE_WATER_METER               0x28
#define GARBAGE                         0x29
#define COMMUNICATION_CONTROLLER        0x31

//#define STORE_NVM_DEVINFO  /*Uncomment to Initialize DeviceInfo & MeterDB*/
                             /*(Local meter database of concentrator)in EEPROM*/

//#define ADDRESS_FILTERING_ENABLE
                                                 
#define RF_ADAPTER_SER_NUM              0x55667788
#define METER_SERIAL_NUM                0x11456317
#define OTHER_SERIAL_NUM                0x11223344
#define ACK_TIMEOUT                     30000
                                        
                                          
#define CMD_BUF_SIZE                128                                         
                                          
/*********************************************************************/
/***********         Hardware PIN Configuration              *********/                                          
  
#define SHDN_PORT                   GPIOA
#define SHDN_PIN                    GPIO_Pin_0
#define SHDN_EXTI_PORT              EXTI_PortSourceGPIOA
#define SHDN_EXTI_PIN               EXTI_PinSource0
#define SHDN_EXTI_LINE              EXTI_Line0
#define RF_SDN_PORT                 GPIOA
#define RF_SDN_PIN                  GPIO_Pin_1


#define SWITCH_PORT                   GPIOA
#define SWITCH_PIN                    GPIO_Pin_10
#define SWITCH_EXTI_PORT              EXTI_PortSourceGPIOA
#define SWITCH_EXTI_PIN               EXTI_PinSource10
#define SWITCH_EXTI_LINE              EXTI_Line10
#define SWITCH_EXTI_LINE_IRQ          EXTI15_10_IRQn

     
#define RF_SPI_SCLK_PORT            GPIOA
#define RF_SPI_SCLK_PIN             GPIO_Pin_5
#define RF_SPI_SCLK_SOURCE          GPIO_PinSource5
#define RF_SPI_MISO_PORT            GPIOA
#define RF_SPI_MISO_PIN             GPIO_Pin_6
#define RF_SPI_MISO_SOURCE          GPIO_PinSource6
#define RF_SPI_MOSI_PORT            GPIOA
#define RF_SPI_MOSI_PIN             GPIO_Pin_7
#define RF_SPI_MOSI_SOURCE          GPIO_PinSource7

#define RF_CS_PORT                  GPIOA
#define RF_CS_PIN                   GPIO_Pin_4

#define UART_TX_PORT                GPIOA
#define UART_TX_PIN                 GPIO_Pin_9
#define UART_RX_PORT                GPIOA
#define UART_RX_PIN                 GPIO_Pin_10
                              
                     
#define GPIO0_PORT                  GPIOB
#define GPIO0_PIN                   GPIO_Pin_15
#define GPIO1_PORT                  GPIOB
#define GPIO1_PIN                   GPIO_Pin_14
#define GPIO2_PORT                  GPIOB
#define GPIO2_PIN                   GPIO_Pin_6
#define GPIO3_PORT                  GPIOB
#define GPIO3_PIN                   GPIO_Pin_7
                              
#define MODE0_PORT                  GPIOB
#define MODE0_PIN                   GPIO_Pin_13
#define MODE0_EXTI_PORT             EXTI_PortSourceGPIOB
#define MODE0_EXTI_PIN              EXTI_PinSource13
#define MODE0_EXTI_LINE             EXTI_Line13
#define MODE1_PORT                  GPIOB
#define MODE1_PIN                   GPIO_Pin_12

#define RF_GPIO0_PORT               GPIOC
#define RF_GPIO0_PIN                GPIO_Pin_4
#define RF_GPIO1_PORT               GPIOB
#define RF_GPIO1_PIN                GPIO_Pin_11
#define RF_GPIO2_PORT               GPIOC
#define RF_GPIO2_PIN                GPIO_Pin_5
#define RF_GPIO3_PORT               GPIOH
#define RF_GPIO3_PIN                GPIO_Pin_0
              
/*********************************************************************/

                                          
/**
* @}
*/
#endif /* __USER_CONFIG_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
