/**
* @file    S2LP_Interface.c
* @author  CLAB
* @version V1.0.0
* @date    03-April-2017
* @brief   Interface function for S2LP.
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
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
#include "s2lp_interface.h"
#include "radio_appli.h"
#include "radio_gpio.h"

#include "S2LP_Timer_ex.h"
/**
* @addtogroup ST_S2LP
* @{
*/


/**
* @addtogroup S2LP_Interface
* @{
*/


/**
* @defgroup S2LP_Interface_Private_TypesDefinitions       S2LP_Interface Private Types Definitions
* @{
*/

/**
* @}
*/


/**
* @defgroup S2LP_Interface_Private_Defines                S2LP_Interface Private Defines
* @{
*/

/**
* @}
*/


/**
* @defgroup S2LP_Interface_Private_Macros                 S2LP_Interface Private Macros
* @{
*/
#define IRQ_PREEMPTION_PRIORITY         0x03
/**
* @}
*/


/**
* @defgroup S2LP_Interface_Private_Variables              S2LP_Interface Private Variables
* @{
*/

static RangeExtType xRangeExtType = RANGE_EXT_NONE;
static uint8_t s_RfModuleBand = 0;
//static uint16_t M2S_GPIO_PIN_IRQ;
extern SRadioInit xRadioInit;
/**
* @}
*/

/**
* @defgroup S2LP_Interface_Private_FunctionPrototypes     S2LP_Interface Private Function Prototypes
* @{
*/

/**
* @}
*/


/**
* @defgroup S2LP_Interface_Private_Functions              S2LP_Interface Private Functions
* @{
*/

/**
* @brief  Read the status register.
* @param  None
* @retval Status
*/
void S2LPInterfaceInit(void)
{ 
  /* Initialize the SDN pin micro side */
  RadioGpioInit(RADIO_GPIO_SDN,RADIO_MODE_GPIO_OUT);
  
  S2LPSpiInit();
  
  EepromSpiInitialization();
  
  /* S2LP ON */
  S2LPEnterShutdown();
  S2LPExitShutdown();  
  
  S2LPManagementIdentificationRFBoard();
  
  /* if the board has eeprom, we can compensate the offset calling S2LPManagementGetOffset
  (if eeprom is not present this fcn will return 0) */
  xRadioInit.lFrequencyBase = xRadioInit.lFrequencyBase + S2LPManagementGetOffset();
  
  /* if needed this will set the range extender pins */
  S2LPManagementRangeExtInit();
  
#if 0 //indar
  /* if needed this will set the EXT_REF bit of the S2-LP */
  S2LPManagementTcxoInit();
#endif  
  /* uC IRQ config */       
  RadioGpioInit(RADIO_GPIO_3,RADIO_MODE_EXTI_IN);
  // M2S_GPIO_PIN_IRQ=SdkEvalGpioGetPin(M2S_GPIO_3);
  
  /* S2LP IRQ config */
  // S2LPGpioInit(&xGpioIRQ); 
  
  /* uC IRQ enable */
  RadioGpioInterruptCmd(RADIO_GPIO_3,IRQ_PREEMPTION_PRIORITY,0,ENABLE);  
  
}


/**
* @brief  Sets the S2LP frequency band
* @param  uint8_t value: RF FREQUENCY
* @retval None
*/
void S2LP_ManagementSetBand(uint8_t value)
{
  s_RfModuleBand = value;
}


/**
* @brief  returns the S2LP frequency band
* @param  None
* @retval uint8_t value: RF FREQUENCY
*/
uint8_t S2LP_ManagementGetBand(void)
{
  return s_RfModuleBand;
}

/**
* @defgroup RANGE_EXT_MANAGEMENT_FUNCTIONS              SDK S2LP Management Range Extender Functions
* @{
*/
void S2LP_ManagementRangeExtInit(void)
{
  RangeExtType range_type = S2LP_ManagementGetRangeExtender();
#if 0  
  ////  if(range_type==RANGE_EXT_SKYWORKS_169) 
  ////  {
  ////    /* TCXO optimization power consumption */
  ////    S2LP_GeneralSetExtRef(MODE_EXT_XIN);
  ////    uint8_t tmp = 0x01; S2LP_SpiWriteRegisters(0xB6,1,&tmp);
  ////    
  ////    /* CSD control */
  ////    S2LPGpioInit(&(SGpioInit){S2LP_GPIO_0, S2LP_GPIO_MODE_DIGITAL_OUTPUT_HP, S2LP_GPIO_DIG_OUT_TX_RX_MODE});
  ////    
  ////    /* CTX/BYP control */
  ////    S2LPGpioInit(&(SGpioInit){S2LP_GPIO_1, S2LP_GPIO_MODE_DIGITAL_OUTPUT_HP, S2LP_GPIO_DIG_OUT_TX_STATE});
  ////    
  ////    /* Vcont control */
  ////    S2LPGpioInit(&(SGpioInit){S2LP_GPIO_2, S2LP_GPIO_MODE_DIGITAL_OUTPUT_HP, S2LP_GPIO_DIG_OUT_RX_STATE});
  ////  }
  //else 
#endif  
  if(range_type==RANGE_EXT_SKYWORKS_868) {   
    /* CSD control */
    S2LPGpioInit(&(SGpioInit){S2LP_GPIO_0, S2LP_GPIO_MODE_DIGITAL_OUTPUT_HP, S2LP_GPIO_DIG_OUT_TX_RX_MODE});
    
    /* CTX/BYP control */
    S2LPGpioInit(&(SGpioInit){S2LP_GPIO_1, S2LP_GPIO_MODE_DIGITAL_OUTPUT_HP, S2LP_GPIO_DIG_OUT_RX_STATE});
    
    /* Vcont control */
    S2LPGpioInit(&(SGpioInit){S2LP_GPIO_2, S2LP_GPIO_MODE_DIGITAL_OUTPUT_HP, S2LP_GPIO_DIG_OUT_TX_STATE});
  }
}

/**
* @brief  returns the S2LP range extender type
* @param  None
* @retval RangeExtType
*/
RangeExtType S2LP_ManagementGetRangeExtender(void)
{
  return xRangeExtType;
}

/**
* @brief  Sets the S2LP range extender type
* @param  RangeExtType
* @retval None
*/
void S2LP_ManagementSetRangeExtender(RangeExtType xRangeType)
{
  xRangeExtType = xRangeType;
}


/**
* @brief  this function sets the radio power
* @param  uint8_t cIndex, float fPowerdBm
* @retval None
*/
void S2LP_SetPower(uint8_t cIndex, float fPowerdBm)
{
  /* S2LP Radio set power */
  S2LPRadioSetPALeveldBm(cIndex,(int32_t)fPowerdBm);
//    S2LPRadioSetPALeveldBm(cIndex,fPowerdBm);
  S2LPRadioSetPALevelMaxIndex(cIndex);
}

/**
* @brief  this function sets the packet configuration according to the protocol used
* @param  None
* @retval None
*/
void S2LP_PacketConfig(void)
{
#if defined(USE_STack_PROTOCOL)
  
  STackProtocolInit();
  
#elif defined(USE_BASIC_PROTOCOL)
  
  BasicProtocolInit();
  
#endif
}

/**
* @brief  this function sets the payload length
* @param  uint8_t length
* @retval None
*/
void S2LP_SetPayloadlength(uint8_t length)
{
#if defined(USE_STack_PROTOCOL)
  /* Payload length config */
  S2LP_PktStackSetPayloadLength(length);
  
#elif defined(USE_BASIC_PROTOCOL)
  /* payload length config */
  S2LPPktBasicSetPayloadLength(length);
#endif
}

/**
* @brief  this function sets the destination address
* @param  uint8_t adress
* @retval None
*/
void S2LP_SetDestinationAddress(uint8_t address)
{
#if defined(USE_STack_PROTOCOL)
  /* Destination address */
  S2LP_PktStackSetDestinationAddress(address);
#elif defined(USE_BASIC_PROTOCOL)
  /* destination address */
  S2LPSetRxSourceReferenceAddress(address);
#endif
}

/**
* @brief  this function enables the Tx IRQ
* @param  None
* @retval None
*/
void S2LP_EnableTxIrq(void)
{
  /* S2LP IRQs enable */
  S2LPGpioIrqConfig(TX_DATA_SENT, S_ENABLE); 
#if defined(USE_STack_LLP)
  S2LPGpioIrqConfig(MAX_RE_TX_REACH, S_ENABLE);
#endif  
}

/**
* @brief  this function enables the Rx IRQ
* @param  None
* @retval None
*/
void S2LP_EnableRxIrq(void)
{
  /* S2LP IRQs enable */
  S2LPGpioIrqConfig(RX_DATA_READY, S_ENABLE);
  S2LPGpioIrqConfig(RX_DATA_DISC, S_ENABLE); 
  S2LPGpioIrqConfig(RX_TIMEOUT, S_ENABLE);
}


/**
* @brief  this function set the receive timeout period
* @param  None
* @retval None
*/
void S2LP_SetRxTimeout(float cRxTimeOut)
{
  if(cRxTimeOut == 0)
  {
    /* rx timeout config */
    SET_INFINITE_RX_TIMEOUT();
    S2LPTimerSetRxTimerStopCondition(ANY_ABOVE_THRESHOLD);
  }
  else
  {
    /* RX timeout config */
    S2LPTimerSetRxTimerMs(cRxTimeOut);   
    S2LP_EnableSQI();
    S2LPTimerSetRxTimerStopCondition(RSSI_AND_SQI_ABOVE_THRESHOLD);     
  }
}


/**
* @brief  this function sets the RSSI threshold
* @param  int dbmValue
* @retval None
*/
float S2LP_GetRssiTH(void)
{
  float dbmValue=0;
  dbmValue = S2LPRadioGetRssidBm();
  return dbmValue;
}

/**
* @brief  this function enables SQI check
* @param  None
* @retval None
*/
void S2LP_EnableSQI(void)
{
  /* enable SQI check */
  S2LPRadioSetPqiCheck(0x00);
  S2LPRadioSetPqiCheck(S_ENABLE);
}

/**
* @brief  this function starts the RX process
* @param  None
* @retval None
*/
void S2LP_StartRx(void)
{
  if(g_xStatus.MC_STATE==MC_STATE_RX)
  {
    S2LPCmdStrobeSabort();
  }
  /* RX command */
  S2LPCmdStrobeRx();
}

/**
* @brief  this function receives the data
* @param  None
* @retval None
*/
void S2LP_GetRxPacket(uint8_t *buffer, uint8_t *cRxData )
{
  uint8_t noofbytes = 0;
  /* when rx data ready read the number of received bytes */
  *cRxData=S2LPFifoReadNumberBytesRxFifo();
  noofbytes = *cRxData;
  /* read the RX FIFO */
  S2LPSpiReadFifo(noofbytes, buffer);
  
  S2LPCmdStrobeFlushRxFifo();
}

/**
* @brief  this function starts the TX process
* @param  None
* @retval None
*/
void S2LP_StartTx(uint8_t *buffer, uint8_t size )
{
  if(g_xStatus.MC_STATE==MC_STATE_RX)
  {
    S2LPCmdStrobeSabort();
  }
  
#ifdef CSMA_ENABLE
  
  /* Enable CSMA */
  S2LPPacketHandlerSetRxPersistentMode(S_DISABLE);
  S2LPRadioCsBlanking(S_DISABLE);
  
  S2LPCsmaInit(&xCsmaInit);
  S2LPCsma(S_ENABLE);
  S2LPRadioSetRssiThreshdBm(CSMA_RSSI_THRESHOLD);
  
#endif 
  
  /* fit the TX FIFO */
  S2LPCmdStrobeFlushTxFifo();
  
  S2LPSpiWriteFifo(size, buffer);
  
  
  /* send the TX command */
  S2LPCmdStrobeTx();
}






/**
* @}
*/

/**
* @}
*/


/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
