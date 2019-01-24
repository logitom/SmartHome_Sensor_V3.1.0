/**
* @file    user_appli.c
* @author  System Lab - NOIDA
* @version V1.0.0
* @date    10-Mar-2016
* @brief   This file is application layer file used by user
* @details
*
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
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
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
#include <string.h>
#include "user_appli.h"
#include "wmbus_phy.h"
#include "wmbus_link.h"
#include "wmbus_linkcommand.h"
#include "wmbus_tim.h"
#include "aes.h"
#include "uart.h"
#include "fifo.h"
#include "command.h"
#include "user_config.h"
#include "wmbus_link.h"

#ifdef USE_STM32F4XX_NUCLEO
#include "stm32f4xx_hal_gpio.h"
#endif

#ifdef USE_STM32L1XX_NUCLEO
#include "stm32l1xx_hal_gpio.h"
#endif

#ifdef USE_STM32L0XX_NUCLEO
#include "stm32l0xx_hal_gpio.h"
#endif





/**
* @addtogroup Application Application
* @{
*/

/**
* @addtogroup User Application    User Application
* @{
*/

/**
* @defgroup user_appli_Private_TypesDefinitions    User Application Private TypesDefinitions
* @{
*/

/**
*@}
*/


/**
* @defgroup user_appli_Private_Defines            User Application Private Defines
* @{
*/

/**
*@}
*/


/**
* @defgroup user_appli_Private_Macros          User Application Private Macros
* @{
*/

// General constant definitions
#define RF_TX_BUF_SIZE              300
#define RF_RX_BUF_SIZE              300

#define LF                          0x0a    // Linefeed character
#define CR                          0x0d    // Carriage return character

/**
*@}
*/

/**
* @defgroup user_appli_Private_Variables      User Application Private Variables
* @{
*/


__IO uint32_t DOWNStatus = 0x00, UPStatus = 0x00, SELStatus = 0x00,\
                     RIGHTStatus=0x00, LEFTStatus=0x00, KEYStatus=0x00;  

uint8_t AES_InitializationVector_Tx[INIT_VECTOR_SIZE] ={0};/*Tx Initialization vector*/
uint8_t AES_InitializationVector_Rx[INIT_VECTOR_SIZE] = {0};/*Rx Initialization vector*/

/* Following variables are used for implementing WMBus usage scenarios*/
uint8_t AlarmReqFlag = 0x00;
uint8_t DataReqFlag = 0x01;
uint8_t ResetLinkFlag = 0x00;
uint8_t SendData2Flag = 0x00;
uint8_t SendDataFlag = 0x00;
uint8_t CurrDataReqFlag = 0x00;
uint8_t CurrAlarmReqFlag = 0x00;
uint8_t CurrSendData2Flag = 0x00;
uint8_t ConfirmInstallFlag = 0;
/*Following Variable are used to generate random access number*/
uint8_t AccessNumber =  0x00;
uint8_t EllAccessNum = 0x00;
uint8_t UserDataBlock[20];
uint8_t PayloadSize = 0x08;/*Payload Size: defined by user*/
uint8_t PayloadBuffer[MAX_DATA_LEN] = {0};/*Payload Buffer: defined by user*/
//static __IO uint8_t CmdResponseFlow[8];
const uint8_t PktPayloadSize=16;
app_mode_t g_app_mode = APP_MODE_OPERATING;
uint8_t   g_cmd_buf_len = 0;
uint8_t    g_cmd_received = 0;
uint8_t    g_cmd_buf[CMD_BUF_SIZE];
OtherCmdFlag NxtCommandFlag;
OtherCmdFlag CurrCommandFlag;
uint8_t    g_mode = 1;
uint8_t    g_mode_prev = 1;
uint8_t    g_shutdown = 0;
uint8_t Flag  = 0;
uint8_t Buff[10] =  "HELLO";
volatile FlagStatus g_tx_done_flag = RESET;
volatile FlagStatus g_rx_done_flag = RESET;
static volatile uint8_t  g_rf_tx_timeout = 0;
//extern uint32_t g_escape_count;
CommandMode_TypedefEnum CommandMode = FIFO_INIT;

uint64_t GasMeterVol = 0;
//extern uint8_t VolumeIncFlag;

#pragma pack(4)
/*********************Default Payload Buffer: defined by user******************/
const uint8_t DefaultPayloadBuffer[MAX_DATA_LEN] = { 0x01,0x02,0x03,0x04,0x05,\
                                  0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0e,0x0f,\
                                  0xff,0x02,0x02,0x03,0x04,0x05,0x06,0x07,0x08,\
                                  0x09,0x0a,0x0b,0x0c,0x0e,0x0f,0xff,0x03,0x02,\
                                  0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,\
                                  0x0c,0x0e,0x0f,0xff,0x04,0x02,0x03,0x04,0x05,\
                                  0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0e,0x0f,\
                                  0xff,0x05,0x02,0x03,0x04,0x05,0x06,0x07,0x08,\
                                  0x09,0x0a,0x0b,0x0c,0x0e,0x0f,0xff,0x06,0x02,\
                                  0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,\
                                  0x0c,0x0e,0x0f,0xff,0x07,0x02,0x03,0x04,0x05,\
                                  0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0e,0x0f,\
                                  0xff,0x08,0x02,0x03,0x04,0x05,0x06,0x07,0x08,\
                                  0x09,0x0a,0x0b,0x0c,0x0e,0x0f,0xff,0x09,0x02,\
                                  0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,\
                                  0x0c,0x0e,0x0f,0xff,0x10,0x02,0x03,0x04,0x05,\
                                  0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0e,0x0f,\
                                  0xff,0x11,0x02,0x03,0x04,0x05,0x06,0x07,0x08,\
                                  0x09,0x0a,0x0b,0x0c,0x0e,0x0f,0xff,0x12,0x02,\
                                  0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,\
                                  0x0c,0x0e,0x0f,0xff,0x13,0x02,0x03,0x04,0x05,\
                                  0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0e,0x0f,\
                                  0xff,0x14,0x02,0x03,0x04,0x05,0x06,0x07,0x08,\
                                  0x09,0x0a,0x0b,0x0c,0x0e,0x0f,0xff,0x15,0x02,\
                                  0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,\
                                  0x0c,0x0e,0x0f,0xff,0x16,0x02,0x03,0x04,0x05,\
                                  0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0e,0x0f,\
                                  0xff,0x17,0x02,0x03,0x04,0x05,0x06,0x07,0x08,\
                                  0x09,0x0a,0x0b,0x0c,0x0e,0x0f,0xff};

#pragma pack(4)
/****************************Default Meter Database****************************/
const MeterDataBase_t DefaultMeterDatabase=
{
  .DataPresentFlag = 0,
  .DeviceCount = 0x0000000A, 
  .MeterList = { 
    /************* METER 1 Data*************/
    {
      {0x8D,0x4E},
      {0x12,0x45,0x63,0x12},0x10,0x02,
      {0xc2,0x86,0x69,0x6d,0x88,0x7c,0x9a,0xa0,
      0x61,0x1b,0xbb,0x3e,0x20,0x25,0xa4,0x5a}
    },
    
    /************* METER 2 Data*************/      
    {
      {0x8D,0x4E},
      {0x11,0x45,0x63,0x17},0x10,0x02,
      {0x03,0x02,0x01,0x00,0x07,0x06,0x05,0x04,
      0x0b,0x0a,0x09,0x08,0x0f,0x0e,0x0d,0x0c}
    },
    
    /************* METER 3 Data*************/    
    {
      {0x8D,0x4E},
      {0x11,0x45,0x63,0x18},0x10,0x02,
      {0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x10,0x11,
      0x12,0x13,0x014,0x15,0x16,0x17,0x18,0x19}
    },
    
    /************* METER 4 Data*************/
    {
      {0x8D,0x4E},
      {0x11,0x45,0x63,0x19},0x10,0x02,
      {0x0D,0x0B,0x0C,0x0D,0x0A,0x0F,0x13,0x11,
      0x12,0x13,0x01F,0x15,0x16,0x17,0x18,0x19}
    },
    
    /************* METER 5 Data*************/   
    {
      {0x8D,0x4E},
      {0x11,0x45,0x63,0x1A},0x10,0x02,
      {0x1A,0x1B,0x1C,0x1D,0x1E,0x1F,0x10,0x11,
      0x12,0x13,0x014,0x15,0x16,0x17,0x18,0x19}
    },
    
    /************* METER 6 Data*************/
    {
      {0x8D,0x4E},
      {0x11,0x45,0x63,0x1B},0x10,0x02,
      {0x2A,0x2B,0x2C,0x2D,0x2E,0x2F,0x20,0x11,
      0x12,0x13,0x014,0x15,0x16,0x17,0x18,0x19}
    },
    
    /************* METER 7 Data*************/
    {
      {0x8D,0x4E},
      {0x11,0x45,0x63,0x1C},0x10,0x02,
      {0x3A,0x3B,0x3C,0x3D,0x3E,0x3F,0x30,0x11,
      0x12,0x13,0x014,0x15,0x16,0x17,0x18,0x19}
    },
    
    /************* METER 8 Data*************/
    {
      {0x8D,0x4E},
      {0x11,0x45,0x63,0x1D},0x10,0x02,
      {0x4A,0x4B,0x4C,0x4D,0x4E,0x4F,0x40,0x11,
      0x12,0x13,0x014,0x15,0x16,0x17,0x18,0x19}
    },
    
    /************* METER 9 Data*************/   
    {
      {0x8D,0x4E},
      {0x11,0x45,0x63,0x1E},0x10,0x03,
      {0x5A,0x5B,0x5C,0x5D,0x5E,0x5F,0x50,0x11,
      0x12,0x13,0x014,0x15,0x16,0x17,0x18,0x19}
    },
    
    /************* METER 10 Data*************/
    {
      {0x8D,0x4E},
      {0x11,0x45,0x63,0x1F},0x10,0x04,
      {0x6A,0x6B,0x6C,0x6D,0x6E,0x6F,0x60,0x11,
      0x12,0x13,0x014,0x15,0x16,0x17,0x18,0x19}
    }   
  }
};
#pragma pack(4)
MeterDataBase_t MeterDatabase;  /*List of device infos of meters & Count in DB*/

#pragma pack(4)
DeviceInfo_t NewDevInfo =     {
      {0x8D,0x4E},
      {0x11,0x45,0x63,0x1F},0x10,0x04,
       {0x01,0x01,0x02,0x03,0x04,0x05,0x06,0x07,
      0x12,0x13,0x014,0x15,0x16,0x17,0x18,0x19}
    }  ;    /*Device info like Mfr, ID, Ver, Type, Enc Key*/

#pragma pack(4)
DeviceInfo_t ConcDevInfo = {
  
  {0x8D,0x4E},
  {0x12,0x45,0x63,0x12},0x10,0x02,
  {0xc2,0x86,0x69,0x6d,0x88,0x7c,0x9a,0xa0,
  0x61,0x1b,0xbb,0x3e,0x20,0x25,0xa4,0x5a}
};


/***************Application Attributes****************/
#pragma pack(4)
WMBusAppliAttributes  AppliAttr=  
{ .ELL_Len = 0,
  .SerialNum = 0x00000000,
  .CIField = CI_APPLI_RESET_NONE,
  .HeaderLength = NO_HEADER,
  .DevManufrID = {0x00,0x00},
  .DevID = {0x00,0x00,0x00,0x00},
  .DevVersion = 0x00,
  .MeterDevType = 0x00,
  .AccessNumber = 0x00,
  .Status = 0x00,
  .CONFIG_WORD ={0x0000},
  .EncKey = {0x00},
  .HardwareRev = 0x10,
  .FirmwareRev = 0x02000000, 
  .AckTimeout = 0x00,
  .FrequentAccessCycle = 0,
  .ELLType = ELL_0,
  .ELLAccessNum = 0x00
};


CfgParam_Typedef CurrCfg;

#ifdef DEVICE_TYPE_METER
#pragma pack(4)
CfgParam_Typedef CurrCfg = 
{
  .Flag = 1, 
  .ModuleFunction = 0,
  .DevType = METER,
  .Channel = DEVICE_WMBUS_CHANNEL,
  .FrameFormat = WMBUS_FRAME_FORMAT,
  .WorkingMode = DEVICE_WMBUS_MODE,
  .UartBaudRate = UART_BAUD_RATE,
  .EncrModeVar = 0xF0FF,
  .DevInfo.MfrID = {0x8D,0x4E},
  .DevInfo.IDNum = {0x11,0x45,0x63,0x1F},
  .DevInfo.Version = 0x10,
  .DevInfo.MeterType = 0x04,
  .DevInfo.EncrKey = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                      0x12,0x13,0x014,0x15,0x16,0x17,0x18,0x19}
                     
};

#endif


#ifndef DEVICE_TYPE_METER
#pragma pack(4)
CfgParam_Typedef CurrCfg = 
{
  .Flag = 1, 
  .ModuleFunction = 0,
  .DevType = OTHER,
  .Channel = DEVICE_WMBUS_CHANNEL,
  .FrameFormat = WMBUS_FRAME_FORMAT,
  .WorkingMode = DEVICE_WMBUS_MODE,
  .UartBaudRate = UART_BAUD_RATE,
  .EncrModeVar = 0xF0FF,
  .DevInfo.MfrID = {0x8D,0x4E},
  .DevInfo.IDNum = {0x11,0x45,0x63,0x1F},
  .DevInfo.Version = 0x10,
  .DevInfo.MeterType = 0x04,
  .DevInfo.EncrKey = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                      0x12,0x13,0x014,0x15,0x16,0x17,0x18,0x19}
                     
};
#endif



/**
*@}
*/

/**
* @defgroup user_appli_Private_FunctionPrototypes     User Application Private FunctionPrototypes
* @{
*/

uint8_t GetDBMeterIndex(Frame_t  *rxFrame);

/**
*@}
*/


/**
* @defgroup user_appli_Private_Functions         User Application Private Functions
* @{
*/


/**
* @brief  This function will set the flags in order to send next command
* @param  OtherCmdFlag cmdflag : Command Flag
* @retval None.
*/
void SetNextCmdFlagOther(OtherCmdFlag nextCmdflag)
{
  switch(nextCmdflag)
  {
  case DATA_FLAG:
    DataReqFlag = 0x01;
    AlarmReqFlag = 0x00;
    SendDataFlag = 0x00;
    SendData2Flag = 0x00;
    ResetLinkFlag = 0x00;
    break;
  case ALARM_FLAG:
    DataReqFlag = 0x00;
    AlarmReqFlag = 0x01;
    SendDataFlag = 0x00;
    SendData2Flag = 0x00;
    ResetLinkFlag = 0x00;
    break;
  case SND_UD_FLAG:
    DataReqFlag = 0x00;
    AlarmReqFlag = 0x00;
    SendDataFlag = 0x01;
    SendData2Flag = 0x00;
    ResetLinkFlag = 0x00;
    break;
  case SND_UD2_FLAG:
    DataReqFlag = 0x00;
    AlarmReqFlag = 0x00;
    SendDataFlag = 0x00;
    SendData2Flag = 0x01;
    ResetLinkFlag = 0x00;
    break;
  case RESET_LINK_FLAG:
    DataReqFlag = 0x00;
    AlarmReqFlag = 0x00;
    SendDataFlag = 0x00;
    SendData2Flag = 0x00;
    ResetLinkFlag = 0x01;
    break;
  case RESET_ALL_FLAG:
    DataReqFlag = 0x00;
    AlarmReqFlag = 0x00;
    SendDataFlag = 0x00;
    SendData2Flag = 0x00;
    ResetLinkFlag = 0x00;
    break;
  default :
    DataReqFlag = 0x01;  
    AlarmReqFlag = 0x00;
    SendDataFlag = 0x00;
    SendData2Flag = 0x00;
    ResetLinkFlag = 0x00;
  }
}

/**
* @brief  This function will set the current command flag
* @param  OtherCmdFlag currCmdflag : Current command executed
* @retval None.
*/
void SetCurrCmdFlagOther(OtherCmdFlag currCmdflag)
{
  switch(currCmdflag)
  {
  case DATA_FLAG:
    CurrDataReqFlag = 0x01;
    CurrAlarmReqFlag = 0x00;
    CurrSendData2Flag = 0x00;
    break;
  case ALARM_FLAG:
    CurrDataReqFlag = 0x00;
    CurrAlarmReqFlag = 0x01;
    CurrSendData2Flag = 0x00;
    break;
  case SND_UD2_FLAG:
    CurrDataReqFlag = 0x00;
    CurrAlarmReqFlag = 0x00;
    CurrSendData2Flag = 0x01;
    break;
  default :
    break;
  }
}


/**
* @brief  This function will write Meter Database in Meter EEPROM
*         in Meter Database. 
* @param  None.
* @retval None.
*/
void WriteDeviceSelfinfoToEEPROM(void)
{
//  /*Write Device self info*/
//  WMBus_WriteDeviceInfoToEEPROM((uint8_t *)&CurrCfg.DevInfo,
//    sizeof(DeviceInfo_t), EEPROM_DEVINFO_START); 

}


/**
* @brief  This function will write New meter database in Concentrator EEPROM
* @param  None.
* @retval None.
*/
void WriteNewMeterDatabaseToEEPROM(DeviceInfo_t devInfo,uint16_t meterNum)
{
//  /*Write meter database*/
//   WMBus_WriteDeviceInfoToEEPROM((uint8_t *)&MeterDatabase,
//     sizeof(MeterDataBase_t), EEPROM_METERDB_START);

}


/**
* @brief  This function will write New meter database in Concentrator EEPROM
* @param  None.
* @retval None.
*/
void WriteConcDatabaseToEEPROM(void)
{
//  /*Write meter database*/
//   WMBus_WriteDeviceInfoToEEPROM((uint8_t *)&MeterDatabase,
//     sizeof(MeterDataBase_t), EEPROM_METERDB_START);
}



/**
* @brief  This function will write All database in Concentrator EEPROM
* @param  None.
* @retval None.
*/
void WriteDataBaseToEEPROM(void)
{
//  WMBus_WriteDeviceInfoToEEPROM((uint8_t *)&DefaultMeterDatabase,
//                                sizeof(DefaultMeterDatabase),
//                                DATABASE_START_ADDR);
    
}

/**
* @brief  This function will Erase All database in Concentrator EEPROM
* @param  None.
* @retval None.
*/
void EraseDataBaseFromEEPROM(void)
{
//  WMBus_EraseMeterDatabaseFromEEPROM1(DATABASE_START_ADDR,
//                                      sizeof(DefaultMeterDatabase));
   
}


/**
* @brief  This function will Read Concentrator EEPROM and put the data in the
*                      MeterDatabase structure.
* @param  None.
* @retval None.
*/
void UpdateMeterDatabaseFromEEPROM(void)
{
//  /*Read the meter database*/
//  WMBus_ReadDeviceInfoFromEEPROM((uint8_t*)&MeterDatabase,
//    sizeof(MeterDataBase_t), EEPROM_METERDB_START); 
}

/**
* @brief  This function writes the current configuration data to the EEPROM.
*         The purpose is to Load configuration even after Reset. 
* @param  None.
* @retval None.
*/
void WriteCurrentCfgToEEPROM(void)                                              
{
  //Write the parameters to be saved in the EEPROM 
  
//   /*Write The Parameter in the Database*/
//   WMBus_WriteDeviceInfoToEEPROM((uint8_t *)&CurrCfg,
//                                  sizeof(CurrCfg),
//                                  EEPROM_CURRENT_CFG_SADDRESS);
}


/**
* @brief  This function Reads the current configuration data from EEPROM Database.
*         The purpose is to Load configuration even after Reset. 
* @param  None.
* @retval None.
*/
void ReadCurrentCfgFromEEPROM(CfgParam_Typedef *Config)               
{
  
  //Update the parameters to be saved in the EEPROM 
  
//   /*Write The Parameter in the Database*/
//   WMBus_ReadDeviceInfoFromEEPROM((uint8_t *)Config,
//                                  sizeof(CfgParam_Typedef),
//                                  EEPROM_CURRENT_CFG_SADDRESS);
}

/**
* @brief  This function writes the Default configuration data to the EEPROM.
*         This will change the Default configuration (To be used carefully). 
* @param  None.
* @retval None.
*/
void WriteDefaultCfgToEEPROM(void)                                               
{
//  uint16_t numByte;
  //Update the parameters to be saved in the EEPROM 
  
//   /*Write The Parameter in the Database*/
//   WMBus_WriteDeviceInfoToEEPROM((uint8_t *)&DefaultCfg,
//                                  sizeof(DefaultCfg),
//                                  EEPROM_DEFAULT_CFG_SADDRESS);
}


/**
* @brief  This function reads the current configuration data from EEPROM Database.
*         The purpose is to Load configuration even after Reset. 
* @param  None.
* @retval None.
*/
void ReadDefaultCfgFromEEPROM(CfgParam_Typedef *defaultCfg)             
{

  //Update the parameters to be saved in the EEPROM 
  
//   /*Write The Parameter in the Database*/
//   WMBus_ReadDeviceInfoFromEEPROM((uint8_t *)defaultCfg,
//                                  sizeof(CfgParam_Typedef),
//                                  EEPROM_DEFAULT_CFG_SADDRESS);
}


#ifndef NO_EEPROM

void ManageCfgFromEEPROM(void)
{
  CfgParam_Typedef tempCfg;

//  if(OverWriteFlag)
#ifdef STORE_NVM_DEVINFO
    OverWriteCurrCfgToEEPROM(CurrCfg);
#endif  
  ReadCurrentCfgFromEEPROM(&tempCfg);
    
  if(tempCfg.Flag)/*If configuration is present in EEPROM*/
  {
     CurrCfg = tempCfg;
  }
  else  /*If No configuration is present in EEPROM*/
  {
   WriteCurrentCfgToEEPROM();
  }
}


void OverWriteCurrCfgToEEPROM(CfgParam_Typedef tempCfg)
{
    //Write the parameters to be saved in the EEPROM 
  
   /*Write The Parameter in the Database*/
//   WMBus_WriteDeviceInfoToEEPROM((uint8_t *)&tempCfg,
//                                  sizeof(tempCfg),
//                                  EEPROM_CURRENT_CFG_SADDRESS);
}


/**
* @brief  This function will Read Meter EEPROM and put the data in the
*                      MeterDataBase structure.
* @param  None.
* @retval None.
*/
void UpdateDeviceSelfInfoFromEEPROM(void)
{
//  /*Read the device self info*/
//    WMBus_ReadDeviceInfoFromEEPROM((uint8_t*)&CurrCfg.DevInfo,
//      sizeof(DeviceInfo_t), EEPROM_DEVINFO_START);
}

#endif
/**
* @brief  This function Sends the Firmware and Hardware Version to PC. 
* @param  None.
* @retval None.
*/
void GetVersion(void)             
{

  uart_sendDMA((uint8_t*)"\n",1);
  uart_sendDMA((uint8_t*)"HW VERSION: ",13);
  uart_sendDMA((uint8_t*)"V1",2);
  
  uart_sendDMA((uint8_t*)"\n",1);
  uart_sendDMA((uint8_t*)"FW VERSION: ",13);
  uart_sendDMA((uint8_t*)"V1.1",4);
  uart_sendDMA((uint8_t*)"\n",1);
 
}




/**
* @brief  This function will find out the serial number(Array Index)
*         in Meter Database. 
* @param  rxFrame: Recieved Frame.
* @retval If meter found in database then the index of the meter else 0xFF.
*/
uint8_t GetDBMeterIndex(Frame_t  *rxFrame)
{
  uint16_t count=0;
  uint8_t isSuccess=0;
  
  while(count<MeterDatabase.DeviceCount)
  {       
    if((MeterDatabase.MeterList[count].MfrID[0]==rxFrame->m_field[0]) &&\
      (MeterDatabase.MeterList[count].MfrID[1]==rxFrame->m_field[1]) &&\
        (MeterDatabase.MeterList[count].IDNum[0]==rxFrame->a_field[0]) &&\
          (MeterDatabase.MeterList[count].IDNum[1]==rxFrame->a_field[1]) &&\
            (MeterDatabase.MeterList[count].IDNum[2]==rxFrame->a_field[2]) &&\
              (MeterDatabase.MeterList[count].IDNum[3]==rxFrame->a_field[3]) &&\
                (MeterDatabase.MeterList[count].Version==rxFrame->a_field[4]) &&\
                  (MeterDatabase.MeterList[count].MeterType==rxFrame->a_field[5]))
    {
      isSuccess=1;
      break;
    }
    count++; 
  }
  if(isSuccess==1)
  {
    return count;
  }
  else
  {
    return 0xFF;
  }
}

/**
* @brief  This function will Initialize all Application Attributes 
*         defined by User
* @param  None.
* @retval None.
*/
void WMBUS_AppliAttrInit(uint8_t isDefault)
{
  uint16_t xTime = 3;
  uint16_t yTime = 2;
  
  /*Generating random access number*/
  srand(xTime);
  AccessNumber = rand()%255;
  
  srand(yTime);
  EllAccessNum = rand()%255;
  
  if (isDefault==0) return;
  
#ifdef STORE_NVM_DEVINFO
   if( CurrCfg.DevType == OTHER) 
  {
    CurrCfg.DevInfo = DefaultMeterDatabase.MeterList[0];
    CurrCfg.DevInfo.MeterType = COMMUNICATION_CONTROLLER; 
    MeterDatabase = DefaultMeterDatabase;
      
    WriteMeterDatabaseToEEPROM();                                               
  }
  else
  {
    CurrCfg.DevInfo = DefaultMeterDatabase.MeterList[METER_DB_INDEX];
  }
  /* Write Self info and Meter Database */
    WriteDeviceSelfinfoToEEPROM();                                                
#else
  /*Read self info and meter database*/
  if( CurrCfg.DevType == OTHER) 
  {
//    UpdateMeterDatabaseFromEEPROM();
  } 
#endif  
  
  /*Set WMBus Device Basic Info : Manfr Id, Identification, Version, metertype*/
  if( CurrCfg.DevType == METER) 
  {
    AppliAttr.SerialNum  = RF_ADAPTER_SER_NUM;/*RF Module Serial Number*/
    memcpy(AppliAttr.DevID, CurrCfg.DevInfo.IDNum,4);
    AppliAttr.MeterDevType = CurrCfg.DevInfo.MeterType;
  }
  else if( CurrCfg.DevType == OTHER) 
  {
    AppliAttr.SerialNum  = OTHER_SERIAL_NUM;
    memcpy(AppliAttr.DevID, CurrCfg.DevInfo.IDNum,4);
    AppliAttr.MeterDevType = CurrCfg.DevInfo.MeterType;
  }
  
  

  memcpy(AppliAttr.DevManufrID, CurrCfg.DevInfo.MfrID,2);
  AppliAttr.DevVersion = CurrCfg.DevInfo.Version;
  
  /************Set extended link layer info****************/
  if(AppliAttr.ELLType == ELL_I)
  {
    AppliAttr.ELL_Len = ELL_I;
  }
  else if(AppliAttr.ELLType == ELL_III)
  {
    AppliAttr.ELL_Len = ELL_III;
  }
  else
  {
    AppliAttr.ELL_Len = ELL_0; 
  }
  
  AppliAttr.ELLAccessNum = EllAccessNum;
  
  AppliAttr.CIField = CI_APPLI_RESET_NONE;
  AppliAttr.HeaderLength = NO_HEADER;
  AppliAttr.AccessNumber = AccessNumber;
  AppliAttr.Status = 0x00;

  /*Set used bits of CONFIG_WORD*/
  AppliAttr.CONFIG_WORD.Accessibility = 0x00;
  AppliAttr.CONFIG_WORD.BidirComm = 0x01;
  AppliAttr.CONFIG_WORD.Mode = 0x05;
  AppliAttr.CONFIG_WORD.NumEncrBlocks = 0x0F;
  AppliAttr.CONFIG_WORD.Synchronous = 0x00;
  
  /************Set Default Encryption Key****************/
  memcpy(AppliAttr.EncKey,CurrCfg.DevInfo.EncrKey,ENCKEY_LEN);
  
  AppliAttr.HardwareRev = 0x10;
  AppliAttr.FirmwareRev = 0x02000000;       
  AppliAttr.AckTimeout = ACK_TIMEOUT;
}

/**
* @brief  This function copies the application attributes required by the 
*              link layer
* @param  None
* @retval None .
*/
void WMBus_LinkSetAppliParams(void)
{
  uint16_t manfrID,devid;
  
  WMBusLinkAppliParams_t* linkAppliParams=WMBus_Link_GetAppliParameterPointer();
    
  linkAppliParams->LinkEllLength=AppliAttr.ELL_Len;
  linkAppliParams->LinkHeaderLength=AppliAttr.HeaderLength;
  
  manfrID =  (AppliAttr.DevManufrID[0]) |( AppliAttr.DevManufrID[1]<<8);
  devid = AppliAttr.DevID[0]| (AppliAttr.DevID[1]<<8)|\
         (AppliAttr.DevID[2]<<16)| (AppliAttr.DevID[3]<<24);/**((uint32_t*)(&AppliAttr.DevID));**/
    
  linkAppliParams->LinkManufrID = manfrID; /**(uint16_t*)(&AppliAttr.DevManufrID);**/
  linkAppliParams->LinkDeviceID = devid;  /**(uint32_t*)(&AppliAttr.DevID);       **/
  
  linkAppliParams->LinkDeviceVersion = AppliAttr.DevVersion;  
  linkAppliParams->LinkMeterType = AppliAttr.MeterDevType;
  
  linkAppliParams->LinkAccessNumber= AppliAttr.AccessNumber;
  linkAppliParams->LinkStatusField=AppliAttr.Status;

  memcpy(&linkAppliParams->LinkConfigWord,&AppliAttr.CONFIG_WORD,2);
/**  linkAppliParams->LinkConfigWord = *(uint16_t*)(&AppliAttr.CONFIG_WORD); **/
  linkAppliParams->LinkTimeout=AppliAttr.AckTimeout;
  linkAppliParams->LinkFreqAccessCycle=AppliAttr.FrequentAccessCycle;
  linkAppliParams->LinkEllType=AppliAttr.ELLType;
  linkAppliParams->LinkEllAccessNum=AppliAttr.ELLAccessNum;
}

/**
* @brief  This function will set the Application layer attribute value.
* @param  Input Param: APPLI_ATTR_ID attr  - The Appli attribute.
*                     uint32_t value   - The value of the attribute.
* @retval APPLI_STATUS : APPLI_STATUS_SUCCESS
*       APPLI_STATUS_INVALID_ATTR   - The attribute ID is invalid.
*       APPLI_STATUS_INVALID_ATTR_VAL   - The attribute value is invalid.
*       APPLI_STATUS_PHY_ERROR_READ_ONLY_ADDRESS   - The attribute is read only.
*                       
*/
APPLI_STATUS WMBus_AppliSetAttribute(APPLI_ATTR_ID attr, uint32_t value)
{
  switch(attr)
  {  
  case APPLI_ATTR_DEV_ID_NUM:
    memcpy(&AppliAttr.DevID[0], &value, 4);
    break;
    
  case APPLI_ATTR_DEV_MFR_ID:
     memcpy(&AppliAttr.DevManufrID[0], &value, 2);
    break;
    
  case APPLI_ATTR_DEV_VERSION:
    AppliAttr.DevVersion = (uint8_t)value;
    break;
    
  case APPLI_ATTR_METER_DEV_TYPE:
    AppliAttr.MeterDevType = (uint8_t)value;
    break;
    
  case APPLI_ATTR_ACCESS_NUM:
    AppliAttr.AccessNumber = (uint8_t)value;
    break;
    
  case APPLI_ATTR_STATUS_FIELD:
    AppliAttr.Status = (uint8_t)value;
    break;
    
  case APPLI_ATTR_CONFIG_WORD:
    memcpy(&AppliAttr.CONFIG_WORD, &value,2);
    break;  
    
  case APPLI_ATTR_ENCKEY:
    memcpy(&AppliAttr.EncKey[0], &value, ENCKEY_LEN);
    break; 
    
  case APPLI_ATTR_HW_REV:
    AppliAttr.HardwareRev = (uint8_t)value;
    break;  
    
  case APPLI_ATTR_SW_REV:
    AppliAttr.FirmwareRev = (uint32_t)value;
    break;
    
  case APPLI_ATTR_ACK_TIMEOUT:
    AppliAttr.AckTimeout = (uint32_t)value;
    break; 

  case APPLI_ATTR_FAC:
    AppliAttr.FrequentAccessCycle = (uint8_t)value;
    break;    

  case APPLI_ATTR_ELL_TYPE:
    AppliAttr.ELLType = (ExtendedLinkLayer_t)value;
    break;    
    
  default:
    return APPLI_STATUS_INVALID_ATTR;
  }
  return APPLI_STATUS_SUCCESS;
}


/**
* @brief  This function returns the Appli layer attributes 
* @param  Input Param: 1. APPLI_ATTR_ID attr  - The Appli attribute.
*                      2. uint32_t* value   - The value of the attribute.
* @retval APPLI_STATUS : APPLI_STATUS_SUCCESS
*       APPLI_STATUS_INVALID_ATTR   - The attribute ID is invalid.
*                       
*/
APPLI_STATUS WMBus_AppliGetAttribute(APPLI_ATTR_ID attr, uint32_t* value)
{ 
  switch(attr)
  {          
  case APPLI_ATTR_DEV_ID_NUM:
    memcpy(value, &AppliAttr.DevID,4);
    break;
    
  case APPLI_ATTR_DEV_MFR_ID:
    memcpy(value, &AppliAttr.DevManufrID,2);
    break;
    
  case APPLI_ATTR_DEV_VERSION:
    *value = (uint32_t)AppliAttr.DevVersion;
    break;
    
  case APPLI_ATTR_METER_DEV_TYPE:
    *value = (uint32_t)AppliAttr.MeterDevType;
    break;
    
  case APPLI_ATTR_ACCESS_NUM:
    *value = (uint32_t)AppliAttr.AccessNumber;
    break;
    
  case APPLI_ATTR_STATUS_FIELD:
    *value = (uint32_t)AppliAttr.Status;
    break;
    
  case APPLI_ATTR_CONFIG_WORD:
    memcpy(value, &AppliAttr.CONFIG_WORD,2);
    break;
    
  case APPLI_ATTR_ENCKEY:
    memcpy(value, &AppliAttr.EncKey[0], ENCKEY_LEN);
    break; 
    
  case APPLI_ATTR_HW_REV:
    *value = (uint32_t)AppliAttr.HardwareRev;
    break; 
    
  case APPLI_ATTR_SW_REV:
    *value = (uint32_t)AppliAttr.FirmwareRev;
    break;
    
  case APPLI_ATTR_ACK_TIMEOUT:
    *value = (uint32_t)AppliAttr.AckTimeout;
    break;
    
  case APPLI_ATTR_FAC:
    *value = (uint32_t)AppliAttr.FrequentAccessCycle;
    break;    

  case APPLI_ATTR_ELL_TYPE:
    *value = (uint32_t)AppliAttr.ELLType;
    break;
    
  default:
    return APPLI_STATUS_INVALID_ATTR;
  }
  return APPLI_STATUS_SUCCESS;
}


/**
* @brief  This function encrypts the meter data before sending to the other dev 
* @param  none.
* @retval 1 (success) or 0 (failure).
*/
////uint8_t EncryptMeterData(uint8_t *txPayloadSize)
////{
////  ConfigWordField configWord;
////  uint32_t tempAttr=0;
////  WMBus_AppliGetAttribute(APPLI_ATTR_CONFIG_WORD,  (uint32_t*)(&configWord));
////
////  if(configWord.Mode==ENCR_MODE_5)
////  {
////    WMBus_AppliGetAttribute(APPLI_ATTR_DEV_MFR_ID,  &tempAttr);
////    AES_InitializationVector_Tx[0] = (uint8_t)((tempAttr));
////    AES_InitializationVector_Tx[1] = (uint8_t)(tempAttr>>8);
////    
////    WMBus_AppliGetAttribute(APPLI_ATTR_DEV_ID_NUM,  &tempAttr);  
////    AES_InitializationVector_Tx[2] = ((uint8_t)tempAttr);
////    AES_InitializationVector_Tx[3] = (uint8_t)(tempAttr>>8);
////    AES_InitializationVector_Tx[4] = (uint8_t)(tempAttr>>16);
////    AES_InitializationVector_Tx[5] = (uint8_t)(tempAttr>>24);
////    
////    WMBus_AppliGetAttribute(APPLI_ATTR_DEV_VERSION,  &tempAttr);
////    AES_InitializationVector_Tx[6] = (uint8_t)(tempAttr);
////    
////    WMBus_AppliGetAttribute(APPLI_ATTR_METER_DEV_TYPE,  &tempAttr);
////    AES_InitializationVector_Tx[7] = (uint8_t)(tempAttr);
////    
////    WMBus_AppliGetAttribute(APPLI_ATTR_ACCESS_NUM,  &tempAttr);
////    AES_InitializationVector_Tx[8] = (uint8_t)(tempAttr);                                             
////    AES_InitializationVector_Tx[9] = (uint8_t)(tempAttr);                                            
////    AES_InitializationVector_Tx[10] = (uint8_t)(tempAttr);                                            
////    AES_InitializationVector_Tx[11] = (uint8_t)(tempAttr);                                           
////    AES_InitializationVector_Tx[12] = (uint8_t)(tempAttr);                                           
////    AES_InitializationVector_Tx[13] = (uint8_t)(tempAttr);                                           
////    AES_InitializationVector_Tx[14] = (uint8_t)(tempAttr);                                          
////    AES_InitializationVector_Tx[15] = (uint8_t)(tempAttr); 
////    
////    *txPayloadSize = WMBus_AppliEncrypt(CurrCfg.DevInfo.EncrKey,
////                                        AES_InitializationVector_Tx,
////                                        PayloadBuffer,
////                                        PayloadBuffer,
////                                        PayloadSize);
////    
////    configWord.NumEncrBlocks= *txPayloadSize / ENCR_BLOCK_SIZE;
////    WMBus_AppliSetAttribute(APPLI_ATTR_CONFIG_WORD,*((uint32_t*)(&configWord)));
////  }
////  return 1;
////}

/**
* @brief  This function encrypts other device data before sending to meter 
* @param  none.
* @retval 1 (success) or 0 (failure).
*/
////uint8_t EncryptConcData(uint8_t *txPayloadSize)
////{
////  uint8_t isSuccess=1; 
////  ConfigWordField configWord;
////  uint32_t tempAttr=0;
////  uint8_t meterIndex=0;
////  
////  WMBus_AppliGetAttribute(APPLI_ATTR_CONFIG_WORD,  (uint32_t*)(&configWord));
////
////  if(configWord.Mode==ENCR_MODE_5)
////  {
////    if(meterIndex!=0xFF)
////    {
////      WMBus_AppliGetAttribute(APPLI_ATTR_DEV_MFR_ID,  &tempAttr);
////      AES_InitializationVector_Tx[0] = (uint8_t)((tempAttr));
////      AES_InitializationVector_Tx[1] = (uint8_t)(tempAttr>>8);
////      
////      WMBus_AppliGetAttribute(APPLI_ATTR_DEV_ID_NUM,  &tempAttr);  
////      AES_InitializationVector_Tx[2] = ((uint8_t)tempAttr);
////      AES_InitializationVector_Tx[3] = (uint8_t)(tempAttr>>8);
////      AES_InitializationVector_Tx[4] = (uint8_t)(tempAttr>>16);
////      AES_InitializationVector_Tx[5] = (uint8_t)(tempAttr>>24);
////      
////      WMBus_AppliGetAttribute(APPLI_ATTR_DEV_VERSION,  &tempAttr);
////      AES_InitializationVector_Tx[6] = (uint8_t)(tempAttr);
////      
////      WMBus_AppliGetAttribute(APPLI_ATTR_METER_DEV_TYPE,  &tempAttr);
////      AES_InitializationVector_Tx[7] = (uint8_t)(tempAttr);
////      
////      WMBus_AppliGetAttribute(APPLI_ATTR_ACCESS_NUM,  &tempAttr);
////      AES_InitializationVector_Tx[8] = (uint8_t)(tempAttr);                                             
////      AES_InitializationVector_Tx[9] = (uint8_t)(tempAttr);                                            
////      AES_InitializationVector_Tx[10] = (uint8_t)(tempAttr);                                            
////      AES_InitializationVector_Tx[11] = (uint8_t)(tempAttr);                                           
////      AES_InitializationVector_Tx[12] = (uint8_t)(tempAttr);                                           
////      AES_InitializationVector_Tx[13] = (uint8_t)(tempAttr);                                           
////      AES_InitializationVector_Tx[14] = (uint8_t)(tempAttr);                                          
////      AES_InitializationVector_Tx[15] = (uint8_t)(tempAttr); 
////      
////      *txPayloadSize  = WMBus_AppliEncrypt(MeterDatabase.MeterList[meterIndex].EncrKey,
////                                           AES_InitializationVector_Tx,                                       
////                                           PayloadBuffer ,
////                                           PayloadBuffer ,
////                                           PayloadSize);
////    configWord.NumEncrBlocks= *txPayloadSize / ENCR_BLOCK_SIZE;
////    WMBus_AppliSetAttribute(APPLI_ATTR_CONFIG_WORD,  *((uint32_t*)(&configWord)));
////    }
////    else
////    {
////      /*Error : Meter not in database*/
////      isSuccess=0;
////    }
////  } 
////  return isSuccess;
////}

/**
* @brief  This function decrypts the data received at concentrator side
* @param  Frame *frame.
* @retval 1 (success) or 0 (failure).
*/
////uint8_t DecryptConcData(Frame_t * frame)
////{
////  uint8_t isSuccess=1;
////  ConfigWordField configWord;
////  uint8_t meterIndex=0;
////  
////  WMBus_AppliGetAttribute(APPLI_ATTR_CONFIG_WORD,  (uint32_t*)(&configWord));
////  
////  if(configWord.Mode==ENCR_MODE_5)
////  {  
////    AES_InitializationVector_Rx[0] = frame->m_field[0]; /*lsb of manufacture id*/ 
////    AES_InitializationVector_Rx[1] = frame->m_field[1]; /*msb of manufacure id*/
////    AES_InitializationVector_Rx[2] = frame->a_field[0];
////    AES_InitializationVector_Rx[3] = frame->a_field[1];
////    AES_InitializationVector_Rx[4] = frame->a_field[2];
////    AES_InitializationVector_Rx[5] = frame->a_field[3];
////    AES_InitializationVector_Rx[6] = frame->a_field[4];
////    AES_InitializationVector_Rx[7] = frame->a_field[5];
////    AES_InitializationVector_Rx[8] = frame->data_field.header.accessNum;
////    AES_InitializationVector_Rx[9] = frame->data_field.header.accessNum;
////    AES_InitializationVector_Rx[10] = frame->data_field.header.accessNum;
////    AES_InitializationVector_Rx[11] = frame->data_field.header.accessNum;
////    AES_InitializationVector_Rx[12] = frame->data_field.header.accessNum;
////    AES_InitializationVector_Rx[13] = frame->data_field.header.accessNum;
////    AES_InitializationVector_Rx[14] = frame->data_field.header.accessNum;
////    AES_InitializationVector_Rx[15] = frame->data_field.header.accessNum;
////    
////    meterIndex = GetDBMeterIndex(frame);
////    if(meterIndex!=0xFF)
////    {
////      WMBus_AppliDecrypt(MeterDatabase.MeterList[meterIndex].EncrKey,
////                         AES_InitializationVector_Rx,       
////                         frame->data_field.payload,
////                         frame->data_field.payload,
////                         frame->data_field.size);
////    }
////    else
////    {
////      /*Error : Meter not in database*/
////      isSuccess=0;
////    }
////  }
////  return isSuccess;
////}

/**
* @brief  This function decrypts the data received at meter side. 
* @param  Frame *frame.
* @retval 1 (success) or 0 (failure).
*/
////uint8_t DecryptMeterData(Frame_t * frame)
////{
////  ConfigWordField configWord;
////  
////  WMBus_AppliGetAttribute(APPLI_ATTR_CONFIG_WORD,  (uint32_t*)(&configWord));
////
////  if(configWord.Mode==ENCR_MODE_5)
////  {
////    AES_InitializationVector_Rx[0] = frame->m_field[0]; /*lsb of manufacture id*/ 
////    AES_InitializationVector_Rx[1] = frame->m_field[1]; /*msb of manufacure id*/
////    AES_InitializationVector_Rx[2] = frame->a_field[0];
////    AES_InitializationVector_Rx[3] = frame->a_field[1];
////    AES_InitializationVector_Rx[4] = frame->a_field[2];
////    AES_InitializationVector_Rx[5] = frame->a_field[3];
////    AES_InitializationVector_Rx[6] = frame->a_field[4];
////    AES_InitializationVector_Rx[7] = frame->a_field[5];
////    
////    AES_InitializationVector_Rx[8] =  frame->data_field.header.accessNum;
////    AES_InitializationVector_Rx[9] =  frame->data_field.header.accessNum;
////    AES_InitializationVector_Rx[10] = frame->data_field.header.accessNum;
////    AES_InitializationVector_Rx[11] = frame->data_field.header.accessNum;
////    AES_InitializationVector_Rx[12] = frame->data_field.header.accessNum;
////    AES_InitializationVector_Rx[13] = frame->data_field.header.accessNum;
////    AES_InitializationVector_Rx[14] = frame->data_field.header.accessNum;
////    AES_InitializationVector_Rx[15] = frame->data_field.header.accessNum;
////    
////    
////    WMBus_AppliDecrypt(CurrCfg.DevInfo.EncrKey,
////                       AES_InitializationVector_Rx,
////                       frame->data_field.payload,
////                       frame->data_field.payload,
////                       frame->data_field.size);
////  }
////  return 1;
////}

/**
* @brief  This routine updates the respective status for key press.
* @param  None
* @retval None
*/
void Set_KeyStatus(void)
{
  KEYStatus=1;
}

/**
* @brief  This routine updates the respective status for SEL key press.
* @param  None
* @retval None
*/
void Set_SELStatus(void)
{
  SELStatus = 1;
}

/**
* @brief  This routine updates the respective status for Down key press.
* @param  None
* @retval None
*/
void Set_DOWNStatus()
{
  DOWNStatus=1;
}

/**
* @brief  This routine updates the respective status for Up key press.
* @param  None
* @retval None
*/
void Set_UPStatus(void)
{
  UPStatus=1;
}

/**
* @brief  This routine updates the respective status for Right key press.
* @param  None
* @retval None
*/
void Set_RightStatus(void)
{
  RIGHTStatus=1;
}

/**
* @brief  This routine updates the respective status for Left key press.
* @param  None
* @retval None
*/
void SetLeftKeyStatus(void)
{
  LEFTStatus=1;
}


/**
* @brief  This function will write all meter database in Concentrator EEPROM
* @param  None.
* @retval None.
*/
void WriteMeterDatabaseToEEPROM(void)
{
//  /*Write meter database*/
//   WMBus_WriteDeviceInfoToEEPROM((uint8_t *)&MeterDatabase,
//     sizeof(MeterDataBase_t), EEPROM_METERDB_START);
}

/******************************************************************************/
/**
 * @brief Handle the activity LED timer callback.
 *
 * This function handles the activity LED timer callback when the LED timer
 * expires. This ensures that the LED is turned off after being previously
 * being turned on when data was received or transmitted by the S2LP.
 */
void app_tx_rx_led_timer_fn(void)
{
    app_tx_rx_led_off();
}

/******************************************************************************/
/**
 * @brief Handle the transmit delay timer callback.
 *
 * This function handles the TX delay timer callback when the timer expires.
 */
void app_rf_tx_delay_timer_fn(void)
{
    g_rf_tx_timeout = 1;
}



/******************************************************************************/
/**
 * @brief Function to turn the TX TX LED off
 */
void app_tx_rx_led_off(void)
{
////    if (config_get()->tx_rx_led == 2)
////    {
////        gpio_set_state(LED_PORT, LED_PIN, 0);
////    }
////    else
////    {
////        gpio_set_state(LED_PORT, LED_PIN, 1);
////    }
}



/******************************************************************************/
/**
 * @brief The application command mode loop.
 */
void app_command_mode(void)
{

  if((CurrCfg.ModuleFunction == WMBUS_FUNCT) || (CurrCfg.ModuleFunction == SNIFFER_FUNCT))
  {
  switch(CommandMode)
  {
    
  case FIFO_INIT:
    fifo_init(g_p_uart_rx_fifo);
    
    CommandMode = FIFO_PROCESS;
      
    break;
    
  case FIFO_PROCESS:
    
    app_process_uart_rx_fifo();
    
    if(g_cmd_received)
      CommandMode = FIFO_PROCESS_COMMAND;
    
    break;
    
  case FIFO_PROCESS_COMMAND:
    cmd_process(g_cmd_buf, g_cmd_buf_len);
    
    if(g_cmd_received == 0)
      CommandMode = FIFO_CLEAR_COMMAND;
    
    break;
    
  case FIFO_CLEAR_COMMAND:
    app_clear_command_buffer();
    
    CommandMode = FIFO_INIT;
    
    break;
       
  default:
    break;
    
  }
  }
}



/******************************************************************************/
/**
 * @brief Process data received from the uart, from the rx fifo.
 */
void app_process_uart_rx_fifo(void)
{
    uint8_t rx = 0;
    
    if (fifo_dequeue(g_p_uart_rx_fifo, &rx))
    {
        // Convert character to upper case
        if ((rx >= 'a') && (rx <= 'z'))
        {
            rx = rx + 'A' - 'a';
        }

        if (rx != LF)
        {
            if (g_cmd_buf_len == CMD_BUF_SIZE)
            {
                g_cmd_buf_len = 0;
            }

            g_cmd_buf[g_cmd_buf_len++] = rx;

            if (rx == CR)
            {
                g_cmd_received = 1;
            }
        }
    }
}


/******************************************************************************/
/**
 * @brief Clear the command buffer for the UART interface.
 *
 * This function is called to clear the command buffer for data received on
 * the UART interface, after the data has been processed.
 */
void app_clear_command_buffer(void)
{
    memset((uint8_t*)g_cmd_buf, 0, CMD_BUF_SIZE);
    g_cmd_buf_len = 0;
}

/******************************************************************************/
/**
 * @brief Send a response string on the UART interface.
 *
 * This function wraps the UART send function and calculates the string length
 * to avoid having to specify it.
 *
 * @param  p_response  The NULL terminated response string to send.
 */
void app_send_response(const char* p_response)
{
  uart_sendDMA((uint8_t*)p_response, strlen(p_response));
}


/******************************************************************************/
/**
 * @brief Function to turn the TX TX LED on
 */
void app_tx_rx_led_on(void)
{
//    if (config_get()->tx_rx_led == 2)
//    {
//        gpio_set_state(LED_PORT, LED_PIN, 1);
//    }
//    else
//    {
//        gpio_set_state(LED_PORT, LED_PIN, 0);
//    }
}




/******************************************************************************/
/**
 * @brief Initialize the txrx LED into its configured state.
 */
void app_init_txrx_led(void)
{
//    if (config_get()->tx_rx_led == 2)
//    {
//        gpio_config(LED_PORT, LED_PIN, GPIO_PP);
//        gpio_set_state(LED_PORT, LED_PIN, 0);
//    }
//    else
//    {
//        gpio_config(LED_PORT, LED_PIN, GPIO_OD);
//        gpio_set_state(LED_PORT, LED_PIN, 1);
//    }
}

/******************************************************************************/
/**
 * @brief Set the application mode, generally operating mode or command mode.
 *
 * @param  mode  The app mode to set.
 */
void app_set_mode(app_mode_t mode)
{
    if ((mode == APP_MODE_COMMAND) || 
        (mode ==  APP_MODE_OPERATING) ||
        (mode == APP_MODE_MTC))
    {
        g_app_mode = mode;
    }
    else
    {
//        assert(0);                                                            
    }
}


/******************************************************************************/
/**
 * @brief Process mode switching triggered by the mode pin.
 */
void app_process_mode_switch()
{
   if (g_mode != g_mode_prev)
    {
        g_mode_prev = g_mode;
        
        if (g_mode)
        {
            if (g_app_mode == APP_MODE_OPERATING)
            {
                g_app_mode = APP_MODE_COMMAND;
            }
        }
        else
        {
            if (g_app_mode == APP_MODE_COMMAND)
            {
                g_app_mode = APP_MODE_OPERATING;
            }
        }
    }
}

/////**
////* @brief  This callback is called when SND-IR is received at other device.
////* @param  *frame: Pointer to received frame
////* @retval SBool: S_TRUE/S_FALSE
////*/
////uint8_t WMBus_LinkInstallReq_CB(Frame_t *frame)
////{
////  LinkFrameReceived(frame);
////  WMBus_LinkSendConfirmInstallationRequest(0,0,CI_TRANSPORT_TO_DEV_LONG);
////  return S_TRUE;
////}
////
/////**
////* @brief  This callback is called when SND-NR is received at other device.
////* @param  *frame: Pointer to received frame
////* @retval SBool: S_TRUE/S_FALSE
////*/
////uint8_t WMBus_LinkSendNoReply_CB(Frame_t *frame)
////{
////  LinkFrameReceived(frame);
////  return S_TRUE;
////}
////
/////**
////* @brief  This callback is called when ACC-NR is received at other device.
////* @param  *frame: Pointer to received frame
////* @retval SBool: S_TRUE/S_FALSE
////*/
////uint8_t WMBus_LinkAccessNoReply_CB(Frame_t *frame)
////{
////  LinkFrameReceived(frame);
////  return S_TRUE;
////}
////
/////**
////* @brief  This callback is called when RSP-UD is received at other device.
////* @param  *frame: Pointer to received frame
////* @retval SBool: S_TRUE/S_FALSE
////*/
////uint8_t WMBus_LinkResponseUD_CB(Frame_t *frame)
////{
////  LinkFrameReceived(frame);
////  return S_TRUE;
////}
////
////
////
/////**
////* @brief  This callback is called when ACC_DMD is received at other device.
////* @param  *frame: Pointer to received frame
////* @retval SBool: S_TRUE/S_FALSE
////*/
////uint8_t WMBus_LinkAccessDemand_CB(Frame_t *frame)
////{
////  LinkFrameReceived(frame);
//////  WMBus_LinkSendAck(CI_TRANSPORT_FROM_DEV_LONG);
////  return S_TRUE;
////}
////
////#ifdef WMBUS_GUI
/////**
////* @brief  RSP-UD Timeout callback, called when RSP_UD is not received.
////* @param  None
////* @retval SBool: S_TRUE/S_FALSE
////*/
////uint8_t WMBus_LinkRspUdTimeout_CB(void)
////{
////  
////  WMBus_LinkRequestUserData2(0,0,CI_TRANSPORT_TO_DEV_LONG);
////  return S_TRUE;
////}
////#endif
/////******************************Callbacks from meter****************************/
////
/////**
////* @brief  This callback is called when CNF-IR is received at meter.
////* @param  *frame: Pointer to received frame
////* @retval SBool: S_TRUE/S_FALSE
////*/
////uint8_t WMBus_LinkConfirmInstallReq_CB(Frame_t *frame)
////{
////  LinkFrameReceived(frame);
////  return S_TRUE;
////}
////
////
////
/////**
////* @brief  This callback is called when SND-UD is received at meter.
////* @param  *frame: Pointer to received frame
////* @retval SBool: S_TRUE/S_FALSE
////*/
////uint8_t WMBus_LinkSendUD_CB(Frame_t *frame)
////{
////  
////  LinkFrameReceived(frame);
//////  WMBus_LinkSendAck(CI_TRANSPORT_FROM_DEV_LONG);
////  return S_TRUE;
////}
////
/////**
////* @brief  This callback is called when REQ-UD2/SND-UD2 is received at meter.
////* @param  *frame: Pointer to received frame
////* @retval SBool: S_TRUE/S_FALSE
////*/
////uint8_t WMBus_LinkRequestUD2_CB(Frame_t *frame)
////{
////  LinkFrameReceived(frame);
////  WMBus_LinkSendResponse_UD(PktPayload,
////                            PktPayloadSize,
////                            PktCIField);
////  return S_TRUE;
////}
////
/////**
////* @brief  This callback is called when REQ-UD1 is received at meter.
////* @param  *frame: Pointer to received frame
////* @retval SBool: S_TRUE/S_FALSE
////*/
////uint8_t WMBus_LinkRequestUD1_CB(Frame_t *frame)
////{
////  LinkFrameReceived(frame);
////  WMBus_LinkSendResponse_UD(PktPayload,
////                            PktPayloadSize,
////                            PktCIField);
////  return S_TRUE;
////}
////
/////**
////* @brief  This callback is called when SND-NKE is received at meter.
////* @param  *frame: Pointer to received frame
////* @retval SBool: S_TRUE/S_FALSE
////*/
////uint8_t WMBus_LinkSendNKE_CB( Frame_t *frame)
////{
////  LinkFrameReceived(frame);
////  return S_TRUE;
////}
////
/////******************************************************************************/
////
////
/////**
////* @brief  Send ACK callback.
////* @param  *frame: Pointer to received frame
////* @retval SBool: S_TRUE/S_FALSE
////*/
////void WMBus_LinkSendAck_CB(Frame_t *frame)
////{
////  
//////  WMBus_LinkSendAck(CI_TRANSPORT_FROM_DEV_LONG);
//////  return S_TRUE;
////}
////
////
/////**
////* @brief  Tx Completed callback.
////* @param  None
////* @retval SBool: S_TRUE/S_FALSE
////*/
//////void WMBus_LinkTxCompleted_CB(Tx_Outcome_t tx_outcome, Frame_t *frame)
//////{
//////  
////////  return S_TRUE;
//////}
////
////
////#ifdef WMBUS_GUI
/////**
////* @brief  CNF-IR timeout callback.
////* @param  None
////* @retval SBool: S_TRUE/S_FALSE
////*/
////uint8_t WMBus_LinkCnfIRTimeout_CB(void)
////{
////  
////  return S_TRUE;
////}
////
/////**
////* @brief  This callback is called when ACK is received at meter/Other device.
////* @param  *frame: Pointer to received frame
////* @retval SBool: S_TRUE/S_FALSE
////*/
////uint8_t WMBus_LinkAckReceived_CB(Frame_t *frame)
////{
////  LinkFrameReceived(frame);
////  return S_TRUE;
////}
////
/////**
////* @brief  ACK timeout callback.
////* @param  None
////* @retval SBool: S_TRUE/S_FALSE
////*/
////uint8_t WMBus_LinkAckTimeout_CB(void)
////{
////  
////  return S_TRUE;
////}
////
/////**
////* @brief  Rx Timeout callback.
////* @param  None
////* @retval SBool: S_TRUE/S_FALSE
////*/
////uint8_t WMBus_LinkRxTimeout_CB(void)                            
////{
////  
////  return S_TRUE;
////}
////
/////**
////* @brief  This callback check for meter data.
////* @param  None
////* @retval SBool: S_TRUE/S_FALSE
////*/
////uint8_t WMBus_LinkIsMeterDataAvailable_CB(void)
////{
////  
////  return S_TRUE;
////}
////
/////**
////* @brief  This callback check for alarm data.
////* @param  None
////* @retval SBool: S_TRUE/S_FALSE
////*/
////uint8_t WMBus_LinkIsAlarmDataAvailable_CB(void)
////{
////  
////  return S_TRUE;
////}

////
/////*Copy only the required data to the buffer */
////void LinkFrameReceived(Frame_t* frm )
////{
////  uint8_t offset=0;
////  uint8_t dataSize=0;
////  uint8_t dataCounter=0;
////  float rssi=0.0;
////  /* Time stamp */ 
////  LinkFrameData[offset++]= (uint8_t)(frm->timestamp>>0);
////  LinkFrameData[offset++]= (uint8_t)(frm->timestamp>>8);
////  LinkFrameData[offset++]= (uint8_t)(frm->timestamp>>16);
////  LinkFrameData[offset++]= (uint8_t)(frm->timestamp>>24);
////  
////  rssi=frm->RSSI_field;
////  
////  LinkFrameData[offset++]= *((uint8_t*)(&rssi)+0);
////  LinkFrameData[offset++]= *((uint8_t*)(&rssi)+1);
////  LinkFrameData[offset++]= *((uint8_t*)(&rssi)+2);
////  LinkFrameData[offset++]= *((uint8_t*)(&rssi)+3);
////  
////  /* L C M A CRC CI Fields Offset 8*/
////  LinkFrameData[offset++]= 0; //Not required by GUI
////  
////  LinkFrameData[offset++]= frm->c_field;
////  
////  LinkFrameData[offset++]= frm->m_field[0];
////  LinkFrameData[offset++]= frm->m_field[1];
////  
////  LinkFrameData[offset++]= frm->a_field[0];
////  LinkFrameData[offset++]= frm->a_field[1];
////  LinkFrameData[offset++]= frm->a_field[2];
////  LinkFrameData[offset++]= frm->a_field[3];
////  LinkFrameData[offset++]= frm->a_field[4];
////  LinkFrameData[offset++]= frm->a_field[5];  
////  
////  LinkFrameData[offset++]= 0; //Not required by GUI
////  LinkFrameData[offset++]= 0; //Not required by GUI
////  /* Offset 8 + 12*/
////  LinkFrameData[offset++]= frm->ci_field;
////  /* Data Length */
////  dataSize= frm->data_field.size;
//// 
//// LinkFrameData[offset++]=  dataSize;
////  /* offset 8 + 12 + 2 */
////  /* Data Payload upto 32 bytes only */ 
//// if(dataSize>32)
//// {
////   dataSize=32;
//// }
////  
//// for(dataCounter =0; dataCounter<dataSize;dataCounter++)
//// {
////   LinkFrameData[offset+ dataCounter]= frm->data_field.payload[dataCounter];
//// }
//// 
//// offset+=dataSize;
//// 
////  FrameRecd=1;
////  //Local processing
////}



//#endif



#if defined (LPM_ENABLE)
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
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable Ultra low power mode */
  HAL_PWREx_EnableUltraLowPower();
  
  /* Enable the fast wake up from Ultra low power mode */
  HAL_PWREx_EnableFastWakeUp();

  /* Select HSI as system clock source after Wake Up from Stop mode */
  __HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_StopWakeUpClock_HSI);
  
  /* Enable GPIOs clock */
  
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();
  __GPIOH_CLK_ENABLE();
  

  /* Configure GPIO Pins: PA0,PA1,PA2,PA3,PA4,PA5,PA8,PA9,PA11,PA12,PA13,\
  PA14,PA15 in Analog Input mode (floating input trigger OFF) */
  
  /* PA6: SDO, PA7: SDI, PA10 :SDn  are not put in analog mode  */
    
  GPIO_InitStructure.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | \
    GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_11 | \
      GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure); 
  
  /* Configure GPIO Pins: PB0,PB1,PB2,PB4,PB5,PB7,PB8,PB9,PB10,PB11,PB12,PB13,\
  PB14,PB15 in Analog Input mode (floating input trigger OFF) */
  
  /*PB6: CSn, PB3: SCLK are not put in analog mode  */
  
  GPIO_InitStructure.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_4 | \
    GPIO_PIN_5 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | \
      GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  /* Configure GPIO Pins: PC0,PC1,PC2,PC3,PC4,PC5,PC6,PC8,PC9,PC10,PC11,PC12,\
  PC13,PC14,PC15 in Analog Input mode (floating input trigger OFF) */
  
  /*PC7: GPIO3, PC13: Push Button are not put in analog mode  */
  
  GPIO_InitStructure.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | \
    GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_8 | GPIO_PIN_9 | \
      GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  /* Configure all GPIO port pins in Analog Input mode (floating input trigger OFF) */
  GPIO_InitStructure.Pin = GPIO_PIN_All;
  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = GPIO_NOPULL;

  HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
  HAL_GPIO_Init(GPIOH, &GPIO_InitStructure);

  /* Disable GPIOs clock */
  __GPIOA_CLK_DISABLE();
  __GPIOB_CLK_DISABLE();
  __GPIOC_CLK_DISABLE();
  __GPIOD_CLK_DISABLE();
  __GPIOH_CLK_DISABLE();
  
}
#endif



#ifdef  USE_FULL_ASSERT
/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param file: pointer to the source file name
* @param line: assert_param error line source number
* @retval : None
*/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number */
  printf("Wrong parameters value: file %s on line %d\r\n", file, line);
  
  /* Infinite loop */
  while (1)
  {
  }
}
#endif

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
