/**
* @file    user_appli.h
* @author  System Lab - NOIDA
* @version V1.0.0
* @date    15-Mar-2016
* @brief  This file contains header for user_appli.c
* @details
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USER_APPLI_H
#define __USER_APPLI_H
/* Includes ------------------------------------------------------------------*/
//#include "stm32l1xx.h"
#include "wmbus_link.h"
//#include "user_config.h"
//#include "wmbus_phydatamem.h"
//#include "timer.h"
#include "fifo.h"
#include "command.h"
#include "wmbus_link.h"

/** @addtogroup Application Application
* @{
*/

/** @addtogroup User Application    User Application
* @brief Configuration and management of WMBus Application layer.
* @details See the file <i>@ref user_appli.h</i> for more details.
* @{
*/

/** @defgroup user_appli_Exported_Macros        User Application Exported Macros
* @{
*/
#define IS_WMBUS_APPLI_ATTR(attr)  (((attr) >= APPLI_ATTR_ID_START)&&\
                                   ((attr) <= APPLI_ATTR_ID_END))

#define APPLI_ATTR_ID_START        0x55/*!< Appli Layer Attribute Start index; */


#define WMBUS_FUNCT      0 
#define SNIFFER_FUNCT    1
#define RAW_FUNCT        2


//#define HARDWARE_ID    "\nSTEVAL-SP1ML868\n"
//#define FIRMWARE_ID    "\nSP1ML-WMBUS\n" 
//
//
//// General response messages
//#define RESPONSE_OK                 "\nOK\n\r"
//#define RESPONSE_ERROR              "\nERROR\n\r"
//#define RESPONSE_ERROR_PARAM        "\nERROR PARAM\n\r"
//#define RESPONSE_ERROR_VALUE        "\nERROR VALUE\n\r"
//#define COMMAND_ERROR                "\nCOMMAND ERROR\n\r"


////#define VIF_EXT_TEMP         0x64 //0b01100100
////#define VIF_EXT_TEMP_MASK    0x7C //0b01111100
////#define DIF_CODE             0x00 //0b00000000   // | INSTATNAT_VAL
////#define VIF_CODE    

#define INSTATNAT_VAL     ((0x00))
#define MAX_VAL           ((0x01))
#define MIN_VAL           ((0x02))
#define VAL_DURING_ERROR  ((0x03))   
 
////#define MULTIPLIER_001    0x00  /***(Multiplier will be 10^(0-3) = 0.001)***/
////#define MULTIPLIER_010    0x01  /***(Multiplier will be 10^(1-3) = 0.01) ***/
////#define MULTIPLIER_100    0x02  /***(Multiplier will be 10^(2-3) = 0.1)  ***/
////#define MULTIPLIER_ONE    0x03  /***(Multiplier will be 10^(3-3) = 1)    ***/


#define BIT_LEN_0                       0x00   /**No Data**/
#define BIT_LEN_8                       0x01   /**8 bit integer/binary**/
#define BIT_LEN_16                      0x02   /**16 bit integer/binary**/
#define BIT_LEN_24                      0x03   /**24 bit integer/binary**/
#define BIT_LEN_32                      0x04   /**32 bit integer/binary**/
#define BIT_LEN_32_N                    0x05   /**32 bit Real**/
#define BIT_LEN_48                      0x06   /**48 bit integer/binary**/
#define BIT_LEN_64                      0x07   /**64 bit integer/binary**/


#define VIF_VOLUME              0x10 //0b00010000
#define DIF_CODE                0x00 //0b00000000   // | INSTATNAT_VAL
#define VIF_CODE    


#define MULTIPLIER_ONE        0x06  /***(Multiplier will be 10^(6-6) = 1)  ***/

#define INIT_VECTOR_SIZE        16
#define MAX_DATA_LEN            255


/**
* @}
*/

/** @defgroup user_appli_Exported_Types          User Application Exported Types
* @{
*/
/**
* @brief  Other device command flags.
*/
//typedef enum
//{
//  DATA_FLAG = 0,
//  ALARM_FLAG = 1,
//  SND_UD_FLAG = 2,
//  SND_UD2_FLAG = 3,
//  RESET_LINK_FLAG =4,
//  RESET_ALL_FLAG = 5
//} OtherCmdFlag;

/**
* @brief  Application layer Attributes IDs.
*/
typedef enum APPLI_ATTR_ID
{  
  /*!< The device Identification number */
  APPLI_ATTR_DEV_ID_NUM = APPLI_ATTR_ID_START,
  /*!< The device Manufacturer ID */
  APPLI_ATTR_DEV_MFR_ID,
  /*!< The device Version*/
  APPLI_ATTR_DEV_VERSION,
  /*!< The device type, for details refer EN 13757-3*/
  APPLI_ATTR_METER_DEV_TYPE,
  /*!< The Access number */
  APPLI_ATTR_ACCESS_NUM,
  /*!< The Status field */
  APPLI_ATTR_STATUS_FIELD,
  /*!< The Configuration word; used for encryption*/
  APPLI_ATTR_CONFIG_WORD,
  /*!< The Encryption key*/
  APPLI_ATTR_ENCKEY,
  /*!< The Hardware Revision */
  APPLI_ATTR_HW_REV,
  /*!< The Firmware Revision */
  APPLI_ATTR_SW_REV,
  /*!< The Ack timeout, this is set by User*/
  APPLI_ATTR_ACK_TIMEOUT,  
  /*!< The Frequent ACcess Cycle, this is set by User*/ 
  APPLI_ATTR_FAC,
  /*!< Extended Link Layer Type, this is set by User*/ 
  APPLI_ATTR_ELL_TYPE
    
}APPLI_ATTR_ID;


typedef enum
{
  FIFO_INIT,
  FIFO_PROCESS,
  FIFO_PROCESS_COMMAND,
  FIFO_CLEAR_COMMAND,
  FIFO_DEFAULT_MODE
}CommandMode_TypedefEnum;


typedef enum
{
    APP_MODE_COMMAND=0,
    APP_MODE_OPERATING,
    APP_MODE_MTC
} app_mode_t;

/**
* @brief  Appli Layer Status Enum.
*/
typedef enum APPLI_STATUS
{
  /* APPLI layer status codes */
  APPLI_STATUS_SUCCESS = 0x00,
  /*!< The attribute ID is invalid*/
  APPLI_STATUS_INVALID_ATTR,
  /*!<Data is not received within the timeout period specified*/
  APPLI_STATUS_UNKNOWN_ERROR = 0xFF
}APPLI_STATUS;

#pragma pack(1)
/**
* @brief  Device information structure
*/
typedef struct
{
  uint8_t  MfrID[2];
  uint8_t  IDNum[4];
  uint8_t  Version;
  uint8_t  MeterType;
  uint8_t  EncrKey[16];
}DeviceInfo_t;

#pragma pack(1)

typedef struct 
{
  uint8_t  Flag;
  uint8_t  ModuleFunction;
  uint8_t  DevType;
  uint8_t  Channel;
  uint8_t  FrameFormat;
  uint8_t  WorkingMode;
  uint32_t UartBaudRate;
  uint16_t EncrModeVar;
  DeviceInfo_t DevInfo;
  
}CfgParam_Typedef;


#pragma pack(1)

#define EEPROM_METER_DB_MAXCOUNT 10


/**
* @brief  Meter Database structure
*/
typedef struct
{
  uint32_t  DataPresentFlag;  
  uint32_t  DeviceCount;
  DeviceInfo_t MeterList[EEPROM_METER_DB_MAXCOUNT];
}MeterDataBase_t;

#pragma pack(1)
/**
* @brief  Configuration word field definition
*/
typedef struct 
{
  uint16_t BidirComm : 1;
  uint16_t Accessibility : 1;
  uint16_t Synchronous : 1;
  uint16_t Reserved : 1;
  uint16_t Mode : 4;
  uint16_t NumEncrBlocks : 4;
  uint16_t MessageContent : 2;
  uint16_t RepeatedAccess : 1;
  uint16_t HopCounter : 1;
 
}ConfigWordField;

#pragma pack(1)
/**
* @brief  Application layer attributes
*/
typedef struct 
{
  uint8_t ELL_Len;
  uint32_t SerialNum;
  uint8_t CIField;
  uint8_t HeaderLength;
  uint8_t DevManufrID[2];
  uint8_t DevID[4];
  uint8_t DevVersion;
  uint8_t MeterDevType;
  uint8_t AccessNumber;
  uint8_t Status;
  ConfigWordField  CONFIG_WORD;
  uint8_t EncKey[ENCKEY_LEN];
  uint8_t HardwareRev;
  uint32_t FirmwareRev;
  uint32_t AckTimeout;
  uint8_t FrequentAccessCycle;
  ExtendedLinkLayer_t ELLType;
  uint8_t ELLAccessNum;
}WMBusAppliAttributes;




/**
* @}
*/

/** @defgroup user_appli_Exported_Constants  User Application Exported Constants
* @{
*/

/**
* @}
*/

/** @defgroup user_appli_Exported_Variables  User Application Exported Variables
* @{
*/
extern uint32_t __IO DOWNStatus, UPStatus, SELStatus, \
                     RIGHTStatus, LEFTStatus, KEYStatus;  
extern const uint8_t DefaultPayloadBuffer[];
extern uint8_t PayloadSize;
extern uint8_t PayloadBuffer[];
extern uint8_t AES_InitializationVector_Tx[];
extern uint8_t AES_InitializationVector_Rx[];
extern uint8_t AlarmReqFlag;
extern uint8_t DataReqFlag;
extern uint8_t ResetLinkFlag;
extern uint8_t SendData2Flag;
extern uint8_t SendDataFlag;
extern uint8_t CurrDataReqFlag;
extern uint8_t CurrAlarmReqFlag;
extern uint8_t CurrSendData2Flag;
extern uint8_t ConfirmInstallFlag;
extern MeterDataBase_t MeterDatabase;
extern WMBusAppliAttributes  AppliAttr;
extern uint8_t DeviceType;
extern DeviceInfo_t NewDevInfo; 
extern CfgParam_Typedef CurrCfg,DefaultCfg;
extern uint8_t Payload_Size;
extern uint8_t WorkingMode ;
extern uint8_t g_cmd_received;
extern uint8_t a_field[6];
extern uint8_t m_field[2];
extern app_mode_t g_app_mode ;
extern fifo_t*    g_p_uart_rx_fifo;
//extern timer_t  g_rf_tx_timer ;
//extern timer_t  g_tx_rx_led_timer;
extern uint8_t UserDataBlock[];
/**
* @}
*/

/** @defgroup user_appli_Exported_Functions  User Application Exported Functions
* @{
*/
void Set_SELStatus(void);
void Set_UPStatus(void);
void Set_DOWNStatus(void);
void SetLeftKeyStatus(void);
void Set_RightStatus(void);
void Set_KeyStatus(void);
void SetCurrCmdFlagOther(OtherCmdFlag currCmdflag);
void SetNextCmdFlagOther(OtherCmdFlag nextCmdflag);
void WMBUS_AppliAttrInit(uint8_t isDefault);
void WMBus_LinkSetAppliParams(void);
uint8_t EncryptMeterData(uint8_t *txPayloadSize);
uint8_t EncryptConcData(uint8_t *txPayloadSize);
uint8_t DecryptConcData(Frame_t * frame);
uint8_t DecryptMeterData(Frame_t * frame);
APPLI_STATUS WMBus_AppliSetAttribute(APPLI_ATTR_ID attr, uint32_t value);
APPLI_STATUS WMBus_AppliGetAttribute(APPLI_ATTR_ID attr, uint32_t* value);
void WriteNewMeterDataBaseToEEPROM(void);  
void WriteCurrentCfgToEEPROM(void);
void WriteDefaultCfgToEEPROM(void);  
void ReadCurrentCfgFromEEPROM(CfgParam_Typedef *Config) ;
void ReadDefaultCfgFromEEPROM(CfgParam_Typedef *defaultCfg) ;
void ManageCfgFromEEPROM(void);
void OverWriteCurrCfgToEEPROM(CfgParam_Typedef tempCfg);
void GetVersion(void);
void SetCmdToSend(uint8_t );
void SetCmdResponseFlow(uint8_t* );
void SetAppliData(uint8_t CmdID, uint8_t* pdata, uint8_t dataLen);
void SortArray(uint32_t *ptr,uint16_t NumElement);
void irq_debounce(void);
void app_tx_rx_led_off(void);
void app_tx_rx_led_on(void);
void app_init_txrx_led(void);
void app_mtc_mode(void);
void SnifferModeHandling(void);
void app_rf_tx_delay_timer_fn(void);
void app_command_mode(void);
void TestFunct(void);
void app_process_uart_rx_fifo(void);
void app_process_rf_tx_data(void);
void app_process_rf_rx_data(void);
void app_clear_command_buffer(void);
void app_process_shutdown(void);
void app_clear_command_buffer(void);
void app_clear_command_buffer(void);
void app_process_mode_switch(void);
void app_set_mode(app_mode_t mode);
void app_init_txrx_led(void);
void app_clear_command_buffer(void);
void app_tx_rx_led_timer_fn(void);
void app_operating_mode(void);
void app_send_response(const char* p_response);
void WriteDeviceSelfinfoToEEPROM(void);
void WriteMeterDatabaseToEEPROM(void);
void UpdateMeterDatabaseFromEEPROM(void);
void UpdateDeviceSelfInfoFromEEPROM(void);
uint8_t GetDBMeterIndex(Frame_t  *rxFrame);
void WriteDataBaseToEEPROM(void);
void WriteNewMeterDatabaseToEEPROM(DeviceInfo_t DevInfo,uint16_t meterNum);
void EraseDataBaseFromEEPROM(void);
void Set_DataBlock(void);
void LinkFrameReceived(Frame_t* frm );

#if defined (LPM_ENABLE)
void SystemPower_Config(void);
#endif

/**
* @}
*/

/**
*@}
*/

/**
* @}
*/

#endif /* __USER_APPLI_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
