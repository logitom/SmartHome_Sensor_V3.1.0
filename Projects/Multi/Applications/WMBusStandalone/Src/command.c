/******************************************************************************/
/**
 * @file    command.c
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
 * AT command processing functions.
 */

/*******************************************************************************
 * Include Files
 */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "command.h"
#include "radio_hal.h"
#include "uart.h"
#include "user_config.h"     
#include "user_appli.h"
#include "wmbus_phy.h"
#include "wmbus_link.h"
#include "wmbus_linkcommand.h"
   
/*******************************************************************************
 * Definitions
 */


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


#define CR                          0x0d    // Carriage return character


typedef struct
{
    char* p_name;
    void (*p_fn)(uint32_t param);
    uint32_t param;
} command_t;

/*******************************************************************************
 * Static Functions
 */
static void WMB_SaveCurrentCfg(uint32_t param);
//static void WMB_SelectDevice(uint32_t );
static void WMB_SetMode(uint32_t);
static void WMB_SetPacketFormat(uint32_t);
static void WMB_Update_Parameters(uint32_t );
static void WMB_SetDeviceIdentification(uint32_t param);
static void  WMB_SetOutputPower(uint32_t param);
static void  WMB_SetRSSIThreshold(uint32_t param);
//static void WMB_EncryptionEnable(uint32_t param);
static void WMB_SetEncryptionKey(uint32_t param);
static void WMB_GetEncryptionKey(uint32_t param);
static void WMB_GetEncryptionStatus(uint32_t param);
//static void  WMB_GetPower(uint32_t param);
static void  WMB_GetRSSIThreshold(uint32_t param);
static void WMB_GetPacketFormat(uint32_t param);
static void WMB_GetMode(uint32_t param);
//static void WMB_GetDeviceType(uint32_t param);
static void  WMB_GetOutputPower(uint32_t param);
static void WMB_GetDeviceIdentification(uint32_t param);
static void  WMB_GetRSSI(uint32_t param);
static void WMB_GetVersion(uint32_t param);
static void WMB_SetBaudRate(uint32_t param);
static void WMB_PerformSoftReset(uint32_t param);
static void WMB_LoadDefaultCfg(uint32_t param);
static void WMB_SetModuleFunctionality(uint32_t param);
static void WMB_GetCurrentChannel(uint32_t param);
static void WMB_GetModuleFunctionality(uint32_t param);
static void WMB_SetCurrentChannel(uint32_t param);
static void WMB_GetMeterDatabase(uint32_t param);

/*******************************************************************************
 * Global Variables
 */

uint8_t *g_p_cmd_str = 0;
uint8_t  g_cmd_len = 0;
uint8_t  g_escape_seq_en = 1;
uint32_t g_escape_wait = 0;
uint32_t g_escape_count = 0;
uint8_t     TempDataBuff[150];
//char SUPPORT_ERROR_MESSAGE[] = "\nNOT SUPPORTED\r\n";
/*******************************************************************************
 * Exported Variables
 */
extern uint8_t    g_cmd_received;
extern CfgParam_Typedef CurrCfg;
//extern uint32_t g_packets_received;
//extern uint32_t g_packets_discarded;
//extern uint32_t g_packet_crc_errors;
extern fifo_t*    g_p_uart_rx_fifo ;
extern Frame_t RxFrame;                                                          
extern OtherCmdFlag NxtCommandFlag;
extern OtherCmdFlag CurrCommandFlag;

/*******************************************************************************
 * Define the formats for the AT commands and set the command handler functions.
 */
static command_t g_commands[] =
{
  
 /***************   RAW_MODE Commands  ****************************/  
    
   /*****    WMBUS MODE and SNIFFER Mode Commands  ******/
    
    { "/OPMOD?\r",         WMB_GetModuleFunctionality,  0 },
    { "/WMBMOD=**\r",      WMB_SetMode,                 0 },
    { "/WMBMOD?\r",        WMB_GetMode,                 0 },
    { "/WMBFRM=**\r",      WMB_SetPacketFormat,         0 },
    { "/WMBFRM?\r",        WMB_GetPacketFormat,         0 },
    { "/WMBRSSITH=********\r", WMB_SetRSSIThreshold,    0 },
    { "/WMBRSSITH?\r",     WMB_GetRSSIThreshold,        0 },                                                                      
    { "/WMBRSSI?\r",       WMB_GetRSSI,                 0 },
    { "/WMBPOW=********\r",WMB_SetOutputPower,          0 },
    { "/WMBPOW?\r",        WMB_GetOutputPower,          0 },
    { "/WMBENC=**\r" ,     WMB_SetEncryption,           0 },
    { "/WMBENC?\r",        WMB_GetEncryptionStatus,     0 },
    
    { "/WMBENCKEY=********************************\r",    
                           WMB_SetEncryptionKey,        0 },
    { "/WMBENCKEY?\r",     WMB_GetEncryptionKey,        0 },
    { "/WMBDI=*******************\r",   WMB_SetDeviceIdentification, 0 },
    { "/WMBDI?\r",         WMB_GetDeviceIdentification, 0 },
    { "/WMBUPD\r",         WMB_Update_Parameters,       0 },
    { "/WMBSAVE\r",        WMB_SaveCurrentCfg,          0 },
    { "/WMBVER?\r",        WMB_GetVersion,               0},
    { "/WMBBAUD=**\r",     WMB_SetBaudRate,              0},
    { "/WMBRESET\r",       WMB_PerformSoftReset,         0},
    { "/WMBDEFAULT\r",     WMB_LoadDefaultCfg,           0},
    { "/OPMOD=**\r" ,      WMB_SetModuleFunctionality,   0},
    { "/WMBNXTFLAG=**\r" , WMB_SetNextCommandFlag,       0},
    { "/WMBCRNTFLAG=**\r" ,WMB_SetCurrentCommandFlag,    0},
    { "/WMBHWID?\r" ,      WMB_GetHardwareID,            0},
    { "/WMBFWID?\r" ,      WMB_GetFirmwareID,            0},
    { "/WMBCH=**\r" ,      WMB_SetCurrentChannel,        0},
    { "/WMBCH?\r" ,        WMB_GetCurrentChannel,        0},
    { "/WMBMETERDB?\r",    WMB_GetMeterDatabase,         0},
    { "/WMBSNDMSG=**\r",   WMB_SendMessage,              0},
    { 0, 0 } 
   /***************************************************************/    
};


/**
 * @brief Process an AT command.
 *
 * This function parses data received on the UART interface and process any
 * AT commands received.  When an AT command is received, the associated command
 * handler function is call to process the command.
 *
 * @param p_cmd  Pointer to the AT command string.
 * @param len  Length of the AT command string.
 */
void cmd_process(uint8_t *p_cmd, uint8_t len)
{
  if((CurrCfg.ModuleFunction == WMBUS_FUNCT)||
     (CurrCfg.ModuleFunction == SNIFFER_FUNCT))
  {
    static uint8_t cmd = 0;
    static uint8_t error = 0;
    
    g_p_cmd_str = p_cmd;
    g_cmd_len = len;
    
    if (g_cmd_len > 0)
    {
      if (g_p_cmd_str[0] == CR)
      {
        app_send_response("READY\n\r");
        app_clear_command_buffer();
        fifo_init(g_p_uart_rx_fifo);
        g_cmd_received = 0;
        cmd = 0;
      }
      else
      {
        if (g_p_cmd_str[g_cmd_len - 1] == CR)
        {
          error = 1;
          
          if ((p_cmd[0] == 'A') &&
              (p_cmd[1] == 'T'))
          {
            
            if(g_commands[cmd].p_name)
            {
              char* p_name = g_commands[cmd].p_name;
              uint32_t name_len = strlen(p_name) + 2;
              error = 0;
              
              for (uint8_t i = 2; i < name_len; i++)
              {
                if (*p_name != '*')
                {
                  if (*p_name == 0)
                  {
                    break;
                  }
                  
                  if (*p_name != g_p_cmd_str[i])
                  {
                    error = 1;
                    cmd++;                    
                    break;
                  }
                }
                
                p_name++;
              }
              
              if (error == 0)
              {
                g_commands[cmd].p_fn(g_commands[cmd].param);
                app_clear_command_buffer();
                fifo_init(g_p_uart_rx_fifo);
                cmd = 0;                              
                error = 0;                                                             
                g_cmd_received = 0;  
              }
            }
            
            
          }
          
          else     
          {
            app_clear_command_buffer();
            fifo_init(g_p_uart_rx_fifo);
            error = 0;                                                
            g_cmd_received = 0;
            cmd = 0;
          }
        }
      }
    }
    
    // An error occurs if the command was invalid or unknown
    if (error)
    {
      if(g_commands[cmd].p_name == 0)
      {
        
        app_clear_command_buffer();
        fifo_init(g_p_uart_rx_fifo);
        error = 0;                                                    
        g_cmd_received = 0;
        
        if(cmd == (MAX_COMMAND))
        {
          app_clear_command_buffer();
          app_send_response(RESPONSE_ERROR);
          cmd = 0;//WMBUS_CMD_OFFSET;                                        
        }
      }
    }
  }
  
  
  
  if(CurrCfg.ModuleFunction == RAW_FUNCT)
  {
    uint8_t error = 0;
    
    g_p_cmd_str = p_cmd;
    g_cmd_len = len;
    
    if (g_cmd_len > 0)
    {
      if (g_p_cmd_str[0] == CR)
      {
        app_send_response("READY\n\r");
        app_clear_command_buffer();
      }
      else
      {
        if (g_p_cmd_str[g_cmd_len - 1] == CR)
        {
          error = 1;
          
          if ((p_cmd[0] == 'A') &&
              (p_cmd[1] == 'T'))
          {
            uint8_t cmd = 0;
            
            while(g_commands[cmd].p_name)
            {
              char* p_name = g_commands[cmd].p_name;
              uint32_t name_len = strlen(p_name) + 2;
              error = 0;
              
              for (uint8_t i = 2; i < name_len; i++)
              {
                if (*p_name != '*')
                {
                  if (*p_name == 0)
                  {
                    break;
                  }
                  
                  if (*p_name != g_p_cmd_str[i])
                  {
                    error = 1;
                    break;
                  }
                }
                
                p_name++;
              }
              
              if (error == 0)
              {
                g_commands[cmd].p_fn(g_commands[cmd].param);
                app_clear_command_buffer();
                break;
              }
              
              cmd++;
            }
          }
        }
      }
    }
    
    // An error occurs if the command was invalid or unknown
    if (error)
    {
      app_send_response(RESPONSE_ERROR);
      app_clear_command_buffer();
    }
  }
  
}


/**
 * @brief This function sends the device type to PC.
 * This function is used when parsing values from AT commands.
 * @param  param .
 * @return None.
 */
static void WMB_GetModuleFunctionality(uint32_t param)
{
  uint8_t devType = 0;
  switch(CurrCfg.DevType)
  {
  case 0:                        /**OTHER**/
    devType = 0x00;
    break;
    
  case 1:                        /**METER**/
    devType = 0x01;
    break;
    
  case 2:                       /**ROUTER**/
    devType = 0x02;
    break;
    
  case 3:                       /**SNIFFER**/
    devType = 0x03;
    break;
    
  case 4:                       /**RAW MODE**/
    devType = 0x04;
    break;
    
  default:                      /**NOT CONFIGURED**/
    devType = 0xFF;
    break;  
  }
  
  TempDataBuff[0] = '\n';
  
  HexToText(&devType,(uint8_t *)&TempDataBuff[1],1);

  TempDataBuff[3] = '\n';
  
  uart_sendDMA((uint8_t*)&TempDataBuff,4);
     
}


/**
 * @brief This function set the WMBUS mode received from PC.
 * This function is used when parsing values from AT commands.
 * @param  param .
 * @return None.
 */
static void WMB_SetMode(uint32_t param)
{
  uint16_t TempMod;
  
  TempMod  = ((g_p_cmd_str[11] -  SubstractVal(g_p_cmd_str[11]))|
             ((g_p_cmd_str[10] - SubstractVal(g_p_cmd_str[10]))<<4));
  
  switch(TempMod)
  {
    
  case 0:
    CurrCfg.WorkingMode =  S1_MODE;
    break;
    
  case 1:
    CurrCfg.WorkingMode =  S1m_MODE;
    break;

  case 2:
    CurrCfg.WorkingMode = S2_MODE;
    break;
    
  case 3:
    CurrCfg.WorkingMode = T1_MODE;
    break;
    
  case 4:
    CurrCfg.WorkingMode = T2_MODE;
    break;
    
  case 5:
    CurrCfg.WorkingMode = R2_MODE;
    break;
    
  case 6:
    CurrCfg.WorkingMode = ALL_MODE;
    break;
    
  case 7:
    CurrCfg.WorkingMode = N1_MODE;
    break;
    
  case 8:
    CurrCfg.WorkingMode = N2_MODE;
    break;
    
  case 9:
    CurrCfg.WorkingMode = F2_MODE;
    break;
    
  case 0x0A:
    CurrCfg.WorkingMode = F2m_MODE;
    break;
    
  case 0x0B:
    CurrCfg.WorkingMode = C1_MODE;
    break;
    
  case 0x0C:
    CurrCfg.WorkingMode = C2_MODE;
    break;
    
   default:
    CurrCfg.WorkingMode = INVALID_MODE;
    break;
      
  }
  
  Radio_WMBus_Init();
  
  app_send_response(RESPONSE_OK);
  
}

/**
 * @brief This function sends the WMBUS mode to PC.
 * This function is used when parsing values from AT commands.
 * @param  param .
 * @return None.
 */
static void WMB_GetMode(uint32_t param)
{ 
  uint8_t mod = 0;
  
  switch(CurrCfg.WorkingMode)
  {
  case 0:                       /**S1 MODE**/
    mod = 0;
    break;
    
  case 1:                       /**S1m MODE**/
    mod = 1;
    break;
    
  case 2:                       /**S2 MODE**/
    mod = 2;
    break;
    
  case 3:                       /**T1 MODE**/
    mod = 3;
    break;
    
  case 4:                       /**T2 MODE**/
    mod = 4;
    break;
    
  case 5:                       /**R2 MODE**/
    mod = 5;
    break;
    
  case 6:                       /**ALL MODE**/
    mod = 6;
    break;
    
  case 7:                       /**N1 MODE**/
    mod = 7;
    break;
    
  case 8:                       /**N2 MODE**/
    mod = 8;
    break;
    
  case 9:                       /**F2 MODE**/
    mod = 9;
    break;
    
  case 0x0A:                    /**F2m MODE**/
    mod = 0x0A;
    break;
    
  case 0x0B:                    /**C1 MODE**/
    mod = 0x0B;
    break;
    
  case 0x0C:                    /**C2 MODE**/
    mod = 0x0C;
    break;
    
   default:                     /**INVALID MODE**/
    mod = 0xFF;
    break;
      
  }
  
  TempDataBuff[0] = '\n';
  
  HexToText(&mod,(uint8_t *)&TempDataBuff[1],1);

  TempDataBuff[3] = '\n';
  
  uart_sendDMA((uint8_t*)&TempDataBuff,4);

}


/**
 * @brief This function sets the WMBUS packet format.
 * This function is used when parsing values from AT commands.
 * @param  param .
 * @return None.
 */
static void WMB_SetPacketFormat(uint32_t param)
{
  uint16_t tempFrame;
  
  tempFrame  = ((g_p_cmd_str[10] - SubstractVal(g_p_cmd_str[10]))| 
               ((g_p_cmd_str[9] -  SubstractVal(g_p_cmd_str[9]))<<4));
  
  switch(tempFrame)
  {
  case 0:
    CurrCfg.FrameFormat = FRAME_FORMAT_A;
    break;
    
  case 1:
    CurrCfg.FrameFormat = FRAME_FORMAT_B;
    break;
    
  default :
    CurrCfg.FrameFormat = FRAME_FORMAT_A;
    break;
    
  }
  
  app_send_response(RESPONSE_OK);
  
}

/**
 * @brief This function Gets the Meter Databse.
 * This function is used when parsing values from AT commands.
 * @param  param .
 * @return None.
 */
static void WMB_GetMeterDatabase(uint32_t param)
{
  uart_sendDMA((uint8_t *)SUPPORT_ERROR_MESSAGE, strlen(SUPPORT_ERROR_MESSAGE));
}

/**
 * @brief This function gets the WMBUS packet format.
 * This function is used when parsing values from AT commands.
 * @param  param .
 * @return None.
 */
static void WMB_GetPacketFormat(uint32_t param)
{
  uint8_t pktFormat = 0; 

  switch(CurrCfg.FrameFormat)
  {
  case 0:
    pktFormat = 0;
    break;
    
  case 1:
     pktFormat = 1;
    break;
    
  default :
    pktFormat = 0;
    break;
    
  }
  
  TempDataBuff[0] = '\n';
  
  HexToText(&pktFormat,(uint8_t *)&TempDataBuff[1],1);

  TempDataBuff[3] = '\n';
  
  uart_sendDMA((uint8_t*)&TempDataBuff,4);
}


/**
 * @brief This function sets the RSSI Threshold.
 * This function is used when parsing values from AT commands.
 * @param  param.
 * @return None.
 */
void  WMB_SetRSSIThreshold(uint32_t param)
{
  uint32_t temp;  
  
  temp = ((g_p_cmd_str[13] - SubstractVal(g_p_cmd_str[13]))<<28) |
         ((g_p_cmd_str[14] - SubstractVal(g_p_cmd_str[14]))<<24) |
         ((g_p_cmd_str[15] - SubstractVal(g_p_cmd_str[15]))<<20) |
         ((g_p_cmd_str[16] - SubstractVal(g_p_cmd_str[16]))<<16) |
         ((g_p_cmd_str[17] - SubstractVal(g_p_cmd_str[17]))<<12) |
         ((g_p_cmd_str[18] - SubstractVal(g_p_cmd_str[18]))<<8)  |
         ((g_p_cmd_str[19] - SubstractVal(g_p_cmd_str[19]))<<4)  |
         ((g_p_cmd_str[20] - SubstractVal(g_p_cmd_str[20])));
    
  WMBus_PhySetAttr(WMBUS_PHY_RX_RSSI_THRESHOLD, temp);
  app_send_response(RESPONSE_OK);
}

/**
 * @brief This function gets the RSSI Threshold of WMBUS.
 * This function is used when parsing values from AT commands.
 * @param  param.
 * @return None.
 */
static void  WMB_GetRSSIThreshold(uint32_t param)
{
  uint8_t revArr[4]; 
  uint8_t temVal[4];
  
  WMBus_PhyGetAttr(WMBUS_PHY_RX_RSSI_THRESHOLD,(int32_t *)&temVal[0]);
  
  for(uint8_t cnt = 0;cnt<4;cnt++)
    revArr[cnt] = temVal[3-cnt];
  
  TempDataBuff[0] = '\n';
  
  HexToText((uint8_t*)&revArr[0],(uint8_t *)&TempDataBuff[1],4);
  
  TempDataBuff[9] = '\n';
  
  uart_sendDMA((uint8_t*)&TempDataBuff,10);
}

/**
 * @brief This function gets the RSSI value used in firmware WMBUS.
 * This function is used when parsing values from AT commands.
 * @param  param.
 * @return None.
 */
static void  WMB_GetRSSI(uint32_t param)
{
  uint8_t revArr[4];
  uint8_t temVal[4];
  
  memcpy(temVal ,(uint8_t *)&RxFrame.RSSI_field,4);
  
  for(uint8_t cnt = 0;cnt<4;cnt++)
    revArr[cnt] = temVal[3-cnt];
  
  TempDataBuff[0] = '\n';
  
  HexToText((uint8_t*)&revArr[0],(uint8_t *)&TempDataBuff[1],4);
  
  TempDataBuff[9] = '\n';
  
  uart_sendDMA((uint8_t*)&TempDataBuff,10);
  
}

/**
 * @brief This function sets the output power.
 * This function is used when parsing values from AT commands.
 * @param  param.
 * @return None.
 */
void  WMB_SetOutputPower(uint32_t param)
{
  uint32_t temp;
  
  temp = ((g_p_cmd_str[10]-SubstractVal(g_p_cmd_str[10]))<<28)|
         ((g_p_cmd_str[11]-SubstractVal(g_p_cmd_str[11]))<<24)|
         ((g_p_cmd_str[12]-SubstractVal(g_p_cmd_str[12]))<<20)|
         ((g_p_cmd_str[13]-SubstractVal(g_p_cmd_str[13]))<<16)|
         ((g_p_cmd_str[14]-SubstractVal(g_p_cmd_str[14]))<<12)|
         ((g_p_cmd_str[15]-SubstractVal(g_p_cmd_str[15]))<<8) |
         ((g_p_cmd_str[16]-SubstractVal(g_p_cmd_str[16]))<<4) |
         (g_p_cmd_str[17]-SubstractVal(g_p_cmd_str[17]));
  
  WMBus_PhySetAttr(WMBUS_PHY_OP_POWER , temp);
  app_send_response(RESPONSE_OK);
}

/**
 * @brief This function gets the output power.
 * This function is used when parsing values from AT commands.
 * @param  param.
 * @return None.
 */
static void  WMB_GetOutputPower(uint32_t param)
{
 
  uint8_t temVal[4],revArr[4];
  
  WMBus_PhyGetAttr(WMBUS_PHY_OP_POWER, (int32_t *)&temVal[0]);
  
  for(uint8_t cnt = 0;cnt<4;cnt++)
    revArr[cnt] = temVal[3-cnt];  
  
  TempDataBuff[0] = '\n';
  
  HexToText((uint8_t*)&revArr[0],(uint8_t *)&TempDataBuff[1],4);
  
  TempDataBuff[9] = '\n';
  
  uart_sendDMA((uint8_t*)&TempDataBuff,10);
}


/**
 * @brief This function update the user configuration parameters.
 * This function is used when parsing values from AT commands.
 * @param  param.
 * @return None.
 */
static void WMB_Update_Parameters(uint32_t param)
{
  Radio_WMBus_Init();
  app_send_response(RESPONSE_OK);
}

/**
 * @brief This function returns proper value to be substracted while converting the 
 *        ASCII to Hex.
 * This function is used when parsing values from AT commands.
 * @param  input: The ASCII Value.
 * @return The Value to be substracted.
 */
uint8_t SubstractVal(uint8_t input)
{
  uint8_t retVal = 0;
  
  if((input >= 0x30) && (input <= 0x39))
  { 
    retVal = '0'; 
  }
  
  if((input >=0x41) && (input <=0x46))
  { 
    retVal = 0x37; 
  }
  
  return retVal;
}


/**
 * @brief This function sets the device identification.
 * This function is used when parsing values from AT commands.
 * @param  param.
 * @return None.
 */
void WMB_SetDeviceIdentification(uint32_t param)
{
  uint8_t  l_MfrID[2];
  uint8_t  l_IDNum[4];
  uint8_t  l_Version;
  uint8_t  l_MeterType;
   
  l_MfrID[0] = (g_p_cmd_str[10] - SubstractVal(g_p_cmd_str[10])) | 
                ((g_p_cmd_str[9] - SubstractVal(g_p_cmd_str[9]))<<4);
  l_MfrID[1] = (g_p_cmd_str[12] - SubstractVal(g_p_cmd_str[12])) |
                ((g_p_cmd_str[11] - SubstractVal(g_p_cmd_str[11]))<<4);
  
  l_IDNum[0] = (g_p_cmd_str[15] - SubstractVal(g_p_cmd_str[15])) |
                ((g_p_cmd_str[14] - SubstractVal(g_p_cmd_str[14]))<<4);
  l_IDNum[1] = (g_p_cmd_str[17] - SubstractVal(g_p_cmd_str[17])) |
                ((g_p_cmd_str[16] - SubstractVal(g_p_cmd_str[16]))<<4);
  l_IDNum[2] = (g_p_cmd_str[19] - SubstractVal(g_p_cmd_str[19])) |
                ((g_p_cmd_str[18] - SubstractVal(g_p_cmd_str[18]))<<4);
  l_IDNum[3] = (g_p_cmd_str[21] - SubstractVal(g_p_cmd_str[21])) |
                ((g_p_cmd_str[20] - SubstractVal(g_p_cmd_str[20]))<<4);
  
  l_Version =  (g_p_cmd_str[24]-SubstractVal(g_p_cmd_str[24]))|
                ((g_p_cmd_str[23]-SubstractVal(g_p_cmd_str[23]))<<4);
  
  l_MeterType = (g_p_cmd_str[27]-SubstractVal(g_p_cmd_str[27]))|
                ((g_p_cmd_str[26]-SubstractVal(g_p_cmd_str[26]))<<4);
  
  
  memcpy(&CurrCfg.DevInfo.MfrID,l_MfrID,2);
  memcpy(&CurrCfg.DevInfo.IDNum,l_IDNum,4);
  
  CurrCfg.DevInfo.Version = l_Version;
  CurrCfg.DevInfo.MeterType = l_MeterType; 

  Radio_WMBus_Init();
  
  app_send_response(RESPONSE_OK);
  
}

/**
 * @brief This function Sets the wM-BUS Next Flag at Concentrator Side.
 * This function is used when parsing values from AT commands.
 * @param  param.
 * @return None.
 */
void WMB_SetNextCommandFlag(uint32_t param )
{
  uint16_t tempData;
  
  tempData  = ((g_p_cmd_str[15] - SubstractVal(g_p_cmd_str[15])) |
              ((g_p_cmd_str[14] - SubstractVal(g_p_cmd_str[14]))<<4));
  
  switch(tempData)
  {
  case 0:
    NxtCommandFlag = DATA_FLAG;
    SetNextCmdFlagOther(NxtCommandFlag);
    break;
    
  case 1:
    NxtCommandFlag = ALARM_FLAG;
    SetNextCmdFlagOther(NxtCommandFlag);
    break;
    
  case 2:
    NxtCommandFlag = SND_UD_FLAG;
    SetNextCmdFlagOther(NxtCommandFlag);
    break;
    
  case 3:
    NxtCommandFlag = SND_UD2_FLAG;
    SetNextCmdFlagOther(NxtCommandFlag);
    break;
    
  case 4:
    NxtCommandFlag = RESET_LINK_FLAG;
    SetNextCmdFlagOther(NxtCommandFlag);
    break;
    
  case 5:
    NxtCommandFlag = RESET_ALL_FLAG;
    SetNextCmdFlagOther(NxtCommandFlag);
    break;
  } 
  
   app_send_response(RESPONSE_OK);
  
}

/**
 * @brief This function Sets the wM-BUS current Flag at Concentrator Side.
 * This function is used when parsing values from AT commands.
 * @param  param.
 * @return None.
 */
void WMB_SetCurrentCommandFlag(uint32_t param )
{
  uint16_t tempData;
  
  tempData  = ((g_p_cmd_str[16] - SubstractVal(g_p_cmd_str[16])) |
              ((g_p_cmd_str[15] - SubstractVal(g_p_cmd_str[15]))<<4));
  
  switch(tempData)
  {
  case 0:
    CurrCommandFlag = DATA_FLAG;
    SetCurrCmdFlagOther(CurrCommandFlag);
    break;
    
  case 1:
    CurrCommandFlag = ALARM_FLAG;
    SetCurrCmdFlagOther(CurrCommandFlag);
    break;
    
  case 2:
    CurrCommandFlag = SND_UD2_FLAG;
    SetCurrCmdFlagOther(CurrCommandFlag);
    break;
  } 
  
   app_send_response(RESPONSE_OK);
}

/**
 * @brief This function gets the Encryption Status.
 * This function is used when parsing values from AT commands.
 * @param  param.
 * @return None.
 */
static void WMB_GetEncryptionStatus(uint32_t param)
{
  uint8_t encStatus = 0;
  
  if(((CurrCfg.EncrModeVar)&0x0F00) == 0x0500)
    encStatus = 1;
  else if(((CurrCfg.EncrModeVar)&0x0F00) == 0x0000)
    encStatus = 0;  
  
  TempDataBuff[0] = '\n';
  
  HexToText(&encStatus,(uint8_t *)&TempDataBuff[1],1);
  
  TempDataBuff[3] = '\n';
  
  uart_sendDMA((uint8_t*)&TempDataBuff,4);
}

/**
 * @brief This function sets the Encryption Key.
 * This function is used when parsing values from AT commands.
 * @param  param.
 * @return None.
 */
static void WMB_SetEncryptionKey(uint32_t param)
{
  if(TextToHex(&g_p_cmd_str[13],&CurrCfg.DevInfo.EncrKey[0],32))
  {
    app_send_response(COMMAND_ERROR);
  }
  else
  {
    memcpy(CurrCfg.DevInfo.EncrKey,&CurrCfg.DevInfo.EncrKey[0],ENCKEY_LEN); 
    app_send_response(RESPONSE_OK);
  }
}

/**
 * @brief This function gets the Encryption Key.
 * This function is used when parsing values from AT commands.
 * @param  param.
 * @return None.
 */
static void WMB_GetEncryptionKey(uint32_t param)
{
  TempDataBuff[0] = '\n';
  
  HexToText(&CurrCfg.DevInfo.EncrKey[0],(uint8_t *)&TempDataBuff[1],2*ENCKEY_LEN);
  
  TempDataBuff[33] = '\n';
  
  uart_sendDMA((uint8_t*)&TempDataBuff, 34);
}


/**
 * @brief This function Gets the device Identification Parameters.
 * This function is used when parsing values from AT commands.
 * @param  param.
 * @return None.
 */
static void WMB_GetDeviceIdentification(uint32_t param)
{
  HexToText(&CurrCfg.DevInfo.MfrID[0] ,(uint8_t *)&TempDataBuff[1],2);
  TempDataBuff[5] = 0x20;   /***0x20 = ASCII value of SPACE ***/
  
  HexToText(&CurrCfg.DevInfo.IDNum[0] ,(uint8_t *)&TempDataBuff[6],4);
  TempDataBuff[14] = 0x20;  /***0x20 = ASCII value of SPACE ***/
  
  HexToText(&CurrCfg.DevInfo.Version  ,(uint8_t *)&TempDataBuff[15],1);
  TempDataBuff[17] = 0x20; /***0x20 = ASCII value of SPACE ***/
  
  HexToText(&CurrCfg.DevInfo.MeterType,(uint8_t *)&TempDataBuff[18],1);
  TempDataBuff[0] = '\n';
  TempDataBuff[20] = '\n';
  
  uart_sendDMA((uint8_t*)&TempDataBuff, 21);
}


/**
 * @brief This function Gets the Current Channel.
 * This function is used when parsing values from AT commands.
 * @param  param.
 * @return None.
 */
static void WMB_GetCurrentChannel(uint32_t param)
{
  int32_t channelNum = 0;
  
  WMBus_PhyGetAttr(WMBUS_PHY_CHANNEL, &(channelNum));
  
  HexToText((uint8_t *)&channelNum,(uint8_t *)&TempDataBuff[1],1);//2);
  
  TempDataBuff[0] = '\n';
  TempDataBuff[3] = '\n';
  
  uart_sendDMA((uint8_t*)&TempDataBuff,4);
}

/**
 * @brief This function Sets the Current Channel.
 * This function is used when parsing values from AT commands.
 * @param  param.
 * @return None.
 */
static void WMB_SetCurrentChannel(uint32_t param)
{
  uint8_t temp;
  temp = ((g_p_cmd_str[10] - SubstractVal(g_p_cmd_str[10]) )|
          ((g_p_cmd_str[9] - SubstractVal(g_p_cmd_str[9]))<<4));
   
  
  if(temp<0x0F)
  { 
    CurrCfg.Channel = temp;
    
    /* Set Channel*/
    WMBus_PhySetAttr(WMBUS_PHY_CHANNEL, CurrCfg.Channel);
    app_send_response(RESPONSE_OK);
  }
  else
  {
     app_send_response(RESPONSE_ERROR_VALUE);
  }
  
}

/**
 * @brief This function gets the Hardware nad Software Version.
 * This function is used when parsing values from AT commands.
 * @param  param.
 * @return None.
 */
static void WMB_GetVersion(uint32_t param)
{
  GetVersion();
}

/**
 * @brief This Function saves current configuration in  the EEPROM,
    So that it can be loaded next time the module is RESET.
 * This function is used when parsing values from AT commands.
 * @param  param.
 * @return None.
 */
static void WMB_SaveCurrentCfg(uint32_t param)
{
  WriteCurrentCfgToEEPROM();
  
  WriteDeviceSelfinfoToEEPROM();
  
  app_send_response(RESPONSE_OK);
}


/**
 * @brief This Function Sets the New Baud Rate for UART.
 * This function is used when parsing values from AT commands.
 * @param  param.
 * @return None.
 */   
static void WMB_SetBaudRate(uint32_t param)
{
  uint16_t temp;
  uint32_t baudRate;

  temp = ((g_p_cmd_str[12] - SubstractVal(g_p_cmd_str[12]) )|
         ((g_p_cmd_str[11] - SubstractVal(g_p_cmd_str[11]))<<4));
         
    switch(temp)
    {
    case 0:
      baudRate = 600;
      break;
      
    case 1:
      baudRate = 1200;
      break;  
      
    case 2:
      baudRate = 2400;
      break;
      
    case 3:
      baudRate = 4800;
      break;
      
    case 4:
      baudRate = 9600;
      break;
      
    case 5:
      baudRate = 14400;
      break;
      
    case 6:
      baudRate = 19200;
      break;
      
    case 7:
      baudRate = 38400;
      break;
      
    case 8:
      baudRate = 56000;
      break;
      
    case 9:
      baudRate = 57600;
      break;
      
    case 10:
      baudRate = 115200;
      break;
      
    default:
      baudRate = 115200;
      break;

    }
       
    uart_init(baudRate);
}


/**
 * @brief This Function performs the software RESET.
 * This function is used when parsing values from AT commands.
 * @param  param.
 * @return None.
 */
static void WMB_PerformSoftReset(uint32_t param)
{
  NVIC_SystemReset(); //Performs the CPU RESET
}


/**
 * @brief This Function Sets the module Functionality.
 * This function is used when parsing values from AT commands.
 * @param  param.
 * @return None.
 */
static void WMB_SetModuleFunctionality(uint32_t param)
{
  uint16_t temp;
  
  temp = ((g_p_cmd_str[10] - SubstractVal(g_p_cmd_str[10]) )|
          ((g_p_cmd_str[9] - SubstractVal(g_p_cmd_str[9]))<<4));
    
  switch(temp)
  {
  
  case 0:  /***OTHER/CONCENTRATOR***/
    CurrCfg.ModuleFunction = WMBUS_FUNCT;
 
     if((CurrCfg.DevType != OTHER))
    {
      CurrCfg.DevType = OTHER;
      Radio_WMBus_Init();
    }
   app_send_response(RESPONSE_OK);
    
    break;

   case 1:  /**METER**/
    CurrCfg.ModuleFunction = WMBUS_FUNCT;
 
     if((CurrCfg.DevType != METER))
    {
      CurrCfg.DevType = METER;
      Radio_WMBus_Init();
    }
   app_send_response(RESPONSE_OK);
//      uart_sendDMA(SUPPORT_ERROR_MESSAGE, strlen(SUPPORT_ERROR_MESSAGE));
    break;
    
   case 2:  /**ROUTER**/
      uart_sendDMA((uint8_t *)SUPPORT_ERROR_MESSAGE,strlen(SUPPORT_ERROR_MESSAGE));
    break;

  case 3:   /**SNIFFER**/
    CurrCfg.ModuleFunction = SNIFFER_FUNCT;
    CurrCfg.DevType = SNIFFER;
    Radio_WMBus_Init();
    app_send_response(RESPONSE_OK);
    break;

  case 4:  /**RAW MODE: STEVAL-SP1ML868**/
//    CurrCfg.ModuleFunction = RAW_FUNCT;
    uart_sendDMA((uint8_t *)SUPPORT_ERROR_MESSAGE,strlen(SUPPORT_ERROR_MESSAGE));
    break;

  default:
    break;
  }
}

/**
 * @brief Load Default Configuration from EEPROM Database saved.
 * This function is used when parsing values from AT commands.
 * @param  param.
 * @return None.
 */
static void WMB_LoadDefaultCfg(uint32_t param)
{
  
//  CfgParam_Typedef tempCfg;
//
//  ReadDefaultCfgFromEEPROM(&tempCfg);
//  
//  CurrCfg = tempCfg;
//   
//  Radio_WMBus_Init();
  
}


/**
 * @brief This Function Enables/Disable Encryption in the f/w.
 * This function is used when parsing values from AT commands.
 * @param  param.
 * @return None.
 */
void WMB_SetEncryption(uint32_t param)
{
//  uint16_t temp;
//  
//  temp = ((g_p_cmd_str[11] - SubstractVal(g_p_cmd_str[11]) )|
//         ((g_p_cmd_str[10] - SubstractVal(g_p_cmd_str[10]))<<4));
//                                                                                          
//  if(temp == 0x01)
//  {
//    CurrCfg.EncrModeVar =  0xF5FF; //Mask for config word with Encryption
//    WMBus_AppliSetAttribute(APPLI_ATTR_CONFIG_WORD,
//                            (ENC_CONFIG_WORD&CurrCfg.EncrModeVar));
//    app_send_response(RESPONSE_OK);
//  }
//  else if(temp == 0x00) 
//  {
//    CurrCfg.EncrModeVar =  0xF0FF; //Mask for config word with No Encryption
//    WMBus_AppliSetAttribute(APPLI_ATTR_CONFIG_WORD, 
//                            (ENC_CONFIG_WORD&CurrCfg.EncrModeVar));
//    app_send_response(RESPONSE_OK);
//  }
//  else
//  {
//    app_send_response(COMMAND_ERROR);   
//  }
}

/**
* @brief This function Sends the Hardware ID to the PC.
* This function is used when parsing values from AT commands.
* @param  param.
* @return None.
 */
void WMB_GetHardwareID(uint32_t param)
{
   uart_sendDMA((uint8_t*)HARDWARE_ID,strlen(HARDWARE_ID));
}

/**
* @brief This function sends Firmware ID to the PC.
* This function is used when parsing values from AT commands.
* @param  param.
* @return None.
 */
void WMB_GetFirmwareID(uint32_t param)
{
    uart_sendDMA((uint8_t*)FIRMWARE_ID,strlen(FIRMWARE_ID));
}

/**
* @brief This function sends the wM-Bus message.
* This function is used when parsing values from AT commands.
* @param  param.
* @return None.
 */
void WMB_SendMessage(uint32_t param)
{
  uint16_t temp;

  temp = ((g_p_cmd_str[14] - SubstractVal(g_p_cmd_str[14]) )|
         ((g_p_cmd_str[13] - SubstractVal(g_p_cmd_str[13]))<<4));
  
  WMBus_SendCommand(temp);
  
}

/**
* @brief This function converts the TEXT STRING to HEX.
* This function is used when parsing values from AT commands.
* @param  input:Pointer to the input string.
* @param  output:Pointer to the Output.
* @param  textLen: The length of String.
* @return None.
 */
uint8_t TextToHex(uint8_t *input,uint8_t *output,uint8_t textLen)
{
  
  uint8_t temp,len;
  len = textLen/2;
  
  for(temp=0;temp<textLen;temp++)
  {
    if(input[temp]>=0x30 && input[temp]<=0x39)
      input[temp] -= 0x30;
      
   else if(input[temp]>=0x41 && input[temp]<=0x46)
      input[temp] -= 0x37;
   else
     return 1;
  }
    
  for(temp=0;temp<len;temp++)
  {
      output[temp] = input[2*temp]<<4 | input[2*temp + 1];
  }
  
    return 0;
 
}

/**
* @brief This function converts the HEX to STRING.
* This function is used when parsing values from AT commands.
* @param  input:Pointer to the input string.
* @param  output:Pointer to the Output.
* @param  textLen: The length of Hex(In Bytes).
* @return None.
*/
void HexToText(uint8_t *input,uint8_t *output,uint8_t len)
{
  
  uint8_t temp,index = 0;
  uint8_t val;
  
  for(temp=0;temp<len;temp++)
  {
    
    val = input[temp];
    
    output[2*index] =  (val & 0xF0)>>4; 

    if((output[2*index]<=9))
    {
      
       output[2*index] +=0x30;
      
    }
    else if ((output[2*index]>=0x0A) &&(output[2*index]<=0x0F))
    {
      output[2*index] +=0x37;
      
    }
    
     output[2*index + 1] =  (val & 0x0F); 
    
    if((output[2*index+1]<=9))
    {
      
       output[2*index+1] +=0x30;
      
    }
    else if ((output[2*index+1]>=0x0A) &&(output[2*index+1]<=0x0F))
    {
      output[2*index+1] +=0x37;
      
    }
    
    index++; 
  }  
}

/***************************************************************************/
