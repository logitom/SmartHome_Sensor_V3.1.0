/**
******************************************************************************
* @file    radio-driver.c
* @author  ST
* @version V3.0.0
* @date    12-December-2018
* @brief   Source file for SPIRIT1 and S2LP radio configuration/driver
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
*    may be used to endorse or promote products derived from this software
*    without specific prior written permission.
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
/*---------------------------------------------------------------------------*/
#include "radio-driver.h"
#include "radio-arch.h"
#include "contiki.h"
#include "net/mac/frame802154.h"
#include "net/netstack.h"
#include "net/packetbuf.h"
#include "net/rime/rimestats.h"
#include <stdio.h>
#include "st-lib.h"
#include "low-power.h"

/**
 * @defgroup ST_Radio
 * @ingroup STM32_Contiki_Library
 * @{
 */

/**
 * @addtogroup ST_Radio
 * @ingroup STM32_Contiki_Library
 * @{
 * @file Porting of the ST_Radio for the Contiki OS
 */

#ifdef USE_STM32L1XX_NUCLEO
#include "stm32l1xx.h"
#endif

#ifdef USE_STM32F4XX_NUCLEO
#include "stm32f4xx.h"
#endif

#if defined(X_NUCLEO_S2868A1) || defined(X_NUCLEO_S2915A1) 
#define S2LP_RADIO
#elif defined(X_NUCLEO_IDS01A4) || defined(X_NUCLEO_IDS01A5)
#define SPIRIT1_RADIO
#endif

#if defined DATA_PACKET_127_BYTES && defined S2LP_RADIO 
#error "DATA_PACKET_127_BYTES makes sense for SPIRIT1 only." 
#endif /*DATA_PACKET_127_BYTES && S2LP_RADIO */
/*---------------------------------------------------------------------------*/
#define XXX_ACK_WORKAROUND 0
/*---------------------------------------------------------------------------*/
#ifndef DEBUG
#define DEBUG 0
#endif
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
/*---------------------------------------------------------------------------*/
#define BUSYWAIT_UNTIL(cond, max_time)                                  \
  do {                                                                  \
    rtimer_clock_t t0;                                                  \
    t0 = RTIMER_NOW();                                                  \
    while(!(cond) && RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + (max_time)));   \
  } while(0)

/*---------------------------------------------------------------------------*/
#define CLEAR_TXBUF()           (radio_txbuf[0] = 0)
#define CLEAR_RXBUF()           (radio_rxbuf[0] = 0)
#define IS_TXBUF_EMPTY()        (radio_txbuf[0] == 0)
#define IS_RXBUF_EMPTY()        (radio_rxbuf[0] == 0)
#define IS_RXBUF_FULL()         (radio_rxbuf[0] != 0)
/*---------------------------------------------------------------------------*/
/* transceiver state. */
#define ON     0
#define OFF    1
/*---------------------------------------------------------------------------*/
static volatile unsigned int radio_on = OFF;
//This is to save the state of the radio: needed to ensure that radio state is aligned with ContikiMAC
static volatile unsigned int was_off = 0;
static volatile uint8_t receiving_packet = 0;
static packetbuf_attr_t last_rssi = 0 ;
static packetbuf_attr_t last_lqi = 0 ;
/*---------------------------------------------------------------------------*/
/**
* @brief GPIO structure fitting
*/
SGpioInit xGpioIRQ={
  Radio_GPIO_IRQ,
  Radio_GPIO_MODE_DIGITAL_OUTPUT_LP,
  Radio_GPIO_DIG_OUT_IRQ
};
/*---------------------------------------------------------------------------*/
/**
* @brief Radio structure fitting
*/
SRadioInit xRadioInit = {
#ifdef SPIRIT1_RADIO   
  XTAL_OFFSET_PPM,
#endif  
  BASE_FREQUENCY,
#ifdef SPIRIT1_RADIO  
  CHANNEL_SPACE,
  CHANNEL_NUMBER,
#endif  
  MODULATION_SELECT,
  DATARATE,
  FREQ_DEVIATION,
  BANDWIDTH
 };
/*---------------------------------------------------------------------------*/
/**
* @brief Packet Basic structure fitting
*/
#ifdef S2LP_RADIO
PktBasicInit xBasicInit={
  PREAMBLE_LENGTH,
  SYNC_LENGTH,
  SYNC_WORD,
  VARIABLE_LENGTH,
  EXTENDED_LENGTH_FIELD,
  CRC_MODE,
  EN_ADDRESS,
  EN_FEC,
  EN_WHITENING
};
#else //SPIRIT1_RADIO

PktBasicInit xBasicInit={
  PREAMBLE_LENGTH,
  SYNC_LENGTH,
  SYNC_WORD,
  LENGTH_TYPE,
  LENGTH_WIDTH,
#ifdef DATA_PACKET_127_BYTES  
  PKT_NO_CRC,
#else
  CRC_MODE,
#endif  
  CONTROL_LENGTH,
  EN_ADDRESS,
  EN_FEC,
  EN_WHITENING
};
#endif
/*---------------------------------------------------------------------------*/
/*Flags declarations*/
volatile FlagStatus rx_timeout=RESET;
volatile FlagStatus xTxDoneFlag=RESET;
/*---------------------------------------------------------------------------*/
uint8_t cThreholdRxFifoAF = 78; 
uint8_t cThreholdTxFifoAE = 24;
uint16_t nTxIndex, nResidualPcktLength, nPayloadLength=60;
uint8_t *radio_128_data_buffp;
uint16_t nRxIndex = 0;
uint8_t cRxDataLen = 0;
/*---------------------------------------------------------------------------*/
#ifdef CSMA_ENABLE
#define PERSISTENT_MODE_EN              S_DISABLE
#define CS_PERIOD                       CSMA_PERIOD_64TBIT
#define CS_TIMEOUT                      3
#define MAX_NB                          5
#define BU_COUNTER_SEED                 0xFA21
#define CU_PRESCALER                    32

// refer to radio config.h for RSSI Thresholds

  /* Radio CSMA config */
#  ifdef S2LP_RADIO
SCsmaInit
#  else /*!S2LP_RADIO*/
CsmaInit
#  endif /*S2LP_RADIO*/
 xCsmaInit={
  PERSISTENT_MODE_EN,
  CS_PERIOD,
  CS_TIMEOUT,
  MAX_NB,
  BU_COUNTER_SEED,
  CU_PRESCALER
};
#  ifdef S2LP_RADIO
SRssiInit xSRssiInit = {
  .cRssiFlt = 14,
  .xRssiMode = RSSI_STATIC_MODE,
  .cRssiThreshdBm = RSSI_TX_THRESHOLD
};
#  endif /*S2LP_RADIO*/
#endif /*CSMA_ENABLE*/
/*---------------------------------------------------------------------------*/
/* 
* The buffers which hold incoming data.
* The +1 because of the first byte, 
* which will contain the length of the packet.
*/
#ifdef S2LP_RADIO
static uint8_t radio_rxbuf[512];
static uint8_t radio_txbuf[512];
#else //SPIRIT1_RADIO
static uint8_t radio_rxbuf[MAX_PACKET_LEN+1];
static uint8_t radio_txbuf[MAX_PACKET_LEN+1-RADIO_MAX_FIFO_LEN];
#endif
/*---------------------------------------------------------------------------*/
#if XXX_ACK_WORKAROUND || NULLRDC_CONF_802154_AUTOACK
static int just_got_an_ack = 0; /* Interrupt callback just detected an ack */
#endif /* XXX_ACK_WORKAROUND */

#if NULLRDC_CONF_802154_AUTOACK
#define ACK_LEN 3
static int wants_an_ack = 0; /* The packet sent expects an ack */
//#define ACKPRINTF printf
#define ACKPRINTF(...)
#endif /* NULLRDC_CONF_802154_AUTOACK */
/*---------------------------------------------------------------------------*/
static int packet_is_prepared = 0;
/*---------------------------------------------------------------------------*/
PROCESS(subGHz_radio_process, "subGHz radio driver");
/*---------------------------------------------------------------------------*/
static int Radio_init(void);
static int Radio_prepare(const void *payload, unsigned short payload_len);
static int Radio_transmit(unsigned short payload_len);
static int Radio_send(const void *data, unsigned short len);
static int Radio_read(void *buf, unsigned short bufsize);
static int Radio_channel_clear(void);
static int Radio_receiving_packet(void);
static int Radio_pending_packet(void);
static int Radio_on(void);
static int Radio_off(void);
/*---------------------------------------------------------------------------*/
/* Radio Driver Structure as per Contiki definition                          */
const struct radio_driver subGHz_radio_driver =
{
  Radio_init,
  Radio_prepare,
  Radio_transmit,
  Radio_send,
  Radio_read,
  Radio_channel_clear,
  Radio_receiving_packet,
  Radio_pending_packet,
  Radio_on,
  Radio_off,  
  NULL,
  NULL,
  NULL,
  NULL
};
/*---------------------------------------------------------------------------*/
/**
  * @brief  radio_printstatus
  * 	prints to the UART the status of the radio
  * @param  none
  * @retval None
  */
void
radio_printstatus(void)
{
  int s = RADIO_STATUS();
  if(s == RADIO_STATE_STANDBY) 
  {
    printf("radio-driver: RADIO_STATE_STANDBY\n");
  } 
  else if(s == RADIO_STATE_SLEEP)
  {
    printf("radio-driver: RADIO_STATE_SLEEP\n");
  }
  else if(s == RADIO_STATE_READY) 
  {
    printf("radio-driver: RADIO_STATE_READY\n");
  }
  else if(s == RADIO_STATE_TX) 
  {
    printf("radio-driver: RADIO_STATE_TX\n");
  } 
  else if(s == RADIO_STATE_RX)
  {
    printf("radio-driver: RADIO_STATE_RX\n");
  } 
#if RADIO_USES_SNIFF_MODE
  else if(s == RADIO_STATE_SLEEP_NOFIFO)
  {
    printf("radio-driver: RADIO_STATE_SLEEP_NOFIFO\n");
  }
  else if(s ==RADIO_STATE_SYNTH_SETUP)
  {
	  printf("radio-driver: RADIO_STATE_SYNTH_SETUP\n");
  }
#endif /*RADIO_USES_SNIFF_MODE*/
  else 
  {
    printf("radio-driver: status: %X\n", s>>1);
  }
}
/*---------------------------------------------------------------------------*/
/* Strobe a command. The rationale for this is to clean up the messy legacy code. */
/**
  * @brief  radio_strobe
  * 	strobe a command to the radio
  * @param  uint8_t s
  * @retval None
  */
static void
radio_strobe(uint8_t s)
{
  st_lib_radio_cmd_strobe_command((st_lib_radio_Cmd)s);
}
/*---------------------------------------------------------------------------*/
/**
  * @brief  radio_set_ready_state
  * 	sets the state of the radio to READY
  * @param  none
  * @retval None
  */
void radio_set_ready_state(void)
{
  PRINTF("READY IN\n");

  st_lib_radio_irq_clear_status();
  RADIO_IRQ_DISABLE();

  if(RADIO_STATUS() == RADIO_STATE_STANDBY) 
  {
    RadioCmdStrobeReady();
    BUSYWAIT_UNTIL(RADIO_STATUS() == RADIO_STATE_READY, 5 * RTIMER_SECOND/1000);
  }
  else if(RADIO_STATUS() == RADIO_STATE_RX) 
  {
#if RADIO_USES_SNIFF_MODE
   RadioCmdStrobeSabort();
   RadioCmdStrobeReady();
#else /*!RADIO_USES_SNIFF_MODE*/
     RadioCmdStrobeSabort();
#endif /*RADIO_USES_SNIFF_MODE*/
    BUSYWAIT_UNTIL(RADIO_STATUS() == RADIO_STATE_READY, 5 * RTIMER_SECOND/1000);
    st_lib_radio_irq_clear_status();
  }
  else if(RADIO_STATUS() == RADIO_STATE_SLEEP)
  {
#if RADIO_USES_SNIFF_MODE
     S2LPCmdStrobeSabort();
#endif /*RADIO_USES_SNIFF_MODE*/
    RadioCmdStrobeReady();
    BUSYWAIT_UNTIL(RADIO_STATUS() == RADIO_STATE_READY, 5 * RTIMER_SECOND/1000);
  }
#if RADIO_USES_SNIFF_MODE
  else if(RADIO_STATUS() == RADIO_STATE_SLEEP_NOFIFO)
  {
    S2LPCmdStrobeSabort();
	RadioCmdStrobeReady();
    BUSYWAIT_UNTIL(RADIO_STATUS() == RADIO_STATE_READY, 5 * RTIMER_SECOND/1000);
  }
#endif /*RADIO_USES_SNIFF_MODE*/
  else
  {
	PRINTF("From %X\r\n", RADIO_STATUS()>>1);
	RadioCmdStrobeSabort();
    BUSYWAIT_UNTIL(0, 5 * RTIMER_SECOND/1000);
    RadioCmdStrobeReady();
    BUSYWAIT_UNTIL(RADIO_STATUS() == RADIO_STATE_READY, 5 * RTIMER_SECOND/1000);
  }
  RADIO_IRQ_ENABLE();

  //radio_printstatus() ;
  PRINTF("READY OUT\n");
}
/*---------------------------------------------------------------------------*/
/**
  * @brief  Radio_init
  * 	Initializes the radio
  * @param  none
  * @retval int result(0 == success)
  */
static int
Radio_init(void)
{ 
  PRINTF("RADIO INIT IN\n");

  /* Configure radio shut-down (SDN) pin and activate radio */
  st_lib_radio_gpio_init(RADIO_GPIO_SDN, RADIO_MODE_GPIO_OUT);
  st_lib_radio_spi_init();
  
#ifdef S2LP_RADIO
   EepromSpiInitialization();
#endif /*S2LP_RADIO*/
  
  /* Configures the Radio library */
  st_lib_radio_radio_set_xtal_frequency(XTAL_FREQUENCY);
  /* wake up to READY state */
  /* weirdly enough, this *should* actually *set* the pin, not clear it! The pins is declared as GPIO_pin13 == 0x2000 */
  /* Radio ON */
  st_lib_radio_enter_shutdown();
  st_lib_radio_exit_shutdown();

  /* wait minimum 1.5 ms to allow Radio a proper boot-up sequence */
  BUSYWAIT_UNTIL(0, 3 * RTIMER_SECOND/2000);

  /* Soft reset of core */  
  radio_strobe(RADIO_STROBE_SRES);  

#ifdef SPIRIT1_RADIO
  /* Configures the SPIRIT1 radio part */
  st_lib_radio_radio_init(&xRadioInit);
  st_lib_radio_radio_set_pa_level_dbm(0,POWER_DBM);
  st_lib_radio_radio_set_pa_level_max_index(0);
#endif /*SPIRIT1_RADIO*/

#ifdef S2LP_RADIO
  S2LPManagementIdentificationRFBoard(); 

  /* if the board has eeprom, we can compensate the offset calling S2LPManagementGetOffset
  (if eeprom is not present this fcn will return 0) */
  xRadioInit.lFrequencyBase = (uint32_t) BASE_FREQUENCY + S2LPManagementGetOffset();

  /* if needed this will set the range extender pins */
  S2LPManagementRangeExtInit();
    
   /* S2LP Radio config */   
  st_lib_radio_radio_init(&xRadioInit);
  
  S2LPRadioSetChannel(CHANNEL_NUMBER);
  S2LPRadioSetChannelSpace(CHANNEL_SPACE);

  /* S2LP Radio set power */
  if(!S2LPManagementGetRangeExtender())
  {
    /* if we haven't an external PA, use the library function */
    S2LPRadioSetPALeveldBm(POWER_INDEX,POWER_DBM);
  }
  else
  { 
    /* in case we are using the PA board, the S2LPRadioSetPALeveldBm will be not functioning because
       the output power is affected by the amplification of this external component.
        Set the raw register. */
    uint8_t paLevelValue=0x25; /* for example, this value will give 23dBm about */
    S2LPSpiWriteRegisters(PA_POWER8_ADDR, 1, &paLevelValue);
  }
   S2LPRadioSetPALevelMaxIndex(POWER_INDEX); 
    
#endif /*S2LP_RADIO*/
  
  /* Configures the Radio packet handler part*/
  st_lib_radio_pkt_basic_init(&xBasicInit);
  
#ifdef CSMA_ENABLE
  st_lib_radio_csma_init(&xCsmaInit);
  st_lib_radio_csma(S_ENABLE);
#  ifdef S2LP_RADIO
  S2LPRadioRssiInit(&xSRssiInit); 
#  else //SPIRIT Radio
  SpiritQiSetRssiThresholddBm(RSSI_TX_THRESHOLD); 
#  endif /*S2LP_RADIO*/
#endif /*CSMA_ENABLE*/

  /* Enable the following interrupt sources, routed to GPIO */
  st_lib_radio_irq_de_init(NULL);
  st_lib_radio_irq_clear_status();
  st_lib_radio_irq(TX_DATA_SENT, S_ENABLE);
  st_lib_radio_irq(RX_DATA_READY,S_ENABLE);

#if RADIO_USES_SNIFF_MODE
  st_lib_radio_irq(VALID_SYNC,S_DISABLE);//For SNIFF mode this gives problems
  st_lib_radio_irq(RX_DATA_DISC, S_DISABLE);//For SNIFF mode this gives problems
  st_lib_radio_irq(RX_TIMEOUT, S_DISABLE);
#else /*!RADIO_USES_SNIFF_MODE*/
  st_lib_radio_irq(VALID_SYNC,S_ENABLE);
  st_lib_radio_irq(RX_DATA_DISC, S_ENABLE);
#endif /*RADIO_USES_SNIFF_MODE*/


#ifdef DATA_PACKET_127_BYTES
  st_lib_radio_irq(RX_FIFO_ALMOST_FULL,S_ENABLE);
#else
  st_lib_radio_irq(RX_FIFO_ALMOST_FULL,S_DISABLE);
#endif /*DATA_PACKET_127_BYTES*/
  
#ifdef CSMA_ENABLE
  st_lib_radio_irq(MAX_BO_CCA_REACH , S_ENABLE);  
  st_lib_radio_csma(S_DISABLE);
#else
  st_lib_radio_irq(MAX_BO_CCA_REACH , S_DISABLE);
#endif /*CSMA_ENABLE*/

  st_lib_radio_irq(TX_FIFO_ERROR, S_ENABLE);
  st_lib_radio_irq(RX_FIFO_ERROR, S_ENABLE);

#if RADIO_USES_SNIFF_MODE
  SRssiInit xSRssiInit = {
      .cRssiFlt = 14,
      .xRssiMode = RSSI_STATIC_MODE,
      .cRssiThreshdBm = RSSI_TX_THRESHOLD
  };
  S2LPRadioRssiInit(&xSRssiInit);

  S2LPTimerSetWakeUpTimerUs(1000*MIN_PERIOD_WAKEUP_MS); //12 ms
  /* set the rx timeout */
  S2LPTimerSetRxTimerUs(1000*RX_TIMEOUT_MS); //30 ms

#  ifdef CSMA_ENABLE
  /* use SLEEP_B mode (mandatory for CSMA) */
  S2LPTimerSleepB(S_ENABLE);
#  else /*!CSMA_ENABLE*/
  /* use SLEEP_A mode (default) */
  S2LPTimerSleepB(S_DISABLE);
#  endif /*CSMA_ENABLE*/

  /* enable LDC mode, FAST RX TERM and start Rx */
  S2LPTimerLdcrMode(S_ENABLE);
  /* enable the fast rx timer */
  S2LpTimerFastRxTermTimer(S_ENABLE);
#else /*!RADIO_USES_SNIFF_MODE*/

#  ifndef CSMA_ENABLE
  st_lib_radio_qi_set_rssi_threshold_dbm(RSSI_RX_THRESHOLD);
#  endif  /*CSMA_ENABLE*/
  SET_INFINITE_RX_TIMEOUT();
  /* Configure Radio */
  st_lib_radio_radio_persisten_rx(S_ENABLE);

#endif /*RADIO_USES_SNIFF_MODE*/

#ifdef SPIRIT1_RADIO
  st_lib_radio_qi_set_sqi_threshold(SQI_TH_0);
  st_lib_radio_qi_sqi_check(S_ENABLE);
  st_lib_radio_timer_set_rx_timeout_stop_condition(SQI_ABOVE_THRESHOLD);
  st_lib_radio_radio_afc_freeze_on_sync(S_ENABLE);
#  ifdef DATA_PACKET_127_BYTES
    /* .. set the almost full threshold and configure the associated IRQ */
  st_lib_FifoSetAlmostFullThresholdRx(96-cThreholdRxFifoAF);
#  endif /*DATA_PACKET_127_BYTES*/

#endif /*SPIRIT1_RADIO*/
  
  CLEAR_RXBUF();
  CLEAR_TXBUF();
  
  /* Initializes the mcu pin as input, used for IRQ */
  st_lib_radio_gpio_init(RADIO_GPIO_IRQ, RADIO_MODE_EXTI_IN);
  
  /* Configure the radio to route the IRQ signal to its GPIO 3 */
  st_lib_radio_gpio_irq_init(&xGpioIRQ);

  /* uC IRQ enable */
  RadioGpioInterruptCmd(RADIO_GPIO_IRQ,3,0,ENABLE);

#if RADIO_USES_SNIFF_MODE
  /* the RX command triggers the LDC in fast RX termination mode */
  S2LPCmdStrobeRx();
  radio_on = ON;
#else
  radio_on = OFF;
#endif /*RADIO_USES_SNIFF_MODE*/

  process_start(&subGHz_radio_process, NULL);

  PRINTF("Radio init done\n");
  return 0;
}

/*---------------------------------------------------------------------------*/
/**
  * @brief  Radio_prepare
  * 	prepares the radio for transmission
  * @param  none
  * @retval int result(0 == success)
  */
static int
Radio_prepare(const void *payload, unsigned short payload_len)
{
  PRINTF("Radio: prepare %u\n", payload_len);
  radio_128_data_buffp =  (uint8_t *) payload; 
  nResidualPcktLength = payload_len;

  packet_is_prepared = 0;

  if(radio_on == OFF) {
	  Radio_on();
  	  was_off = 1;
  } else {
  	  was_off = 0;
  }

  /* Checks if the payload length is supported */
  if(payload_len > SICSLOWPAN_CONF_MAC_MAX_PAYLOAD) 
  {
	PRINTF("Payload len too big (> %d), error.\n", SICSLOWPAN_CONF_MAC_MAX_PAYLOAD);
    return RADIO_TX_ERR;
  }

  /* Should we delay for an ack? */
#if NULLRDC_CONF_802154_AUTOACK
  frame802154_t info154;
  wants_an_ack = 0;
  if(payload_len > ACK_LEN
      && frame802154_parse((char*)payload, payload_len, &info154) != 0) {
    if(info154.fcf.frame_type == FRAME802154_DATAFRAME
        && info154.fcf.ack_required != 0) {
      wants_an_ack = 1;
    }
  }
#endif /* NULLRDC_CONF_802154_AUTOACK */

  /* Sets the length of the packet to send */
  RADIO_IRQ_DISABLE();
  radio_strobe(RADIO_STROBE_FTX);
  st_lib_radio_pkt_basic_set_payload_length(payload_len);

  if(payload_len > MAX_PACKET_LEN) //For S2-LP we can't get here since SICSLOWPAN_CONF_MAC_MAX_PAYLOAD == MAX_PACKET_LEN
  {
	 PRINTF("Payload len bigger than MAX_PACKET_LEN (%d), using auto fifo reload.\n", MAX_PACKET_LEN);
    /* ... if yes transmit data using an AE IRQ and a FIFO reloading mechanism */
    /* set the almost empty threshold */
    st_lib_FifoSetAlmostEmptyThresholdTx(cThreholdTxFifoAE);
    st_lib_radio_irq(TX_FIFO_ALMOST_EMPTY , S_ENABLE);
    
    /* write the linear fifo with the first 96 bytes of payload */
    st_lib_radio_spi_write_linear_fifo(MAX_PACKET_LEN, (uint8_t *)payload);
    
    /* store the number of transmitted bytes */
    nTxIndex = MAX_PACKET_LEN;
    
    /* update the residual number of bytes to be transmitted */
    nResidualPcktLength -= MAX_PACKET_LEN;
  }
  else
  {
    nTxIndex = payload_len;
	st_lib_radio_spi_write_linear_fifo(payload_len, (uint8_t *)payload);
  }

  RADIO_IRQ_ENABLE();
  
  PRINTF("PREPARE OUT\n");

  packet_is_prepared = 1;
  return RADIO_TX_OK;
}
/*---------------------------------------------------------------------------*/
/**
  * @brief  Radio_transmit
  * 	transimts a packet with the Sub GHz radio
  * @param  unsigned short payload_len
  * @retval int result(0 == success)
  */
static int
Radio_transmit(unsigned short payload_len)
{ 
  /* This function blocks until the packet has been transmitted */

  PRINTF("TRANSMIT IN\n");
  if(!packet_is_prepared)
  {
	PRINTF("Radio TRANSMIT: ERROR, packet is NOT prepared.\n");
	return RADIO_TX_ERR;
  }

#ifdef CSMA_ENABLE
   xTxDoneFlag = RESET;  
#endif /*CSMA_ENABLE*/

  /* Stores the length of the packet to send */
  /* Others Radio_prepare will be in hold */
  radio_txbuf[0] = payload_len;
  
  /* Puts the Radio in TX state */
  receiving_packet = 0;

#if RADIO_USES_CONTIKIMAC
  radio_set_ready_state();
#endif /*RADIO_USES_CONTIKIMAC*/

#ifdef CSMA_ENABLE
  st_lib_radio_csma(S_ENABLE);
  st_lib_radio_qi_set_rssi_threshold_dbm(RSSI_TX_THRESHOLD);
#endif  /*CSMA_ENABLE*/

  st_lib_CmdStrobeTx();

#if (XXX_ACK_WORKAROUND || NULLRDC_CONF_802154_AUTOACK) 
  just_got_an_ack = 0;
#endif /* XXX_ACK_WORKAROUND */

#ifdef CSMA_ENABLE
  uint8_t cCsState;
    
  /* wait for TX done */
  while(!xTxDoneFlag) {
    //TODO: Check: st_lib_QiGetCs() should be called with the Radio in RX, but it is the case only at the beginning.
    cCsState = st_lib_QiGetCs();
    if(cCsState){
      while(!xTxDoneFlag);
    }
  }
  xTxDoneFlag = RESET;
    
  st_lib_radio_csma(S_DISABLE);
#  if !RADIO_USES_SNIFF_MODE
  st_lib_radio_qi_set_rssi_threshold_dbm(RSSI_RX_THRESHOLD); 
#  endif /*!RADIO_USES_SNIFF_MODE*/

#else /*!CSMA_ENABLE*/

  while(!xTxDoneFlag);
  xTxDoneFlag = RESET;
#endif /*CSMA_ENABLE*/

  /* Reset radio - needed for immediate RX of ack */
  if(!IS_TXBUF_EMPTY())
  {
	  CLEAR_TXBUF();
  }

  if(!IS_RXBUF_EMPTY())
  {
	  CLEAR_RXBUF();
  }

  RADIO_IRQ_DISABLE();
  st_lib_radio_irq_clear_status();

#if RADIO_USES_SNIFF_MODE
  S2LPCmdStrobeSleep();
  S2LPTimerLdcrMode(S_ENABLE);
  S2LpTimerFastRxTermTimer(S_ENABLE);
  S2LPCmdStrobeRx();
#else /*!RADIO_USES_SNIFF_MODE*/
  RadioCmdStrobeSabort();
  BUSYWAIT_UNTIL(0, RTIMER_SECOND/2500);
  RadioCmdStrobeReady();
  BUSYWAIT_UNTIL(RADIO_STATUS() == RADIO_STATE_READY, 1 * RTIMER_SECOND/1000);
  RadioCmdStrobeRx();
  BUSYWAIT_UNTIL(RADIO_STATUS() == RADIO_STATE_RX, 1 * RTIMER_SECOND/1000);
#endif /*RADIO_USES_SNIFF_MODE*/

  RADIO_IRQ_ENABLE();

#if XXX_ACK_WORKAROUND
  just_got_an_ack = 1;
#endif /* XXX_ACK_WORKAROUND */

#if NULLRDC_CONF_802154_AUTOACK
  if (wants_an_ack)
  {
    BUSYWAIT_UNTIL(just_got_an_ack, 2 * RTIMER_SECOND/1000);
    if(just_got_an_ack) 
    {
    } 
    else 
    {
      ACKPRINTF("debug_ack: no ack received\n");
    }
  }
#endif /* NULLRDC_CONF_802154_AUTOACK */

  PRINTF("TRANSMIT OUT\n");
  if(!IS_TXBUF_EMPTY())
  {
	  CLEAR_TXBUF();
  }

  packet_is_prepared = 0;
  clock_wait(1);
  if(was_off == 1) {
	  /*If the radio was OFF before preparing the packet, we must turn it
	   * OFF to ensure that the state of the radio is aligned with ContikiMAC */
	  Radio_off();
  }

  return RADIO_TX_OK;
}
/*---------------------------------------------------------------------------*/
static int Radio_send(const void *payload, unsigned short payload_len)
{
  PRINTF("Radio Send\r\n");
  if(radio_on == OFF) {
	Radio_on();
	was_off = 1;
  } else {
	was_off = 0;
  }
#if RADIO_USES_SNIFF_MODE
   S2LPTimerLdcrMode(S_DISABLE);
   S2LpTimerFastRxTermTimer(S_DISABLE);
#endif /*RADIO_USES_SNIFF_MODE*/
   radio_set_ready_state();

  if(Radio_prepare(payload, payload_len) == RADIO_TX_ERR) 
  {
#if RADIO_USES_SNIFF_MODE
    S2LPTimerLdcrMode(S_ENABLE);
    S2LpTimerFastRxTermTimer(S_ENABLE);
#endif /*RADIO_USES_SNIFF_MODE*/
	if(was_off == 1) {
	  Radio_off();
	}
	PRINTF("PREPARE FAILED\n");
    return RADIO_TX_ERR;
  } 
  return Radio_transmit(payload_len);
}
/*---------------------------------------------------------------------------*/
/**
  * @brief  Radio_read
  * 	reads a packet received with the subGHz radio
  * @param  void *buf, unsigned short bufsize
  * @retval int bufsize
  */
static int Radio_read(void *buf, unsigned short bufsize)
{  
  PRINTF("READ IN\n");

  /* Checks if the RX buffer is empty */
  if(IS_RXBUF_EMPTY()) 
  {
    RADIO_IRQ_DISABLE();
    CLEAR_RXBUF();
    RadioCmdStrobeSabort();
    BUSYWAIT_UNTIL(0, RTIMER_SECOND/2500);
    RadioCmdStrobeReady();
    BUSYWAIT_UNTIL(RADIO_STATUS() == RADIO_STATE_READY, 1 * RTIMER_SECOND/1000);
#if RADIO_USES_SNIFF_MODE
     S2LPCmdStrobeSleep();
#else /*!RADIO_USES_SNIFF_MODE*/
     RadioCmdStrobeRx();
     BUSYWAIT_UNTIL(RADIO_STATUS() == RADIO_STATE_RX, 1 * RTIMER_SECOND/1000);
#endif/*RADIO_USES_SNIFF_MODE*/
    PRINTF("READ OUT RX BUF EMPTY\n");
    RADIO_IRQ_ENABLE();
    return 0;
  }

  if(bufsize < radio_rxbuf[0]) 
  { 
    /* If buf has the correct size */
    PRINTF("TOO SMALL BUF\n");
    return 0;
  } 
  else 
  {
    /* Copies the packet received */
    memcpy(buf, radio_rxbuf + 1, radio_rxbuf[0]);

    packetbuf_set_attr(PACKETBUF_ATTR_RSSI, last_rssi);        
    packetbuf_set_attr(PACKETBUF_ATTR_LINK_QUALITY, last_lqi); 
    bufsize = radio_rxbuf[0];
    CLEAR_RXBUF();
    
    PRINTF("READ OUT: %d\n", bufsize);
    return bufsize;
  }  
}
/*---------------------------------------------------------------------------*/
/**
  * @brief  Radio_channel_clear
  * 	checks the channel
  * @param  none
  * @retval int result
  */
static int
Radio_channel_clear(void)
{
	float rssi_value;
	/* Local variable used to memorize the S2LP state */
	uint8_t radio_state = radio_on;

	PRINTF("CHANNEL CLEAR IN\n");

	if(radio_on == OFF)
	{
		/* Wakes up the Radio */
	    Radio_on();
	}
	rssi_value = st_lib_radio_qi_get_rssi_dbm_run();
	int ret = (rssi_value<RSSI_TX_THRESHOLD)?1:0;

	/* Puts the S2LP in its previous state */
	if(radio_state==OFF)
	{
		Radio_off();
	}
	return ret;
}
/*---------------------------------------------------------------------------*/
/**
  * @brief  Radio_receiving_packet
  * 	checks for receiving packet
  * @param  none
  * @retval int result
  */
static int
Radio_receiving_packet(void)
{
  return receiving_packet;
}
/*---------------------------------------------------------------------------*/
/**
  * @brief  Radio_receiving_packet
  * 	checks for pending packet
  * @param  none
  * @retval int result
  */
static int
Radio_pending_packet(void)
{
  PRINTF("PENDING PACKET\n");
  return !IS_RXBUF_EMPTY();
}
/*---------------------------------------------------------------------------*/
/**
  * @brief  Radio_off
  * 	turns off the Sub-GHz radio
  * @param  none
  * @retval int result(0 == success)
  */
static int
Radio_off(void)
{  
  PRINTF("Radio: ->off\n");
  if(radio_on == ON) 
  {
    /* Disables the mcu to get IRQ from the RADIO */
    RADIO_IRQ_DISABLE();

#if RADIO_USES_SNIFF_MODE
   S2LPTimerLdcrMode(S_DISABLE);
   S2LpTimerFastRxTermTimer(S_DISABLE);
   RadioCmdStrobeReady();
   RadioCmdStrobeRx();
#endif /*RADIO_USES_SNIFF_MODE*/

   /* first stop rx/tx */
    RadioCmdStrobeSabort();

    /* Clear any pending irqs */
    st_lib_radio_irq_clear_status();
#if RADIO_USES_SNIFF_MODE
    RadioCmdStrobeReady();
#endif /*RADIO_USES_SNIFF_MODE*/
    BUSYWAIT_UNTIL(RADIO_STATUS() == RADIO_STATE_READY, 5 * RTIMER_SECOND/1000);

    if(RADIO_STATUS() != RADIO_STATE_READY) 
    {
      PRINTF("Radio: failed off->ready\n");
      return 1;
    }
    /* Puts the Radio in STANDBY */
    RadioCmdStrobeStandby();
    BUSYWAIT_UNTIL(RADIO_STATUS() == RADIO_STATE_STANDBY, 3 * RTIMER_SECOND/1000);

    if(RADIO_STATUS() != RADIO_STATE_STANDBY) 
    {
      PRINTF("Radio: failed off->stdby\n");
      return 1;
    }

    radio_on = OFF;
    CLEAR_TXBUF();
    CLEAR_RXBUF(); 
  }

  PRINTF("Radio: off.\n");
  //radio_printstatus();
  return 0; 
}
/*---------------------------------------------------------------------------*/
/**
  * @brief  Radio_on
  * 	turns on the Sub-GHz radio
  * @param  none
  * @retval int result(0 == success)
  */
static int Radio_on(void)
{

  PRINTF("Radio: on\n");

#if RADIO_USES_SNIFF_MODE
  if(radio_on == OFF) {

	  if(RADIO_STATUS() == RADIO_STATE_STANDBY) {
		  PRINTF("Switching radio ON from StandBy\r\n");
		  S2LPTimerLdcrMode(S_ENABLE);
		  S2LpTimerFastRxTermTimer(S_ENABLE);
		  RadioCmdStrobeReady();
 	      BUSYWAIT_UNTIL(RADIO_STATUS() == RADIO_STATE_READY, 5 * RTIMER_SECOND/1000);
		  S2LPCmdStrobeRx();
	  } else {
		  S2LPCmdStrobeSleep();
	  }
	  radio_on = ON;
#  if DEBUG
	  radio_printstatus();
#  endif
  }
#else /*!RADIO_USES_SNIFF_MODE*/
  if(radio_on == OFF) 
  {
    RadioCmdStrobeSabort();

    RADIO_IRQ_DISABLE();
    /* ensure we are in READY state as we go from there to Rx */
    RadioCmdStrobeReady();

    BUSYWAIT_UNTIL(RADIO_STATUS() == RADIO_STATE_READY, 5 * RTIMER_SECOND/1000);
//    radio_printstatus();

    if(RADIO_STATUS() != RADIO_STATE_READY) {
      PRINTF("Radio: failed to turn on\n");
      radio_printstatus();
	  while(1);
      //return 1;
    }
 
    /* now we go to Rx */
     RadioCmdStrobeRx();
     BUSYWAIT_UNTIL(RADIO_STATUS() == RADIO_STATE_RX, 5 * RTIMER_SECOND/1000);
    if(RADIO_STATUS() != RADIO_STATE_RX) {
      PRINTF("Radio: failed to enter rx\n");
      radio_printstatus();
	  while(1);
      //return 1;
    }

    /* Enables the mcu to get IRQ from the radio */
    RADIO_IRQ_ENABLE();
    radio_on = ON;
  }
#endif /*RADIO_USES_SNIFF_MODE*/
  return 0;
}
/*---------------------------------------------------------------------------*/
static int interrupt_callback_in_progress = 0;
static int interrupt_callback_wants_poll = 0;

/**
  * @brief  main Radio Contiki PROCESS
  */
PROCESS_THREAD(subGHz_radio_process, ev, data)
{
  PROCESS_BEGIN();
  PRINTF("Radio: process started\n");
  
  while(1) {
    int len;

    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);    
    PRINTF("Radio: polled\n");

    packetbuf_clear();
    len = Radio_read(packetbuf_dataptr(), PACKETBUF_SIZE);

    if(len > 0) {

#if NULLRDC_CONF_802154_AUTOACK
      /* Check if the packet has an ACK request */
      frame802154_t info154;
      if(len > ACK_LEN &&
          frame802154_parse((char*)packetbuf_dataptr(), len, &info154) != 0) {
        if(info154.fcf.frame_type == FRAME802154_DATAFRAME &&
            info154.fcf.ack_required != 0 &&
            linkaddr_cmp((linkaddr_t *)&info154.dest_addr,
                         &linkaddr_node_addr)) {


#if !XXX_ACK_WORKAROUND
          /* Send an ACK packet */
          uint8_t ack_frame[ACK_LEN] = {
              FRAME802154_ACKFRAME,
              0x00,
              info154.seq
          };
          RADIO_IRQ_DISABLE();
          radio_strobe(RADIO_STROBE_FTX);
          st_lib_radio_pkt_basic_set_payload_length((uint16_t) ACK_LEN);
          st_lib_radio_spi_write_linear_fifo((uint16_t) ACK_LEN, (uint8_t *) ack_frame);

          radio_set_ready_state();
          RADIO_IRQ_ENABLE();
          st_lib_CmdStrobeTx();
          BUSYWAIT_UNTIL(RADIO_STATUS() == RADIO_STATE_TX, 1 * RTIMER_SECOND/1000);
          BUSYWAIT_UNTIL(RADIO_STATUS() != RADIO_STATE_TX, 1 * RTIMER_SECOND/1000);
          ACKPRINTF("debug_ack: sent ack %d\n", ack_frame[2]);

#endif /* !XXX_ACK_WORKAROUND */
        }
      }
#endif /* NULLRDC_CONF_802154_AUTOACK */

      packetbuf_set_datalen(len);   
      NETSTACK_RDC.input();
    }

    if(!IS_RXBUF_EMPTY()) {
      process_poll(&subGHz_radio_process);
    }

    if(interrupt_callback_wants_poll) {
    	/*This part of code seems not to happen, so it is not tested.*/
      Radio_interrupt_callback();
      if(RADIO_STATUS() == RADIO_STATE_READY) 
      {
#if RADIO_USES_SNIFF_MODE
    	  /*As per previous comment, this is not tested. */
    	S2LPCmdStrobeSleep();
#else /*!RADIO_USES_SNIFF_MODE*/
    	RadioCmdStrobeRx();
    	BUSYWAIT_UNTIL(RADIO_STATUS() == RADIO_STATE_RX, 1 * RTIMER_SECOND/1000);
#endif /*RADIO_USES_SNIFF_MODE*/
      }
    }
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/**
  * @brief  Radio_interrupt_callback
  * 	callback when an interrupt is received
  * @param  none
  * @retval none
  */
void
Radio_interrupt_callback(void)
{
#define INTPRINTF(...) // PRINTF
  st_lib_radio_irqs x_irq_status;
  if (radio_spi_busy() || interrupt_callback_in_progress) 
  {
    process_poll(&subGHz_radio_process);
    interrupt_callback_wants_poll = 1;
    return;
  }
  
  interrupt_callback_wants_poll = 0;
  interrupt_callback_in_progress = 1;
  
  /* get interrupt source from radio */
  st_lib_radio_irq_get_status(&x_irq_status);
  st_lib_radio_irq_clear_status();

  /* The IRQ_TX_DATA_SENT notifies the packet transmission.
   * Then puts the Radio in RX/Sleep according to the selected mode */
  if(x_irq_status.IRQ_TX_DATA_SENT)
  {
	INTPRINTF("IRQ_TX_DATA_SENT\n");
#if !RADIO_USES_SNIFF_MODE
     RadioCmdStrobeRx();
#endif /*RADIO_USES_SNIFF_MODE*/
    INTPRINTF("SENT\n");
    CLEAR_TXBUF();
    nTxIndex = 0;
    xTxDoneFlag = SET;
    interrupt_callback_in_progress = 0;
    return;
  }
  
  if(x_irq_status.IRQ_RX_FIFO_ERROR)
  {
    receiving_packet = 0;
    interrupt_callback_in_progress = 0;
    radio_strobe(RADIO_STROBE_FRX);
#ifdef S2LP_RADIO
#  if RADIO_USES_SNIFF_MODE
     S2LPCmdStrobeSleep();
#  else /*!RADIO_USES_SNIFF_MODE*/
     RadioCmdStrobeRx();
#  endif /*RADIO_USES_SNIFF_MODE*/
#endif /*S2LP_RADIO*/
    return;
  }

  if(x_irq_status.IRQ_TX_FIFO_ERROR)
  {
    receiving_packet = 0;
    interrupt_callback_in_progress = 0;
    radio_strobe(RADIO_STROBE_FTX);
    return;
  }

#if !RADIO_USES_SNIFF_MODE
  /* The IRQ_VALID_SYNC is used to notify a new packet is coming */
  if(x_irq_status.IRQ_VALID_SYNC) 
  {
    INTPRINTF("IRQ_VALID_SYNC\n");
    receiving_packet = 1;
    RadioCmdStrobeRx();
  }
#endif /*RADIO_USES_SNIFF_MODE*/

#ifdef CSMA_ENABLE  
    if(x_irq_status.IRQ_MAX_BO_CCA_REACH)
    {
      /* Send a Tx command */
      st_lib_CmdStrobeTx();
    }
#endif
#ifdef DATA_PACKET_127_BYTES
    /* Check the SPIRIT TX_FIFO_ALMOST_EMPTY IRQ flag */
    else if(x_irq_status.IRQ_TX_FIFO_ALMOST_EMPTY)
      {
        /* read the number of elements in the Tx FIFO */
        uint8_t cNElemTxFifo = st_lib_FifoReadNumberBytesTxFifo();

        /* check if the sum of the residual payload to be transmitted and the actual bytes in FIFO are higher than 96 */
        if(nResidualPcktLength+cNElemTxFifo > NETSTACK_RADIO_MAX_PAYLOAD_LEN)
        {
          /* .. if yes another almost full IRQ has to be managed */

          /* ..so fill the Tx FIFO */
          st_lib_radio_spi_write_linear_fifo(96-cNElemTxFifo-1, &radio_128_data_buffp[nTxIndex]);

          /* update the number of bytes to be transmitted */
          nResidualPcktLength -= 96-cNElemTxFifo-1;

          /* update the number of bytes transmitted until now */
          nTxIndex += 96-cNElemTxFifo-1;
        }
        else
        {
          /* .. if not all the nResidualPcktLength bytes remaining can be written to the Tx FIFO */
          /* ..so disable the TX_FIFO_ALMOST_EMPTY IRQ */
          st_lib_radio_irq(TX_FIFO_ALMOST_EMPTY , S_DISABLE);

          /* unarm the AE threshold mechanism */
          st_lib_FifoSetAlmostEmptyThresholdTx(96);

          /* fill the Tx fifo */
          st_lib_radio_spi_write_linear_fifo(nResidualPcktLength, &radio_128_data_buffp[nTxIndex]);

          /* update the number of transmitted bytes */
          nTxIndex += nResidualPcktLength;

          /* update the number of bytes to be transmitted */
          nResidualPcktLength = 0;

        }

        /* re-read the number of elements in the Tx FIFO */
        cNElemTxFifo = st_lib_FifoReadNumberBytesTxFifo();

      }
#endif  /*DATA_PACKET_127_BYTES*/
  
  /* The IRQ_RX_DATA_READY notifies a new packet arrived */
  if(x_irq_status.IRQ_RX_DATA_READY) 
  {
	INTPRINTF("IRQ_RX_DATA_READY\n");
#ifndef DATA_PACKET_127_BYTES
    st_lib_radio_spi_read_linear_fifo(st_lib_radio_linear_fifo_read_num_elements_rx_fifo(),
                                       &radio_rxbuf[1]);
#else /*!DATA_PACKET_127_BYTES*/
    cRxDataLen = st_lib_radio_linear_fifo_read_num_elements_rx_fifo();

    if(cRxDataLen!=0)
    {
    /* Read the RX FIFO */
    if(nRxIndex == 0)
      {
       st_lib_radio_spi_read_linear_fifo(cRxDataLen, &radio_rxbuf[1]);
      }
    else
      {
      st_lib_radio_spi_read_linear_fifo(cRxDataLen, &radio_rxbuf[nRxIndex]);
      }        

    }

#endif   /*DATA_PACKET_127_BYTES*/                                    
    radio_rxbuf[0] = st_lib_radio_pkt_basic_get_received_pkt_length();
    st_lib_radio_cmd_strobe_flush_rx_fifo();
    
    INTPRINTF("RECEIVED\n");

    last_rssi = (packetbuf_attr_t) st_lib_radio_qi_get_rssi(); 
    last_lqi  = (packetbuf_attr_t) st_lib_radio_qi_get_lqi(); 

    receiving_packet = 0;
    
#if NULLRDC_CONF_802154_AUTOACK
    if (radio_rxbuf[0] == ACK_LEN) 
    {
      /* For debugging purposes we assume this is an ack for us */
      just_got_an_ack = 1;
    }
#endif /* NULLRDC_CONF_802154_AUTOACK */
    
    /* update the number of received bytes */
    nRxIndex += cRxDataLen;

    nRxIndex = 0;
    interrupt_callback_in_progress = 0;

    process_poll(&subGHz_radio_process);
#if RADIO_USES_SNIFF_MODE
 	S2LPCmdStrobeSleep();
#else /*!RADIO_USES_SNIFF_MODE*/
     RadioCmdStrobeRx();
#endif /*RADIO_USES_SNIFF_MODE*/
    return;
  }
  
#if !RADIO_USES_SNIFF_MODE
  if(x_irq_status.IRQ_RX_DATA_DISC)
  {
	INTPRINTF("IRQ_RX_DATA_DISC\r\n");
	/* RX command - to ensure the device will be ready for the next reception */
	if(x_irq_status.IRQ_RX_TIMEOUT)
    {
      st_lib_radio_cmd_strobe_flush_rx_fifo();
      rx_timeout = SET; 
      RadioCmdStrobeRx();
    }
  }
#endif /*!RADIO_USES_SNIFF_MODE*/
  
#ifdef DATA_PACKET_127_BYTES
  else if(x_irq_status.IRQ_RX_FIFO_ALMOST_FULL)
  {
      nRxIndex = 0; //issue due to packet loss so reset the values
      cRxDataLen = st_lib_radio_linear_fifo_read_num_elements_rx_fifo();
       
      /* Read the RX FIFO */
      st_lib_radio_spi_read_linear_fifo(cRxDataLen, &radio_rxbuf[1]);     
      radio_rxbuf[0] = cRxDataLen;
      //   receiving_packet = 0;
      nRxIndex+=cRxDataLen + 1; 
  }
#endif /*DATA_PACKET_127_BYTES*/
  interrupt_callback_in_progress = 0;
}
/*---------------------------------------------------------------------------*/
/** @} */
/** @} */
