/*
 * Copyright (c) 2011, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "contiki.h"
#include "lib/random.h"
#include "sys/ctimer.h"
#include "sys/etimer.h"
#include "net/ip/uip.h"
#include "net/ipv6/uip-ds6.h"
#include "net/ip/uip-debug.h"
#include "dev/button-sensor.h"
#include "net/ip/ip64-addr.h"
#include "net/ip/uiplib.h"
#include "net/rpl/rpl.h"
#include "sys/node-id.h"
#include "simple-udp.h"
#include "st-lib.h"
#include "main.h"
#include "low-power.h"
#include "stm32l1xx_hal_wwdg.h"
//#include "stm32l1xx_hal.h"

/** @addtogroup Udp_sender
* @{
*/
/*---------------------------------------------------------------------------*/
/*Choose how to set the Receiver address.*/
#define SERVREG_HACK_ENABLED 1
#if SERVREG_HACK_ENABLED
#include "servreg-hack.h"
/*This is used for auto-discovery of the UDP receiver */
#  define SERVICE_ID 190
#else
/*In case you want to use an hardcoded IPv6 Server Adress, specify it here and
 * adapt the code at the beginning of unicast_sender_process accordingly.  */
#  define LWM2M_SERVER_ADDRESS_v6 "aaaa::1" //Usually used for Border Router and tunslip6
//#  define LWM2M_SERVER_ADDRESS_v6 "aaaa::a00:f7ff:104e:efb1"
#endif

#define LOOP_INTERVAL		(1 * CLOCK_SECOND)
#define RESENT_INTERVAL (LOOP_INTERVAL*5)
#define IWDG_INTERVAL   (LOOP_INTERVAL*5)

#define UDP_PORT 1234
#define MESSAGE_SIZE 20
#define MAX_RECORD 10

/* server command */
#define CMD_DISABLE_ALARM  0x04
#define CMD_ALARM_DISABLED 0x05
#define CMD_LED_ON  0x06
#define CMD_LED_OFF 0x07

#define CMD_DISABLE_ALARM_ACK 0x5b
#define CMD_ALARM_ACK 0x40

#define I2C_SENSOR_PACKET1 0x06
#define I2C_SENSOR_PACKET2 0x04 
#define I2C_SENSOR_PACKET3 0x05 
#define I2C_DATA_LEN 0x04

/*---------------------------------------------------------------------------*/
/* for resent function                                                       */
/*---------------------------------------------------------------------------*/
#define ALARM_RETRY_TIMES 8


/*---------------------------------------------------------------------------*/
#define APP_DUTY_CYCLE_SLOT  3
/*---------------------------------------------------------------------------*/
#if MCU_LOW_POWER
//Read below to see how these values are used
#define APP_DUTY_CYCLE_OFF 10

#define KEEP_RADIO_OFF 0
#define WAKE_RADIO_UP 1
/*---------------------------------------------------------------------------*/
/*In this application we define three operating modes from Application perspective.
 * The current operating mode is also reflected in the sent packet that starts with
 * [N], [S], [P] reflecting the three possibilities.
 * - DUTY_CYCLE_NONE: this is the normal behavior, MCU and RADIO are always on, according
 *     to the low power settings (i.e. MCU can sleep in idle and radio might use SNIFF mode).
 *     UDP Packets will be sent every APP_DUTY_CYCLE_SLOT seconds.
 * - DUTY_CYCLE_SIMPLE: simplest case of Application Layer Duty Cycle, that can address a
 *     PERIODIC use case (i.e. send Temperature value every tot seconds); we have a STOP
 *     slot of APP_DUTY_CYCLE_OFF seconds in which both the MCU and the radio are OFF,
 *     the node is not reachable and can't do any computation. Then both MCU and RADIO will
 *     be switched ON for TWO time slot of APP_DUTY_CYCLE_SLOT seconds, in the middle
 *     of this time, a UDP packet will always be sent. As a result, UDP packets will be
 *     sent every (APP_DUTY_CYCLE_OFF + 2*APP_DUTY_CYCLE_SLOT) seconds.
 * - DUTY_CYCLE_PROBING:  (available if the radio uses SNIFF mode, not if it uses ContikiMAC)
 *     more complex Application Layer Duty Cycle, that can address a
 *     EVENT driven use case (i.e. probe if there is smoke in the room and send the alarm);
 *     we have a STOP slot of APP_DUTY_CYCLE_OFF seconds in which both the MCU and the
 *     radio are OFF, the node is not reachable and can't do any computation. Then only MCU
 *     will be switched on while radio will stay off, so the node can do computation but it is
 *     not reachable. A probe function can be called (here we provide a simple demo only).
 *     If the application needs to send data, then it will switch radio ON and do this in
 *     next slot. Probing will be called every APP_DUTY_CYCLE_OFF seconds and will keep MCU on
 *     for APP_DUTY_CYCLE_SLOT seconds. The idea is that there is no fixed periodic
 *     UDP sending interval... actually in this simple demo the probe() function will return
 *     positive result every two calls, so UDP packets will be sent every
 *     (2*APP_DUTY_CYCLE_OFF + 3*APP_DUTY_CYCLE_SLOT) seconds.
 * */
#if RADIO_USES_CONTIKIMAC
#define DUTY_CYCLE_NB 2  //Probing with ContikiMAC is not possible since Radio will be switched on by MAC layer
#else /*!RADIO_USES_CONTIKIMAC*/
#define DUTY_CYCLE_NB 3
#endif /*RADIO_USES_CONTIKIMAC*/
typedef enum {
	DUTY_CYCLE_NONE = 0,
	DUTY_CYCLE_SIMPLE,
#if !RADIO_USES_CONTIKIMAC //Probing with ContikiMAC is not possible since Radio will be switched on by MAC layer
	DUTY_CYCLE_PROBING
#endif /*!RADIO_USES_CONTIKIMAC*/
} duty_cycle_t;
#define NEXT_DUTY_CYCLE(a) (duty_cycle_t)((a+1)%DUTY_CYCLE_NB)

static const char *duty_cycle_name[DUTY_CYCLE_NB] =
{
		"NONE",
		"SIMPLE"
#if !RADIO_USES_CONTIKIMAC  //Probing with ContikiMAC is not possible since Radio will be switched on by MAC layer
		, "PROBING"
#endif /*!RADIO_USES_CONTIKIMAC*/
};



//static void MX_WWDG_Init(void);
//  HAL_IWDG_Refresh(&hiwdg); 




/*We can have two phases:
 * - DUTY_CYCLE_PHASE_STOP: if duty cycle is NOT of type DUTY_CYCLE_NONE the system
 *     will go in STOP mode. According to the specific duty cycle type, after the system
 *     wake up we'll have a probe or a direct UDP sending.
 * - DUTY_CYCLE_PHASE_SEND: time to send some data.
 *
 * */
typedef enum {
	DUTY_CYCLE_PHASE_STOP = 0,
	DUTY_CYCLE_PHASE_SEND
} duty_cycle_phase_t;
/*---------------------------------------------------------------------------*/
duty_cycle_t duty_cycle = DUTY_CYCLE_NONE;
duty_cycle_phase_t phase = DUTY_CYCLE_PHASE_SEND;
int from_stop = 0; //This is needed to skip spurious button IRQ after wake up.
#endif /*#if MCU_LOW_POWER*/
#if LP_PERIPHERAL_IN_SLEEP
extern int from_sleep;
#endif /*LP_PERIPHERAL_IN_SLEEP*/

static struct simple_udp_connection unicast_connection;
#if !SERVREG_HACK_ENABLED
static uip_ipaddr_t server_ipaddr;
#endif /*!SERVREG_HACK_ENABLED*/


static uip_ipaddr_t *addr = NULL;
static struct ctimer send_timer;
static unsigned long int message_number = 0;

extern TIM_HandleTypeDef htim3;
extern I2C_HandleTypeDef I2cHandle;
enum {

  EVENT_COMMAND=0x01,
  EVENT_TEST=0x02,
  EVENT_ALARM=0x03,
  EVENT_REPORT=0x04,
};

typedef struct 
{
    uint8_t cmd;
    uint8_t device_type;
    uint8_t alarm_status;
    uint8_t index;
    uint8_t status;
    uint8_t sensor_type;
    uint8_t sensor_data_len;
    uint8_t battery;
    uint8_t total_sensor;
    uint8_t sensor_data[4][4];
    
}sensor_pkt;


uint8_t sensor_data[4][4];

struct pkt_data{
uint8_t valid_bit;
uip_ipaddr_t node_addr;
uint8_t received_data[100];
uint16_t datalen;
};  

struct pkt_data data_buffer[1];
sensor_pkt *spkt; 


/* Buffer used for I2C reception  */
uint8_t aRxBuffer[6]={0,0,0,0,0,0};
uint8_t aRxBuffer2[4]={0,0,0,0};
uint8_t aRxBuffer3[5]={0,0,0,0,0};
//volatile uint8_t aRxBuffer4[50]={0,0,0,0,0};
volatile uint8_t RxCounter=0;

 uint8_t aTxBuffer[6]={0x00,0xAE,0x8C,0x07,0x20,0x01};
 uint8_t aTxBuffer2[4]={0x08,0x06,0x01,0x04};
 uint8_t aTxBuffer3[5]={0x08,0x55,0xAA,0x55,0xAA};
static uint8_t received_counter=0;
static uint8_t sensor_triggered=0;
sensor_pkt  pkt;

 
int BootUp=1;

int total_sensor=0; 
int sensor_index=0; 
static int tmp_humidity_counter=0;

/* for battery monitoring */ 
extern ADC_HandleTypeDef hadc;
//extern DMA_HandleTypeDef hdma_adc; 
uint32_t BatValue=0; 
extern uint32_t adcValue; 
/* for battery monitoring */ 

extern IWDG_HandleTypeDef hiwdg; 
 
 
 
/*---------------------------------------------------------------------------*/
/* for resent function                                                       */
/*---------------------------------------------------------------------------*/
int Sensor_Alarm_Triggered=0;
int Alarm_Resent_Times=0;
int Alarm_Retry_Flag=0;
 //static int i2c_counter=0;
/*---------------------------------------------------------------------------*/
PROCESS(unicast_sender_process, "Unicast sender example process");
PROCESS(data_receiver_process,"data from server process");
PROCESS(battery_process,"battery monitoring process");
PROCESS(watchdog_process, "watchdog process");
AUTOSTART_PROCESSES(&unicast_sender_process,&data_receiver_process,&battery_process,&watchdog_process);


/*---------------------------------------------------------------------------*/
void Sent_Testing_Data(void)
{
#if 0 
  if(i2c_counter%2==0) 
   {
       aTxBuffer2[1]=0x07; //led off  
       I2C_Sensor_Write();        
   }else
   {
       aTxBuffer2[1]=0x06; //led on 
       I2C_Sensor_Write();       
     
   }
   i2c_counter++;

 Alarm_Retry_Flag=0; 
   
 if(Alarm_Retry_Flag==1)
 {
     Sensor_Alarm_Triggered=0;
 }else
 {         
     //if(aRxBuffer2[1]==0x03||aRxBuffer2[1]==0x02||aRxBuffer2[1]==0x08)
     Sensor_Alarm_Triggered=1;
     aRxBuffer2[1]==0x03;
 } 
 #endif
  Alarm_Retry_Flag=1;
 // aRxBuffer2[1]=0x03; 
 // process_post(&unicast_sender_process,EVENT_TEST,NULL);      
}  
/*---------------------------------------------------------------------------*/


static void
receiver(struct simple_udp_connection *c,
         const uip_ipaddr_t *sender_addr,
         uint16_t sender_port,
         const uip_ipaddr_t *receiver_addr,
         uint16_t receiver_port,
         const uint8_t *data,
         uint16_t datalen)
{
 
#if 0
  printf("Data received %s: from:",
         data);
  
    uip_debug_ipaddr_print(sender_addr);
    printf("\r\n");
#endif
//1. create a new thread
//2. add new event
//3. 
//4. ack :91  
 #if 0   
    if((received_counter%MAX_RECORD)==0)
    {
        received_counter=0;
    } 
#endif    
    //memcpy(&data_buffer[received_counter].node_addr,sender_addr,sizeof(uip_ipaddr_t));
    //memcpy(&data_buffer[received_counter].received_data,data,sizeof(uint8_t)*datalen);
    
    memcpy(&data_buffer[0].node_addr,sender_addr,sizeof(uip_ipaddr_t));
    memcpy(&data_buffer[0].received_data,data,sizeof(uint8_t)*datalen);

      
    data_buffer[0].datalen=datalen;
    data_buffer[0].valid_bit=1;
   // received_counter++;

    printf("\r\n recevied data from: \r\n");
 
    uip_debug_ipaddr_print(sender_addr);
    printf("\r\n datalen:%d\r\n",datalen);
      
  
   // process_post(&data_receiver_process,EVENT_COMMAND,NULL);      
  
    spkt=(sensor_pkt*)&data_buffer[0].received_data;
      
        if(spkt->cmd==CMD_DISABLE_ALARM)
        {
            spkt->cmd=CMD_DISABLE_ALARM_ACK;
            BSP_LED_Off(LED_ALARM);
            HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);
            simple_udp_sendto(&unicast_connection,(const void *)spkt, sizeof(sensor_pkt),&data_buffer[0].node_addr );
            printf(" disable command ACK\r\n");
       
        }else if(spkt->cmd==CMD_LED_ON || spkt->cmd==CMD_LED_OFF)
        {
            aTxBuffer2[1]=spkt->cmd; //led on/off  
            printf("LED command:%d \r\n",aTxBuffer2[1]);             
            I2C_Sensor_Write();  
            // send an LED ack   for resent 
  #if 0         
            if(spkt->cmd==CMD_LED_ON)
            {
                spkt->cmd=CMD_LED_OFF;        
            }else if(spkt->cmd==CMD_LED_OFF)
            {
                spkt->cmd=CMD_LED_ON;       
            } 
  #endif                   
            simple_udp_sendto(&unicast_connection,(const void *)spkt, sizeof(sensor_pkt),&data_buffer[0].node_addr );          
            printf(" sensor  ACK:%d\r\n",spkt->cmd);
        }else if(spkt->cmd==CMD_ALARM_ACK)
        {
          Alarm_Retry_Flag=0;//resent
          Alarm_Resent_Times=0;
          printf(" received an alarm ACK:%d\r\n",spkt->cmd); 
        }
 
    
}
#if MCU_LOW_POWER
/*---------------------------------------------------------------------------*/
#  if !RADIO_USES_CONTIKIMAC
/*This function implements the dummy probing of some sensor to decide if
 * we need to switch the radio on to send some data. retval=1 means it is the case
 * */
static int probe (void)
{
	/*Replace this demo with actual sensor reading, in this simple example we
	 * send  data every two probes */
	static char data_to_send = 1;
	return data_to_send^=1;
}
#  endif /*!RADIO_USES_CONTIKIMAC*/
/*---------------------------------------------------------------------------*/
int sender_check_connection()
{
	if (addr == NULL)  { //Actually can happen only when servreg_hack service is on
#if SERVREG_HACK_ENABLED
      addr = servreg_hack_lookup(SERVICE_ID);
#endif /*SERVREG_HACK_ENABLED*/
    }
	return (addr!=NULL);
}
/*---------------------------------------------------------------------------*/
static void
periodic_sender(void *ptr)
{
    char buf[MESSAGE_SIZE]="";
    (void) sender_check_connection();
    if (duty_cycle != DUTY_CYCLE_NONE && phase == DUTY_CYCLE_PHASE_STOP && addr != NULL){
#if !RADIO_USES_CONTIKIMAC
    	if (duty_cycle==DUTY_CYCLE_PROBING){
    		/*We go in STOP, and we will NOT wake up radio after this */
        	printf("PROBING Duty Cycle, going in stop mode.\r\n");
        	from_stop = 1;
        	LP_enter_stop_mode(APP_DUTY_CYCLE_OFF);
        	LP_exit_stop_mode(KEEP_RADIO_OFF);
    		/* DUTY_CYCLE_PROBING: after STOP, now MCU is ON and RADIO is OFF, so we check if
    		 * there is any data to send, if yes we'll switch radio on and go in DUTY_CYCLE_PHASE_SEND
    		 * step to send this info in next slot, otherwise we stay in DUTY_CYCLE_PHASE_STOP step.
    		 * */
    		if (probe()){
        		subGHz_radio_driver.on();
        		phase=DUTY_CYCLE_PHASE_SEND;
        		printf("Found some data to send.\r\n");
    		} else {
        		printf("No data to send, will keep probing with radio off.\r\n");
    		}
    	}
    	else
#endif /*!RADIO_USES_CONTIKIMAC*/
    	{ //duty_cycle == DUTY_CYCLE_SIMPLE
        	printf("SIMPLE Duty Cycle, going in stop mode.\r\n");
        	phase=DUTY_CYCLE_PHASE_SEND;
        	from_stop = 1;
        	LP_enter_stop_mode(APP_DUTY_CYCLE_OFF);
        	LP_exit_stop_mode(WAKE_RADIO_UP);
    	}
    } else if((duty_cycle == DUTY_CYCLE_NONE || phase == DUTY_CYCLE_PHASE_SEND) && addr != NULL) {
      printf("Sending message %lu to ", message_number);
      uip_debug_ipaddr_print(addr);
      printf("\r\n");

      sprintf(buf, "[%c] Message %lu", duty_cycle_name[duty_cycle][0], message_number);
      //Use next line instead to match the simpler CSV "sender,msg" code of the receiver
      //      sprintf(buf, "%lu", message_number);
      message_number++;
      simple_udp_sendto(&unicast_connection, buf, strlen(buf) + 1, addr);
      if (duty_cycle != DUTY_CYCLE_NONE) {
    	  phase = DUTY_CYCLE_PHASE_STOP;
      }
    } else {
#if SERVREG_HACK_ENABLED
		printf("Service %d not found\n", SERVICE_ID);
#else /*!SERVREG_HACK_ENABLED*/
		/*Should never happen*/
		printf("Unexpected error: no receiver address is set!\n");
#endif /*SERVREG_HACK_ENABLED*/
    }
	ctimer_reset(&send_timer);
}
#else /*!MCU_LOW_POWER*/
static void
periodic_sender(void *ptr)
{
    char buf[MESSAGE_SIZE]="";
    if(addr == NULL) { //Actually can happen only when servreg_hack service is on
#if SERVREG_HACK_ENABLED
      addr = servreg_hack_lookup(SERVICE_ID);
#endif /*SERVREG_HACK_ENABLED*/
    }
    if( addr != NULL) {
          printf("Sending message %lu to ", message_number);
          uip_debug_ipaddr_print(addr);
          printf("\r\n");

          sprintf(buf, "Message %lu",  message_number);
          //Use next line instead to match the simpler CSV "sender,msg" code of the receiver
          //      sprintf(buf, "%lu", message_number);
          message_number++;
          simple_udp_sendto(&unicast_connection, buf, strlen(buf) + 1, addr);
    } else {
    #if SERVREG_HACK_ENABLED
    		printf("Service %d not found\n", SERVICE_ID);
    #else /*!SERVREG_HACK_ENABLED*/
    		/*Should never happen*/
    		printf("Unexpected error: no receiver address is set!\n");
    #endif /*SERVREG_HACK_ENABLED*/
    }
    ctimer_reset(&send_timer);
}
#endif /*MCU_LOW_POWER*/
/*---------------------------------------------------------------------------*/
static void
set_global_address(void)
{
  uip_ipaddr_t ipaddr;
  int i;
  uint8_t state;

  uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
  uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);

  printf("IPv6 addresses: ");
  for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
    state = uip_ds6_if.addr_list[i].state;
    if(uip_ds6_if.addr_list[i].isused &&
       (state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
         uip_debug_ipaddr_print(&uip_ds6_if.addr_list[i].ipaddr);
         printf("\n");
       }
  }
}

/*---------------------------------------------------------------------------*/
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
    
}  
/*---------------------------------------------------------------------------*/
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{

  // if alarm triggered return alarm_flag=0;
  /* Toggle LED2: Transfer in reception process is correct */
  
  #if 1 // 0:for LED image. 1: for other nodes

if(RxCounter==I2C_SENSOR_PACKET1)  
{
      while(HAL_I2C_GetState(I2cHandle) != HAL_I2C_STATE_READY)
      {
      }
      total_sensor=aRxBuffer[5];
      HAL_I2C_Slave_Receive_DMA(I2cHandle,(uint8_t*)aRxBuffer2,I2C_SENSOR_PACKET2);
      //HAL_I2C_Slave_Receive_IT(I2cHandle,(uint8_t*)aRxBuffer2,I2C_SENSOR_PACKET2);        
      RxCounter=I2C_SENSOR_PACKET2;
}else if(RxCounter==I2C_SENSOR_PACKET2)
{
      while (HAL_I2C_GetState(I2cHandle) != HAL_I2C_STATE_READY)
      {
      } 
     HAL_I2C_Slave_Receive_DMA(I2cHandle,(uint8_t*)aRxBuffer3,I2C_SENSOR_PACKET3);
    //HAL_I2C_Slave_Receive_IT(I2cHandle,(uint8_t*)aRxBuffer3,I2C_SENSOR_PACKET3);  
    RxCounter=I2C_SENSOR_PACKET3;
    //sensor_index++;      
}else if(RxCounter==I2C_SENSOR_PACKET3)
{
    while (HAL_I2C_GetState(I2cHandle) != HAL_I2C_STATE_READY)
    {
    }    
    
    memcpy(&sensor_data[sensor_index][0],&aRxBuffer3[1],I2C_SENSOR_PACKET2); 
    sensor_index++;   
    
    if(sensor_index<total_sensor)
    {
       
        HAL_I2C_Slave_Receive_DMA(I2cHandle,(uint8_t*)aRxBuffer2,I2C_SENSOR_PACKET2); 
        // HAL_I2C_Slave_Receive_IT(I2cHandle,(uint8_t*)aRxBuffer2,I2C_SENSOR_PACKET2); 
        RxCounter=I2C_SENSOR_PACKET2;
      
    }else
    {
        
       
        //HAL_I2C_Slave_Receive_IT(I2cHandle,(uint8_t*)aRxBuffer,I2C_SENSOR_PACKET1); 
        RxCounter=I2C_SENSOR_PACKET1;    
        sensor_index=0;
        if(Alarm_Retry_Flag==1)
        {
            Sensor_Alarm_Triggered=0;
        }else
        {         
            if(aRxBuffer2[1]==0x03||aRxBuffer2[1]==0x02||aRxBuffer2[1]==0x08)
            Sensor_Alarm_Triggered=1;
        } 
        HAL_I2C_Slave_Receive_DMA(I2cHandle,(uint8_t*)aRxBuffer,I2C_SENSOR_PACKET1);         
       // alarm_flag=1; resent
       // process_post(&unicast_sender_process,EVENT_REPORT,NULL);  
        // send a event to report thread.
    }      
}
#endif
}


/*****************************************************************
Function: I2C_Sensor_Read
Desription: Read command from sensor.

comment: function header define in main.h

******************************************************************/
void I2C_Sensor_Read(void)
{
	
	HAL_I2C_Slave_Receive_IT(&I2cHandle,(uint8_t*)aRxBuffer,6);
	
	while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY)
  {
  } 		
	
	  //RxCounter++;
	
	//////////////////////////////////////////////////////////////////
	  //RxCounter++;
	
    //if(RxCounter==1)
    //{
      
      
    //  HAL_I2C_Slave_Receive_IT(&I2cHandle, (uint8_t*)aRxBuffer,6);

    //  while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY)
    //  {
    //  } 			
     
    //}    
    //else if(RxCounter==2)
    //{  
      
      HAL_I2C_Slave_Receive_IT(&I2cHandle, (uint8_t*)aRxBuffer2,4);
			
			while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY)
      {
      } 
  
    //}
		//else if(RxCounter==3)
    //{
     
      HAL_I2C_Slave_Receive_IT(&I2cHandle, (uint8_t*)aRxBuffer3,5);
			
			while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY)
      {
      } 
				
    // RxCounter=0;
     sensor_triggered=1; //bootup flag
	  //}		
  ////////////////////////////////////////////////////////////////
	
}	

/*****************************************************************
Function: I2C_Sensor_Write
Desription: Write command to LED sensor.

comment: function header define in main.h

******************************************************************/
void I2C_Sensor_Write(void)
{
   int i=0;
     
  // aRxBuffer[0]=0x00;
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET ); 
	 HAL_Delay(10);
	

 /*##-2- Put I2C peripheral in reception process ###########################*/  
 // (I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size)
for(i=0;i<6;i++)
{
  //if(HAL_I2C_Master_Transmit_IT(&I2cHandle,I2C_ADDRESS,(uint8_t *)&aTxBuffer[i],1) != HAL_OK)
  //if(HAL_I2C_Master_Transmit_DMA(&I2cHandle,I2C_ADDRESS,(uint8_t *)&aTxBuffer[i], 1) != HAL_OK)
  if(HAL_I2C_Master_Transmit_IT(&I2cHandle,I2C_ADDRESS,(uint8_t *)&aTxBuffer[i],1) != HAL_OK)  
  {
    /* Transfer error in reception process */
    Error_Handler();
  }

  /*##-3- Wait for the end of the transfer ###################################*/  
  /*  Before starting a new communication transfer, you need to check the current   
      state of the peripheral; if it’s busy you need to wait for the end of current
      transfer before starting a new one.
      For simplicity reasons, this example is just waiting till the end of the
      transfer, but application may perform other tasks while transfer operation
      is ongoing. */

  while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY)
  {
  }
  
}  

//jas add
HAL_Delay(1); 

for(i=0;i<4;i++)
{    
 /*##-2- Put I2C peripheral in reception process ###########################*/  
  // (I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size)
	if(HAL_I2C_Master_Transmit_IT(&I2cHandle,I2C_ADDRESS,(uint8_t *)&aTxBuffer2[i],1) != HAL_OK)
  //if(HAL_I2C_Master_Transmit_DMA(&I2cHandle,I2C_ADDRESS,(uint8_t *)&aTxBuffer2[i],1) != HAL_OK)
  {
    /* Transfer error in reception process */
    Error_Handler();
  }
  
  /*##-3- Wait for the end of the transfer ###################################*/  
  /*  Before starting a new communication transfer, you need to check the current   
      state of the peripheral; if it’s busy you need to wait for the end of current
      transfer before starting a new one.
      For simplicity reasons, this example is just waiting till the end of the
      transfer, but application may perform other tasks while transfer operation
      is ongoing. */

  while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY)
  {
  }
}
 
for(i=0;i<5;i++)
{   
 /*##-2- Put I2C peripheral in reception process ###########################*/  
  // (I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size)
	if(HAL_I2C_Master_Transmit_IT(&I2cHandle,I2C_ADDRESS,(uint8_t *)&aTxBuffer3[i],1) != HAL_OK)
  //if(HAL_I2C_Master_Transmit_DMA(&I2cHandle,I2C_ADDRESS,(uint8_t *)&aTxBuffer3[i],1) != HAL_OK)
  {
    /* Transfer error in reception process */
    Error_Handler();
  }
  
  /*##-3- Wait for the end of the transfer ###################################*/  
  /*  Before starting a new communication transfer, you need to check the current   
      state of the peripheral; if it’s busy you need to wait for the end of current
      transfer before starting a new one.
      For simplicity reasons, this example is just waiting till the end of the
      transfer, but application may perform other tasks while transfer operation
      is ongoing. */

  while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY)
  {
  }
  
}

//jas add
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);  



	  //}		
  ////////////////////////////////////////////////////////////////
  
   
}  


/*****************************************************************
Function: I2C_Sensor_Query
Desription: To know which sensor is connected with the node.

comment: function header define in main.h

******************************************************************/
void I2C_Sensor_Query(void)
{

     
  //I2C_Sensor_Write();
   int i=0;
  
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET ); 
	 HAL_Delay(10);
	

 /*##-2- Put I2C peripheral in reception process ###########################*/  
 // (I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size)
for(i=0;i<6;i++)
{
  //if(HAL_I2C_Master_Transmit_DMA(&I2cHandle,I2C_ADDRESS,(uint8_t *)&aTxBuffer[i],1) != HAL_OK)
  if(HAL_I2C_Master_Transmit_DMA(&I2cHandle,I2C_ADDRESS,(uint8_t *)&aTxBuffer[i], 1) != HAL_OK)
  //if(HAL_I2C_Master_Transmit_IT(&I2cHandle,I2C_ADDRESS,(uint8_t *)&aTxBuffer[i],1) != HAL_OK)  
  {
    /* Transfer error in reception process */
    Error_Handler();
  }

  /*##-3- Wait for the end of the transfer ###################################*/  
  /*  Before starting a new communication transfer, you need to check the current   
      state of the peripheral; if it’s busy you need to wait for the end of current
      transfer before starting a new one.
      For simplicity reasons, this example is just waiting till the end of the
      transfer, but application may perform other tasks while transfer operation
      is ongoing. */

  while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY)
  {
  }
  
}  

//jas add
HAL_Delay(1); 

for(i=0;i<4;i++)
{    
 /*##-2- Put I2C peripheral in reception process ###########################*/  
  // (I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size)
	//if(HAL_I2C_Master_Transmit_IT(&I2cHandle,I2C_ADDRESS,(uint8_t *)&aTxBuffer2[i],1) != HAL_OK)
  if(HAL_I2C_Master_Transmit_DMA(&I2cHandle,I2C_ADDRESS,(uint8_t *)&aTxBuffer2[i],1) != HAL_OK)
  {
    /* Transfer error in reception process */
    Error_Handler();
  }
  
  /*##-3- Wait for the end of the transfer ###################################*/  
  /*  Before starting a new communication transfer, you need to check the current   
      state of the peripheral; if it’s busy you need to wait for the end of current
      transfer before starting a new one.
      For simplicity reasons, this example is just waiting till the end of the
      transfer, but application may perform other tasks while transfer operation
      is ongoing. */

  while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY)
  {
  }
}
 
for(i=0;i<5;i++)
{   
 /*##-2- Put I2C peripheral in reception process ###########################*/  
  // (I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size)
	//if(HAL_I2C_Master_Transmit_IT(&I2cHandle,I2C_ADDRESS,(uint8_t *)&aTxBuffer3[i],1) != HAL_OK)
  if(HAL_I2C_Master_Transmit_DMA(&I2cHandle,I2C_ADDRESS,(uint8_t *)&aTxBuffer3[i],1) != HAL_OK)
  {
    /* Transfer error in reception process */
    Error_Handler();
  }
  
  /*##-3- Wait for the end of the transfer ###################################*/  
  /*  Before starting a new communication transfer, you need to check the current   
      state of the peripheral; if it’s busy you need to wait for the end of current
      transfer before starting a new one.
      For simplicity reasons, this example is just waiting till the end of the
      transfer, but application may perform other tasks while transfer operation
      is ongoing. */

  while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY)
  {
  }
  
}

  //jas add
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);  


  HAL_I2C_Slave_Receive_DMA(&I2cHandle,(uint8_t*)aRxBuffer,6);
  RxCounter=6;  
  //HAL_I2C_Slave_Receive_IT(&I2cHandle,(uint8_t*)aRxBuffer,6);	
 #if 0 // for led
	while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY)
  {
  } 		
	
	  //RxCounter++;
	
	//////////////////////////////////////////////////////////////////
	  //RxCounter++;
	
    //if(RxCounter==1)
    //{
      
      
    //  HAL_I2C_Slave_Receive_IT(&I2cHandle, (uint8_t*)aRxBuffer,6);

    //  while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY)
    //  {
    //  } 			
     
    //}    
    //else if(RxCounter==2)
    //{  
      
      HAL_I2C_Slave_Receive_IT(&I2cHandle, (uint8_t*)aRxBuffer2,4);
			
			while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY)
      {
      } 
  
    //}
		//else if(RxCounter==3)
    //{
     
      HAL_I2C_Slave_Receive_IT(&I2cHandle, (uint8_t*)aRxBuffer3,5);
			
			while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY)
      {
      } 
				
    // RxCounter=0;
    // sensor_triggered=1; //bootup flag
  
 // for ohter nodes
 // HAL_I2C_Slave_Receive_DMA(&I2cHandle,(uint8_t*)aRxBuffer,6);
	//HAL_I2C_Slave_Receive_IT(&I2cHandle,(uint8_t*)aRxBuffer,6);
#endif	  

	//I2C_Sensor_Read();

}  


/*---------------------------------------------------------------------------*/
PROCESS_THREAD(unicast_sender_process, ev, data)
{


  int i=0;
  int j=0;
  static struct etimer periodic_timer;  
    
  etimer_set(&periodic_timer, RESENT_INTERVAL);

  /*To skip autodetection of the receiver using the servreg service,
 * set SERVREG_HACK_ENABLED to 0 and uncomment one of the specific lines
 * (hardcoded server address or multicast message) and set the addr pointer.
 */
#if !SERVREG_HACK_ENABLED
  uiplib_ip6addrconv(LWM2M_SERVER_ADDRESS_v6, &server_ipaddr); //Hardcoded Server (set accordingly)
  //uip_create_linklocal_allnodes_mcast(&server_ipaddr); //Multicast UDP message.
  addr = &server_ipaddr; //We set it in hardcoded or multicast case...
#else /*SERVREG_HACK_ENABLED*/
  addr = NULL; //... otherwise we leave it to NULL and servreg will sort this out
#endif /*!SERVREG_HACK_ENABLED*/

  PROCESS_BEGIN();
  
#if SERVREG_HACK_ENABLED
  servreg_hack_init();
#endif /*SERVREG_HACK_ENABLED*/
  set_global_address();
  simple_udp_register(&unicast_connection, UDP_PORT,
                      NULL, UDP_PORT, receiver);
  
 

  


  printf("unicast_sender_process \r\n");
  while(1) {
    PROCESS_WAIT_EVENT();
    if(ev==PROCESS_EVENT_TIMER || ev==EVENT_TEST){
   // if(ev==PROCESS_EVENT_TIMER && data==&periodic_timer){  
#if MCU_LOW_POWER
    	if (from_stop
#  if LP_PERIPHERAL_IN_SLEEP
				|| from_sleep
#  endif /*LP_PERIPHERAL_IN_SLEEP*/
    			) {
    		/*After STOP/SLEEP modes we'll get here even without button press, so we skip*/
    		from_stop = 0;
#  if LP_PERIPHERAL_IN_SLEEP
    		from_sleep = 0;
#  endif /*LP_PERIPHERAL_IN_SLEEP*/
       	} else {
    		/*Real button press, we change the Duty Cycle type*/
    		duty_cycle = NEXT_DUTY_CYCLE(duty_cycle);
    		printf("Duty cycle is now %s.\r\n", duty_cycle_name[duty_cycle]);
    	}
#endif /*MCU_LOW_POWER*/
       //etimer_reset(&periodic_timer); 
      addr = servreg_hack_lookup(SERVICE_ID);   
           
    if (addr != NULL && (unicast_connection.udp_conn!=NULL))  { //Actually can happen only when servreg_hack service is on
    BSP_LED_Toggle(LED_GREEN);  ///jas mark
        
     //printf("server address is null\r\n");
     // probe sensor function 
      
     
      if(sensor_triggered ==1 || ev==EVENT_TEST||Sensor_Alarm_Triggered==1||Alarm_Retry_Flag==1)
      {
             
                      
             if(aRxBuffer2[1]==0x03||ev==EVENT_TEST||Alarm_Retry_Flag==1)
             {
                 pkt.cmd=3;
                 HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
                 BSP_LED_On(LED_ALARM);
                 
                 Sensor_Alarm_Triggered=0;
                 
                 if(Alarm_Retry_Flag==1)
                 {Alarm_Resent_Times++;}
                 
                  Alarm_Retry_Flag=1;
                
                 if(Alarm_Resent_Times>=ALARM_RETRY_TIMES)
                 {
                   Alarm_Retry_Flag=0;  
                   Alarm_Resent_Times=0;
                 }  
                 
                 if(Alarm_Retry_Flag==1)
                 printf("\r\n retry times: %d:\r\n",Alarm_Resent_Times); 
                 
                 printf("\r\n command:%d \r\n",pkt.cmd);              
             
             pkt.device_type=0x03;//node
             pkt.alarm_status=aRxBuffer2[1]; // 1: alarm was triggered
             pkt.index=1;
             
             pkt.sensor_type=aRxBuffer2[0];
             
             if( pkt.sensor_type==0x0c)
             {
               tmp_humidity_counter++;   
             }
             
             pkt.status=aRxBuffer2[2];
             pkt.total_sensor=total_sensor;          
             
             for(i=0;i<total_sensor;i++)
             for(j=0;j<I2C_DATA_LEN;j++)
             pkt.sensor_data[i][j]=sensor_data[i][j];
             //data 
             
             //for(i=0;i<5;i++)
             //{
              // pkt.sensor_data1=aRxBuffer3[1];
               
              // memcpy(&pkt.sensor_data,(const void *)&aRxBuffer3[1],4);
            // }        
             
           
             pkt.battery=BatValue;
       
            if(unicast_connection.udp_conn!=NULL)      
            {
                // send sensor report data here
               // BSP_LED_Toggle(LED_GREEN);
             
              //sprintf(&buf[3], "Message %lu",  message_number);
                if( pkt.sensor_type==0x0c &&tmp_humidity_counter%5==0)
                {
                  simple_udp_sendto(&unicast_connection,(const void *)&pkt, sizeof(pkt),addr);
                  tmp_humidity_counter=0;
                }else if( pkt.sensor_type!=0x0c)
                {
                     simple_udp_sendto(&unicast_connection,(const void *)&pkt, sizeof(pkt),addr);// strlen(buf),addr);
                } 
                 
                //message_number++;LED_ALARM
                sensor_triggered=0;
            }                 
                 
                 
             }
             else if(aRxBuffer2[1]==0x02||aRxBuffer2[1]==0x08)
             {
                 pkt.cmd=2;
              #if 0   
                 /* if retry mechanism run this code, it will exit. just */
                 /* retry code should not run this segment*/
                 if(Alarm_Retry_Flag==1)
                 {Alarm_Resent_Times++;}
                 
                  Alarm_Retry_Flag=1;
                
                 if(Alarm_Resent_Times>=ALARM_RETRY_TIMES)
                 {
                   Alarm_Retry_Flag=0;  
                   Alarm_Resent_Times=0;
                 }  
                 
                 if(Alarm_Retry_Flag==1)
                   printf("\r\n retry times: %d:\r\n",Alarm_Resent_Times); 
                 /* if retry mechanism run this code, it will exit. just */
                #endif 

                printf("\r\n command:%d \r\n",pkt.cmd);              
             
             pkt.device_type=0x03;//node
             pkt.alarm_status=aRxBuffer2[1]; // 1: alarm was triggered
             pkt.index=1;
             
             pkt.sensor_type=aRxBuffer2[0];
             
             if( pkt.sensor_type==0x0c)
             {
               tmp_humidity_counter++;   
             }
             
             pkt.status=aRxBuffer2[2];
             pkt.total_sensor=total_sensor;          
             
             for(i=0;i<total_sensor;i++)
             for(j=0;j<I2C_DATA_LEN;j++)
             pkt.sensor_data[i][j]=sensor_data[i][j];
             //data 
             
             //for(i=0;i<5;i++)
             //{
              // pkt.sensor_data1=aRxBuffer3[1];
               
              // memcpy(&pkt.sensor_data,(const void *)&aRxBuffer3[1],4);
            // }        
             
           
             pkt.battery=BatValue;
       
            if(unicast_connection.udp_conn!=NULL)      
            {
                // send sensor report data here
               // BSP_LED_Toggle(LED_GREEN);
             
              //sprintf(&buf[3], "Message %lu",  message_number);
                if( pkt.sensor_type==0x0c &&tmp_humidity_counter%5==0)
                {
                  simple_udp_sendto(&unicast_connection,(const void *)&pkt, sizeof(pkt),addr);
                  tmp_humidity_counter=0;
                }else if( pkt.sensor_type!=0x0c)
                {
                     simple_udp_sendto(&unicast_connection,(const void *)&pkt, sizeof(pkt),addr);// strlen(buf),addr);
                } 
                 
                //message_number++;LED_ALARM
                sensor_triggered=0;
            }                 
           }             
             
     }
    
     
      // 0x01; // report type: 0,periodic 1,alarm
      //buf[1]=0x09; // device ID:0x04 door detector
  
     // printf("Sensor d: %d\n", pkt.sensor_data);       
    //  etimer_reset_with_new_interval(&periodic_timer,rand()%LOOP_INTERVAL);
      etimer_reset_with_new_interval(&periodic_timer,RESENT_INTERVAL);
      etimer_restart(&periodic_timer);  
      }
      
    }
  }
  PROCESS_END();
}

WWDG_HandleTypeDef hwwdg;

static void MX_WWDG_Init(void)
{
  
  hwwdg.Instance = WWDG;
  hwwdg.Init.Prescaler = WWDG_PRESCALER_8;
  hwwdg.Init.Window = 64;
  hwwdg.Init.Counter = 127;
  hwwdg.Init.EWIMode = WWDG_EWI_ENABLE;
  if (HAL_WWDG_Init(&hwwdg) != HAL_OK)
  {
    printf("watch dog MX_WWDG_Init failed \r\n");
  }
  #if 0
  hwwdg.Init.Counter = 127;
  if (HAL_WWDG_Init(&hwwdg) != HAL_OK)
  {
      printf("EWI watch dog MX_WWDG_Init failed \r\n");  
  }   
  #endif  
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(data_receiver_process, ev, data)
{
  
    
    PROCESS_BEGIN();    
    
    while(1){
    PROCESS_WAIT_EVENT();  
    if(ev==EVENT_COMMAND)   
    {
        spkt=(sensor_pkt*)&data_buffer[0].received_data;
      
        if(spkt->cmd==CMD_DISABLE_ALARM)
        {
            spkt->cmd=CMD_DISABLE_ALARM_ACK;
            BSP_LED_Off(LED_ALARM);
            HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);
            simple_udp_sendto(&unicast_connection,(const void *)spkt, sizeof(sensor_pkt),&data_buffer[0].node_addr );
            printf(" disable command ACK\r\n");
       
        }else if(spkt->cmd==CMD_LED_ON || spkt->cmd==CMD_LED_OFF)
        {
            aTxBuffer2[1]=spkt->cmd; //led on/off  
           // printf("received command:%d \r\n",aTxBuffer2[1]);             
            I2C_Sensor_Write();  
            // send an LED ack   for resent 
            
            if(spkt->cmd==CMD_LED_ON)
            {
                spkt->cmd=CMD_LED_OFF;        
            }else if(spkt->cmd==CMD_LED_OFF)
            {
                spkt->cmd=CMD_LED_ON;       
            }
            simple_udp_sendto(&unicast_connection,(const void *)spkt, sizeof(sensor_pkt),&data_buffer[0].node_addr );          
          printf(" command:%d,sensor  ACK:%d\r\n",pkt.cmd,spkt->cmd);
        }else if(spkt->cmd==CMD_ALARM_ACK)
        {
          Alarm_Retry_Flag=0;//resent
          Alarm_Resent_Times=0;
          printf(" received an alarm ACK:%d\r\n",spkt->cmd); 
        }

    }// end of EVENT_COMMAND       
    
    }
    PROCESS_END();  
}




/*---------------------------------------------------------------------------*/
PROCESS_THREAD(watchdog_process, ev, data)
{
  
    static struct etimer IWDG_timer;  
    
    etimer_set(&IWDG_timer, IWDG_INTERVAL);
  
    
    PROCESS_BEGIN();    
    
    while(1){
    PROCESS_WAIT_EVENT();
    if(ev==PROCESS_EVENT_TIMER)   
    {
        HAL_IWDG_Refresh(&hiwdg);	 
    } 
    
        etimer_restart(&IWDG_timer);  
  }
     
  
    PROCESS_END();  
}


/**
  * @brief  WWDG Early Wakeup callback.
  * @param  hwwdg  pointer to a WWDG_HandleTypeDef structure that contains
  *                the configuration information for the specified WWDG module.
  * @retval None
  */
 void HAL_WWDG_EarlyWakeupCallback(WWDG_HandleTypeDef* hwwdg)
{
  /* Prevent unused argument(s) compilation warning */
  hwwdg->Init.Counter = 127;
  
  printf("HAL_WWDG_EarlyWakeupCallback \r\n");
  if (HAL_WWDG_Init(hwwdg) != HAL_OK)
  {
      printf("EWI watch dog MX_WWDG_Init failed \r\n");  
  }   
  //UNUSED(hwwdg);

  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_WWDG_EarlyWakeupCallback could be implemented in the user file
   */
}



/*---------------------------------------------------------------------------*/
PROCESS_THREAD(battery_process, ev, data)
{

  static struct etimer battery_timer;  
    
  etimer_set(&battery_timer, LOOP_INTERVAL*10);
  PROCESS_BEGIN();   
 
  while(1)
  {
      PROCESS_WAIT_EVENT(); 
      if(ev==PROCESS_EVENT_TIMER)
      {
         adcValue=HAL_ADC_GetValue(&hadc);
         adcValue=adcValue-2482;
         BatValue=(adcValue*100)/1241;//battery Reference Voltage: 3V,ADC Reference Volgate: 3.6V
         if(BatValue>100&& BatValue <130)
         {
             BatValue=100;
         }else if(BatValue>130)
         {
             BatValue=0;
         }
         
         
      }    
      etimer_restart(&battery_timer); 
  }
  PROCESS_END();  
  
  
}


/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  //volatile int i;
 // adcValue=HAL_ADC_GetValue(hadc);
 // BatValue=(adcValue*100)/4096;
 // HAL_ADC_Start_DMA(hadc,&adcValue,1);   
  //printf("adc Value:%d\r\n",adcValue);
 
  
}  



/**
* @}
*/
