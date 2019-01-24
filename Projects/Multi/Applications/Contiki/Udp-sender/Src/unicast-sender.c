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

#define UDP_PORT 1234
#define MESSAGE_SIZE 20
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
/*---------------------------------------------------------------------------*/
PROCESS(unicast_sender_process, "Unicast sender example process");
AUTOSTART_PROCESSES(&unicast_sender_process);
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
  printf("Data received on port %d from port %d with length %d\n",
         receiver_port, sender_port, datalen);
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
PROCESS_THREAD(unicast_sender_process, ev, data)
{
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

  ctimer_set(&send_timer, APP_DUTY_CYCLE_SLOT * CLOCK_SECOND, periodic_sender, NULL);

  while(1) {
    PROCESS_WAIT_EVENT();
    if(ev == sensors_event && data == &button_sensor ){
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
    }
  }
  PROCESS_END();
}
/**
* @}
*/
