/**
******************************************************************************
  * @file    st-utils.c
  * @author  Central LAB
  * @version V1.0.0
  * @date    23-January-2017
  * @brief   Various 6LoWPAN related utility functions.
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
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
/*---------------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include "net/ip/ip64-addr.h"
#include "net/rpl/rpl.h"
#include "st-utils.h"
/*---------------------------------------------------------------------------*/
static rpl_dag_t *dag = NULL;
/*---------------------------------------------------------------------------*/
void
st_utils_print_local_addresses(void)
{
  int i;
  uint8_t state;

  printf("IPv6 addresses:\n");
  for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
    state = uip_ds6_if.addr_list[i].state;
    if(uip_ds6_if.addr_list[i].isused &&
       (state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
      printf(" ");
      uip_debug_ipaddr_print(&uip_ds6_if.addr_list[i].ipaddr);

      printf("\n");
    }
  }
}
/*---------------------------------------------------------------------------*/
/* Gets the first address, ok for now. */
int
st_utils_get_address_literal(char * address_literal)
{
  int addr_idx;
  uint8_t state;
  static uip_ipaddr_t *my_address = NULL;
  static int i = 0, j = 0;
  uint16_t val = 0;

  for(addr_idx = 0; addr_idx < UIP_DS6_ADDR_NB; addr_idx++) {
    state = uip_ds6_if.addr_list[addr_idx].state;
    if(uip_ds6_if.addr_list[addr_idx].isused &&
       (state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
    	my_address = &uip_ds6_if.addr_list[addr_idx].ipaddr;
    	break;
    }
  }
  if (my_address == NULL) return 0;

  j = 0;
  for (i=0;i<16;i+=2){
    val = (my_address->u8[i] << 8) + my_address->u8[i+1];
    j += sprintf(address_literal+j, "%x", val);
    if (i<14){
    	address_literal[j]=':';
	  j++;
    }
  }
  address_literal[j++]='\0';

  return 1;
}
/*---------------------------------------------------------------------------*/
/*
 * Converts a local address by putting the RPL prefix.
 */
int
st_utils_local_to_prefix_address(uip_ipaddr_t* prefix_p, uip_ipaddr_t* local_p)
{
    dag = rpl_get_any_dag();
    if (dag == NULL) return 0;

	memcpy(prefix_p, &(dag->prefix_info.prefix),dag->prefix_info.length/8);
	memcpy((uint8_t *)prefix_p + dag->prefix_info.length/8, (uint8_t *)local_p+dag->prefix_info.length/8, 16-dag->prefix_info.length/8);

	return 1;
}
/*---------------------------------------------------------------------------*/
/*
 * Returns RPL parent (with global prefix) in uip_ipaddr_t format.
 */
int
st_utils_get_parent_address(uip_ipaddr_t* parent_with_prefix_p)
{
    static uip_ipaddr_t *parent_local_p = NULL;

    dag = rpl_get_any_dag();
    if (dag == NULL) return 0;
	//gets the link local address
	parent_local_p = rpl_get_parent_ipaddr(dag->preferred_parent);

	if (parent_local_p == NULL) return 0;

	memcpy(parent_with_prefix_p, &(dag->prefix_info.prefix),dag->prefix_info.length/8);
	memcpy((uint8_t *)parent_with_prefix_p + dag->prefix_info.length/8, (uint8_t *)parent_local_p+dag->prefix_info.length/8, 16-dag->prefix_info.length/8);

	return 1;
}
/*---------------------------------------------------------------------------*/
/*
 * Returns RPL parent (with global prefix) in literal, non compressed
 * format, i.e.:
 * "aaaa:0:0:0:1234:5678:9abc:def0"
 * "aaaa:0:0:0:123:4567:89ab:cdef"
 */
int
st_utils_get_parent_literal(char* parent_literal)
{
	static uip_ipaddr_t parent_with_prefix;
    static int i = 0, j = 0;
    uint16_t val = 0;

    if (!st_utils_get_parent_address(&parent_with_prefix)){
    	return 0;
    }

    j = 0;
	for (i=0;i<16;i+=2){
	  val = (parent_with_prefix.u8[i] << 8) + parent_with_prefix.u8[i+1];
	  j += sprintf(parent_literal+j, "%x", val);
	  if (i<14){
		  parent_literal[j]=':';
		  j++;
	  }
	}
	parent_literal[j++]='\0';

	return 1;
}
/*---------------------------------------------------------------------------*/
/*
 * Returns RPL parent (with global prefix) in literal compressed
 * format, i.e.:
 * "aaaa::1234:5678:9abc:def0"
 * "aaaa::123:4567:89ab:cdef"
 */
int
st_utils_get_parent_literal_cmp(char* parent_literal_cmp)
{
	static uip_ipaddr_t parent_address;
	if (st_utils_get_parent_address(&parent_address)){
		st_utils_ipaddr_sprintf(parent_literal_cmp, &parent_address);
		return 1;
	}
	return 0;
}
/*---------------------------------------------------------------------------*/
/*
 * "sprintf" version of uip_debug_ipaddr_print(), prints on 'literal_cmp'
 */
void
st_utils_ipaddr_sprintf(char * literal_cmp, const uip_ipaddr_t *addr)
{
#if NETSTACK_CONF_WITH_IPV6
  uint16_t a;
  unsigned int i;
  int f;
  static int cnt = 0;
#endif /* NETSTACK_CONF_WITH_IPV6 */
  if(addr == NULL) {
    printf("(NULL IP addr)");
    return;
  }
#if NETSTACK_CONF_WITH_IPV6
  if(ip64_addr_is_ipv4_mapped_addr(addr)) {
    sprintf(literal_cmp, "::FFFF:%u.%u.%u.%u", addr->u8[12], addr->u8[13], addr->u8[14], addr->u8[15]);
  } else {
	cnt = 0;
    for(i = 0, f = 0; i < sizeof(uip_ipaddr_t); i += 2) {
      a = (addr->u8[i] << 8) + addr->u8[i + 1];
      if(a == 0 && f >= 0) {
        if(f++ == 0) {
          literal_cmp[cnt++]=':';
          literal_cmp[cnt++]=':';
        }
      } else {
        if(f > 0) {
          f = -1;
        } else if(i > 0) {
          literal_cmp[cnt++]=':';
        }
        cnt += sprintf(literal_cmp + cnt, "%x", a);
      }
	}
    literal_cmp[cnt++]='\0';
  }
#else /* NETSTACK_CONF_WITH_IPV6 */
  sprintf(literal_cmp, "%u.%u.%u.%u", addr->u8[0], addr->u8[1], addr->u8[2], addr->u8[3]);
#endif /* NETSTACK_CONF_WITH_IPV6 */
}
/*---------------------------------------------------------------------------*/
