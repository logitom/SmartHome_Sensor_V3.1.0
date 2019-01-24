/**
******************************************************************************
  * @file    st-utils.h
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
#ifndef _ST_UTILS_H_
#define _ST_UTILS_H_
/*---------------------------------------------------------------------------*/
#define ADDRESS_LITERAL_LEN     50
/*---------------------------------------------------------------------------*/
/*
 * Converts a local address by putting the RPL prefix.
 */
int
st_utils_local_to_prefix_address(uip_ipaddr_t* prefix_p, uip_ipaddr_t* local_p);
/*---------------------------------------------------------------------------*/
/*
 * Returns RPL parent (with global prefix) in uip_ipaddr_t format.
 */
int
st_utils_get_parent_address(uip_ipaddr_t* parent_with_prefix);
/*---------------------------------------------------------------------------*/
/*
 * Returns RPL parent (with global prefix) in literal, non compressed
 * format, i.e.:
 * "aaaa:0:0:0:1234:5678:9abc:def0"
 * "aaaa:0:0:0:123:4567:89ab:cdef"
 */
int
st_utils_get_parent_literal(char* parent_literal);
/*---------------------------------------------------------------------------*/
/*
 * Returns RPL parent (with global prefix) in literal compressed
 * format, i.e.:
 * "aaaa::1234:5678:9abc:def0"
 * "aaaa::123:4567:89ab:cdef"
 */
int
st_utils_get_parent_literal_cmp(char* parent_literal_cmp);
/*---------------------------------------------------------------------------*/
/*
 * "sprintf" version of uip_debug_ipaddr_print(), prints on 'literal_cmp' 
 */
void
st_utils_ipaddr_sprintf(char * literal_cmp, const uip_ipaddr_t *addr);
/*---------------------------------------------------------------------------*/
void st_utils_print_local_addresses (void);
/*---------------------------------------------------------------------------*/
/* Returns first IP address. */
int
st_utils_get_address_literal(char * address_literal);
#endif /*_ST_UTILS_H_*/
