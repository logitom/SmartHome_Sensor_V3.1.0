/////******************************************************************************/
/**
 * @file    fifo.h
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
 * A generic byte FIFO implementation.
 */

#ifndef __FIFO_H__
#define __FIFO_H__

/*******************************************************************************
 * Include Files
 */
#include <stdint.h>

/*******************************************************************************
 * Type Definitions
 */
typedef struct
{
    uint16_t  size;
    uint8_t*  p_data;
    uint16_t  count;
    uint16_t  front;
    uint16_t  back;
} fifo_t;

/*******************************************************************************
 * Functions
 */
void fifo_init(fifo_t* p_fifo);
 void fifo_queue(fifo_t* p_fifo, uint8_t data);
 uint8_t fifo_dequeue(fifo_t* p_fifo, uint8_t* p_data);
 uint16_t fifo_get_count(const fifo_t* p_fifo);

#endif
