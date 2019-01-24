/******************************************************************************/
/**
 * @file    fifo.c
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

/*******************************************************************************
 * Include Files
 */
#include <string.h>
#include "fifo.h"


/******************************************************************************/
/**
 * @brief Initialize the FIFO.
 *
 * @param  p_fifo  A pointer to a structure to maintain FIFO data.
 */
void fifo_init(fifo_t* p_fifo)
{
//    __disable_irq();

    if (p_fifo)
    {
        p_fifo->count = 0;
        p_fifo->front = 0;
        p_fifo->back  = 0;
        memset((void*)p_fifo->p_data, 0, p_fifo->size);
    }

//    __enable_irq();
}

/******************************************************************************/
/**
 * @brief Queue a byte at the back of the FIFO.
 *
 * @param  p_fifo  A pointer to the FIFO structure to queue data to.
 * @param  data  The data byte to add to the FIFO.
 */
 void fifo_queue(fifo_t* p_fifo, uint8_t data)
{
//    __disable_irq();

    p_fifo->p_data[p_fifo->back] = data;

    p_fifo->back++;
    p_fifo->back %= p_fifo->size;

    if (p_fifo->count < p_fifo->size)
    {
        p_fifo->count++;
    }
    else
    {
        p_fifo->front++;
        p_fifo->front %= p_fifo->size;
    }

//    __enable_irq();
}

/******************************************************************************/
/**
 * @brief Dequeue and return a pointer the next byte the front of the FIFO.
 *
 * @param  p_fifo  A pointer to the FIFO structure to dequeue data from.
 *
 * @return A pointer to the dequeued data, else 0 if none.
 */
 uint8_t fifo_dequeue(fifo_t* p_fifo, uint8_t* p_data)
{
    uint8_t result = 0;
    
//    __disable_irq();

    if (p_fifo->count)
    {
        *p_data = p_fifo->p_data[p_fifo->front];

        p_fifo->count--;
        p_fifo->front++;
        p_fifo->front %= p_fifo->size;
        result = 1;
    }

//    __enable_irq();

    return result;
}

/******************************************************************************/
/**
 * @brief Get the count of bytes remaining in the FIFO.
 *
 * @param  p_fifo  A pointer to the FIFO structure to get the size of.
 *
 * @return The count of bytes remaining in the FIFO.
 */
 uint16_t fifo_get_count(const fifo_t* p_fifo)
{
    return p_fifo->count;
}
