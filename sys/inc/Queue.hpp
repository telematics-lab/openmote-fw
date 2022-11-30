/**
 * @file       Queue.hpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       November, 2018
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef QUEUE_HPP_
#define QUEUE_HPP_

#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

template <class T>
class Queue
{
public:
  Queue<T>(uint32_t elements);
  bool send(T* t, uint32_t timeout = 0);
  bool receive(T* t, uint32_t timeout = 0);
private:
  QueueHandle_t xQueue;
  uint8_t elements_;
};

template <class T>
Queue<T>::Queue(uint32_t elements):
  elements_(elements)
{
  /* Create a queue capable of holding elements_ of type T */
  xQueue = xQueueCreate(elements_, sizeof(T));
}

template <class T>
bool Queue<T>::send(T* t, uint32_t timeout)
{
  BaseType_t result;
  TickType_t xBlockTicks;
  
  /* Calculate the ticks to block */
  xBlockTicks = pdMS_TO_TICKS(timeout);

  /* Send data to queue by copying contets of pointer */
  result = xQueueSend(xQueue, (void *) t, xBlockTicks);
  
  /* Check if result is success */
  return (result == pdTRUE);
}

template <class T>
bool Queue<T>::receive(T* t, uint32_t timeout)
{
  BaseType_t result;
  TickType_t xBlockTicks; 
  
  xBlockTicks = pdMS_TO_TICKS(timeout);
  
  /* Receive data from queue by copying contents of pointer */
  result = xQueueReceive(xQueue, (void *) t, xBlockTicks);
  
  /* Check if result is success */
  return (result == pdTRUE);
}

#endif /* QUEUE_HPP_ */
