/**
 * @file       Semaphore.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include "Semaphore.hpp"

/*================================ define ===================================*/

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

Semaphore::Semaphore(void):
  semaphore_(nullptr)
{
}

Semaphore::~Semaphore(void)
{
  vSemaphoreDelete(semaphore_);
}

bool Semaphore::isTaken(void) const
{
  uint32_t count;

  count = uxSemaphoreGetCount(semaphore_);

  /* If semaphore is available */
  if (count > 0)
  {
    return false;
  }

  return true;
}

bool Semaphore::take(uint32_t milliseconds) const
{
  bool status;

  if (milliseconds != 0)
  {
    TickType_t timeout = milliseconds / portTICK_RATE_MS;
    status = (xSemaphoreTake(semaphore_, timeout) == pdTRUE);
  }
  else
  {
    status = (xSemaphoreTake(semaphore_, portMAX_DELAY) == pdTRUE);
  }
  
  return status;
}

void Semaphore::give(void) const
{
  xSemaphoreGive(semaphore_);
}

void Semaphore::giveFromInterrupt(void) const
{
  BaseType_t priorityTaskWoken_ = pdFALSE;
  xSemaphoreGiveFromISR(semaphore_, &priorityTaskWoken_);
  portYIELD_FROM_ISR(priorityTaskWoken_);
}

SemaphoreBinary::SemaphoreBinary(bool given)
{
  semaphore_ = xSemaphoreCreateBinary();
  if (semaphore_ == NULL)
  {
      while(true);
  }
  
  if (given)
  {
      Semaphore::give();
  }
}

SemaphoreCounting::SemaphoreCounting(uint32_t initialCount, uint32_t maxCount)
{
  semaphore_ = xSemaphoreCreateCounting(maxCount, initialCount);
  if (semaphore_ == NULL)
  {
    while(true);
  }
}

/*=============================== protected =================================*/

/*================================ private ==================================*/
