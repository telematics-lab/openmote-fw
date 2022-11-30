 /**
 * @file       Mutex.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include "Mutex.hpp"

/*================================ define ===================================*/

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

Mutex::Mutex(void)
{
  mutex_ = xSemaphoreCreateMutex();
  if (mutex_ == NULL)
  {
    while (true)
      ;
  }
}

Mutex::~Mutex(void)
{
  vSemaphoreDelete(mutex_);
}

bool Mutex::take(void)
{
  bool status;
  status = (xSemaphoreTake(mutex_, portMAX_DELAY) == pdTRUE);
  return status;
}

bool Mutex::take(uint32_t milliseconds)
{
  bool status;
  TickType_t timeout = milliseconds / portTICK_RATE_MS;
  status = (xSemaphoreTake(mutex_, timeout) == pdTRUE);
  return status;
}

void Mutex::give(void)
{
  xSemaphoreGive(mutex_);
}

MutexRecursive::MutexRecursive(void)
{
  mutex_ = xSemaphoreCreateRecursiveMutex();
  if (mutex_ == NULL)
  {
    while (true)
      ;
  }
}

MutexRecursive::~MutexRecursive(void)
{
  vSemaphoreDelete(mutex_);
}

bool MutexRecursive::take(void)
{
  bool status;
  status = (xSemaphoreTakeRecursive(mutex_, portMAX_DELAY) == pdTRUE);
  return status;
}

bool MutexRecursive::take(uint32_t milliseconds)
{
  bool status;
  TickType_t timeout = milliseconds / portTICK_RATE_MS;
  status = (xSemaphoreTakeRecursive(mutex_, timeout) == pdTRUE);
  return status;
}

void MutexRecursive::give(void)
{
  xSemaphoreGiveRecursive(mutex_);
}

/*=============================== protected =================================*/

/*================================ private ==================================*/
