 /**
 * @file       Thread.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       November, 2018
 * @brief
 *
 * @copyright  Copyright 2018, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include "Thread.hpp"

/*================================ define ===================================*/

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

Thread::Thread(const char* const name, const uint16_t size, uint8_t priority):
  name_(name), priority_(priority)
{
  xTaskCreate(Thread::init, name_, size, this, priority_, &handle_);
}

void Thread::init(void * params)
{
  if (params != nullptr)
  {
    Thread* task = (Thread *)params;
    task->run();
  }
}

/*=============================== protected =================================*/

/*================================ private ==================================*/
