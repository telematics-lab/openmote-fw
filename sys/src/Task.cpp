 /**
 * @file       Task.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       November, 2018
 * @brief
 *
 * @copyright  Copyright 2018, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include "Task.hpp"

/*================================ define ===================================*/

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

Task::Task(const char* const name, const uint16_t size, uint8_t priority, TaskFunction_t function, void * const parameters):
  name_(name), priority_(priority), function_(function), parameters_(parameters)
{
  xTaskCreate(Task::init, name_, size, this, priority_, &handle_);
}

void Task::init(void * params)
{
  if (params != nullptr)
  {
    Task* task = (Task *)params;
    task->function_(task->parameters_);
  }
}

/*=============================== protected =================================*/

/*================================ private ==================================*/
