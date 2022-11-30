/**
 * @file       Task.hpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       November, 2018
 * @brief
 *
 * @copyright  Copyright 2018, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef TASK_HPP_
#define TASK_HPP_

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

class Task
{
public:
  Task(const char* const name, const uint16_t size, uint8_t priority, TaskFunction_t function, void * const parameters);
public:
  static void init(void* params);
private:
  TaskHandle_t handle_;
  const char* const name_;
  UBaseType_t priority_;
  TaskFunction_t function_;
  void *parameters_;
};

#endif /* TASK_HPP_ */
