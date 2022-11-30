/**
 * @file       Thread.hpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       November, 2018
 * @brief
 *
 * @copyright  Copyright 2018, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef THREAD_HPP_
#define THREAD_HPP_

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

class Thread
{
public:
  Thread(const char* const name, const uint16_t size, uint8_t priority);
public:
  static void init(void* params);
protected:
  virtual void run(void) = 0;
protected:
  TaskHandle_t handle_;
	const char* const name_;
  UBaseType_t priority_;
};

#endif /* THREAD_HPP_ */
