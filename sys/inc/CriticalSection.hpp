/**
 * @file       CriticalSection.hpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef CRITICAL_SECTION_HPP_
#define CRITICAL_SECTION_HPP_

#include "FreeRTOS.h"
#include "task.h"

class CriticalSection
{
public:
  CriticalSection(bool interrupt = false);
  ~CriticalSection(void);
private:
  bool interrupt_;
  UBaseType_t uxSavedInterruptStatus;
};

#endif /* CRITICAL_SECTION_HPP_ */
