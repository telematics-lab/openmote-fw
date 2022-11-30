/**
 * @file       CriticalSection.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include "CriticalSection.hpp"

/*================================ define ===================================*/

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

CriticalSection::CriticalSection(bool interrupt):
  interrupt_(interrupt)
{
  if (!interrupt_)
  {
    taskENTER_CRITICAL();
  }
  else
  {
    uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
  }
}

CriticalSection::~CriticalSection(void)
{
  if (!interrupt_)
  {
    taskEXIT_CRITICAL();
  }
  else
  {
    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
  }  
}

/*=============================== protected =================================*/

/*================================ private ==================================*/

