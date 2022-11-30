 /**
 * @file       Scheduler.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2016
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include "Scheduler.hpp"

#include "FreeRTOS.h"
#include "task.h"

/*================================ define ===================================*/

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

Scheduler::Scheduler(void)
{
}

void Scheduler::run(void)
{
  vTaskStartScheduler();
}

void Scheduler::yield(void)
{
  taskYIELD();
}

uint32_t Scheduler::get_ticks(void)
{
  return xTaskGetTickCount();
}

uint32_t Scheduler::get_ms(void)
{
  TickType_t ticks = xTaskGetTickCount();
  return ticks / portTICK_PERIOD_MS;
}

void Scheduler::delay_ms(uint32_t ms)
{
  TickType_t ticks = pdMS_TO_TICKS(ms);
  vTaskDelay(ticks);
}

/*=============================== protected =================================*/

/*================================ private ==================================*/
