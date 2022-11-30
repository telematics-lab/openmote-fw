/**
 * @file       LedBlinker.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2016
 * @brief
 *
 * @copyright  Copyright 2016, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/


#include "Scheduler.hpp"
#include "LedBlinker.hpp"

#include "Gpio.hpp"

/*================================ define ===================================*/

/*================================ typedef ==================================*/

/*=============================== prototypes ================================*/

/*=============================== variables =================================*/

/*================================= public ==================================*/

LedBlinker::LedBlinker(const char * const name, uint16_t size, uint8_t priority, GpioOut& gpio):
	Thread(name, size, priority), gpio_(gpio), period_(0), timeOn_(0)
{
}

void LedBlinker::setTime(uint32_t timeOn, uint32_t period)
{
	timeOn_ = timeOn;
	period_ = period;
}

void LedBlinker::run(void) 
{
  /* Forever */
  while (true)
  {
    /* Turn LED on and wait */
    gpio_.on();
    Scheduler::delay_ms(timeOn_);
    
    /* Turn LED off and wait */
    gpio_.off();
    Scheduler::delay_ms(period_ - timeOn_);
  }
}

/*================================ private ==================================*/
