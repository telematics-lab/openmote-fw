/**
 * @file       Scheduler.hpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2016
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef SCHEDULER_HPP_
#define SCHEDULER_HPP_

#include <stdint.h>

class Scheduler
{
public:
  static void run(void);
  static void yield(void);
  static uint32_t get_ticks(void);
  static uint32_t get_ms(void);
  static void delay_ms(uint32_t ms);
private:
	Scheduler(void);
};

#endif /* SCHEDULER_HPP_ */
