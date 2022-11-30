/**
 * @file       LedBlinker.hpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2016
 * @brief
 *
 * @copyright  Copyright 2016, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef LED_BLINKER_HPP_
#define LED_BLINKER_HPP_

#include "Thread.hpp"

class GpioOut;

class LedBlinker : protected Thread
{
public:
  LedBlinker(const char* const name, uint16_t size, uint8_t priority, GpioOut& gpio);
  void setTime(uint32_t timeOn, uint32_t period);
protected:
  virtual void run(void);
private:
  GpioOut& gpio_;
  uint32_t period_;
  uint32_t timeOn_;
};

#endif /* LED_BLINKER_HPP_ */
