/**
 * @file       Board.hpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       November, 2018
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef BOARD_HPP_
#define BOARD_HPP_

#include <stdint.h>

#include "Callback.hpp"

#include "platform_types.h"

enum SleepMode : uint8_t;

class Board {
public:
  Board();
  void init(const BoardParams& boardParams);
  void reset(void);
  void error(void);
  uint32_t getClock(void);
  void setSleepMode(SleepMode sleepMode);
  void sleep(void);
  void wakeup(void);
  void enableInterrupts(void);
  void disableInterrupts(void);
  uint32_t getCurrentTicks(void);
  bool isExpiredTicks(uint32_t futureTicks);
  void delayMicroseconds(uint32_t milliseconds);
  void delayMicroseconds2(uint32_t microseconds);
  void delayMilliseconds(uint32_t milliseconds);
  void getEUI48(uint8_t* address);
  void getEUI64(uint8_t* address);
private:
  void setup();
public:
  static const uint32_t BOARD_TICKS_PER_US;
private:
  SleepMode sleepMode_;
};

class BoardImplementation : public Board {
public:
  BoardImplementation(const BoardParams &params);
  void init(void);
private:
  const BoardParams& boardParams_;
};

#endif /* BOARD_HPP_ */
