/**
 * @file       Semaphore.hpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       November, 2018
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef SEMAPHORE_HPP_
#define SEMAPHORE_HPP_

#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

class Semaphore
{
public:
  Semaphore(void);
  ~Semaphore(void);
  bool isTaken(void) const;
  bool take(uint32_t milliseconds = 0) const;
  void give(void) const;
  void giveFromInterrupt(void) const;
protected:
  SemaphoreHandle_t semaphore_;
};

class SemaphoreBinary : public Semaphore
{
public:
  SemaphoreBinary(bool given = true);
};

class SemaphoreCounting : public Semaphore
{
public:
  SemaphoreCounting(uint32_t initialCount, uint32_t maxCount);
};

#endif /* SEMAPHORE_HPP_ */
