/**
 * @file       platform_timer.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       November, 2018
 * @brief
 *
 * @copyright  Copyright 2018, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include "BoardImplementation.hpp"

#include "InterruptHandler.hpp"
#include "Timer.hpp"

#include "platform_includes.h"
#include "platform_types.hpp"

/*================================ define ===================================*/

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

extern BoardImplementation board;

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

Timer::Timer(const TimerConfig& config):
    config_(config), callback_(nullptr), prescaler_(1)
{
}

void Timer::init(void)
{
  /* Disable peripheral previous to configuring it */
  TimerDisable(config_.base, config_.source);

  /* Enable peripheral except in deep sleep modes (e.g. LPM1, LPM2, LPM3) */
  SysCtrlPeripheralEnable(config_.peripheral);
  SysCtrlPeripheralSleepEnable(config_.peripheral);
  SysCtrlPeripheralDeepSleepDisable(config_.peripheral);

  /* Configure the peripheral */
  TimerConfigure(config_.base, config_.config);
}

void Timer::setPrescaler(uint32_t prescaler)
{
  /* Prescaler is only available for individual timers */
  if (config_.source != GPTIMER_BOTH)
  {
    return;
  }
    
  /* Prescaler is 8-bits */
  if (prescaler > 255)
  {
      prescaler = 255;
  }
  
  /* Remember prescaler value */
  prescaler_ = prescaler;
  
  /* Configure the prescaler */
  TimerPrescaleSet(config_.base, config_.source, prescaler_);
}

void Timer::setFrequency(uint32_t milliseconds)
{
  uint32_t clock, frequency;

  /* Get the system clock */
  clock = board.getClock();

  /* Calculate the frequency given the system clock and expected frequency */
  frequency = (clock) / (milliseconds * prescaler_);

  /* Set the timer to the given frequency */
  TimerLoadSet(config_.base, config_.source, frequency);
}

void Timer::setPeriod(uint32_t microseconds)
{
  uint32_t ticks, ticks_us;
  
  /* Calculate the number of ticks/microsecond based on the system clock */
  ticks_us = board.getClock() / (1000000 * prescaler_);
  
  /* Calculate the number of ticks for the given period */
  ticks = (ticks_us * microseconds);

  /* Set the timer to the given number of ticks */
  TimerLoadSet(config_.base, config_.source, ticks);
}

void Timer::start(void)
{
  TimerEnable(config_.base, config_.source);
}

void Timer::stop(void)
{
  TimerDisable(config_.base, config_.source);
}

uint32_t Timer::read(void)
{
  return TimerValueGet(config_.base, config_.source);
}

void Timer::setCallback(Callback* callback)
{
  callback_ = callback;
}

void Timer::clearCallback(void)
{
  callback_ = nullptr;
}

void Timer::enableInterrupts(void)
{
  InterruptHandler::getInstance().setInterruptHandler(*this);

  /* Clear Timer interrupts */
  TimerIntClear(config_.base, config_.interrupt_mode);

  /* Enable Timer interrupts */
  TimerIntEnable(config_.base, config_.interrupt_mode);

  /* Set the Timer interrupt priority */
  IntPrioritySet(config_.interrupt, 0xF0);

  /* Enable the Timer interrupt */
  IntEnable(config_.interrupt);
}

void Timer::disableInterrupts(void)
{
  /* Disable Timer interrupts */
  TimerIntDisable(config_.base, config_.interrupt_mode);

  /* Disable the Timer interrupt */
  IntDisable(config_.interrupt);
}

/*=============================== protected =================================*/

const TimerConfig& Timer::getConfig(void)
{
  return config_;
}

void Timer::interruptHandler(void)
{
  if (callback_ != nullptr)
  {
      callback_->execute();
  }
}

/*================================ private ==================================*/
