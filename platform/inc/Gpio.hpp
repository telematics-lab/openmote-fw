/**
 * @file       Gpio.hpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef GPIO_HPP_
#define GPIO_HPP_

#include <stdint.h>

#include "Callback.hpp"
#include "Timer.hpp"

struct GpioConfig;
struct AdcConfig;

class Gpio
{
public:
  Gpio(const GpioConfig& config);
  const GpioConfig& getGpioConfig(void);
public:
  bool operator==(const Gpio& other);
protected:
  const GpioConfig& config_;
};

/*****************************************************************************/

class GpioIn : public Gpio
{

friend class InterruptHandler;

public:
  GpioIn(const GpioConfig& config);
  bool read(void);
  void setCallback(Callback* callback);
  void clearCallback(void);
  void enableInterrupts(void);
  void disableInterrupts(void);
protected:
  void interruptHandler(void);
protected:
  Callback* callback_;
};

/*****************************************************************************/

class GpioInPow : public GpioIn
{

friend class InterruptHandler;

public:
  GpioInPow(const GpioConfig& config);
  void enableInterrupts(void);
  void disableInterrupts(void);
};

/*****************************************************************************/

class GpioOut : public Gpio
{

public:
  GpioOut(const GpioConfig& config);
  void on(void);
  void off(void);
  void toggle(void);
  void high(void);
  void low(void);
  uint32_t status(void);
};

/*****************************************************************************/

class GpioPwm : public GpioOut, Timer
{

public:
  GpioPwm(const GpioConfig& gpioConfig, TimerConfig& timerConfig);
  void init(bool level);
  void setCallback(Callback* callback);
  void clearCallback(void);
  void enableInterrupts(void);
  void disableInterrupts(void);
  void setFrequency(uint16_t frequency);
  void setDutyCycle(uint16_t dutyCycle);
  void start(void);
  void stop(void);
private:
  uint16_t dutyCycle_;
};

/*****************************************************************************/

class GpioAdc: public Gpio
{

friend class InterruptHandler;

public:
  GpioAdc(const GpioConfig& gpioConfig, AdcConfig& adcConfig);
  void start(void);
  void poll(void);
  uint32_t read(void);
  void setCallback(Callback* callback);
  void clearCallback(void);
  void enableInterrupts(void);
  void disableInterrupts(void);
protected:
  void interruptHandler(void);
protected:
  Callback* callback_;
private:
  AdcConfig& adcConfig_;
};

/*****************************************************************************/

#endif /* GPIO_HPP_ */
