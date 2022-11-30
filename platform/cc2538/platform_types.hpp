/**
 * @file       platform_types.hpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef PLATFORM_TYPES_HPP_
#define PLATFORM_TYPES_HPP_

/*================================ include ==================================*/

#include "platform_types.h"

/*================================ define ===================================*/

/*================================ typedef ==================================*/

/**
 * SleepMode_None:   ~2 mA,   0 + 0 us,   wake-up from any interrupt source (e.g. UART)
 * SleepMode_1:    ~600 uA, 0.5 + 4 us,   wake-up from Gpio, Sleep timer, USB resume
 * SleepMode_2:    ~1.5 uA, 136 + 136 us, wake-up from Gpio, Sleep timer
 * SleepMode_3:    ~0.8 uA, 136 + 136 us, wake-up from Gpio
 */
enum SleepMode : uint8_t
{
  SleepMode_None = SYS_CTRL_PM_NOACTION,
  SleepMode_1    = SYS_CTRL_PM_1,
  SleepMode_2    = SYS_CTRL_PM_2,
  SleepMode_3    = SYS_CTRL_PM_3
};

class AdcConfig
{
public:
  uint32_t resolution;
  uint32_t reference;
  uint32_t channel;
};

class GpioConfig
{
public:
  bool operator==(const GpioConfig& other) {return (port == other.port) && (pin == other.pin);}
public:
  uint32_t port;
  uint32_t pin;
  uint32_t ioc;
  uint32_t edge;
  uint32_t invert;
};

class TimerConfig
{
public:
  bool operator==(const TimerConfig& other) {return (peripheral == other.peripheral);}
public:
  uint32_t peripheral;
  uint32_t base;
  uint32_t source;
  uint32_t config;
  uint32_t interrupt;
  uint32_t interrupt_mode;
};

class I2cConfig
{
public:
  bool operator==(const I2cConfig& other) {return (peripheral == other.peripheral);}
public:
  uint32_t peripheral;
  uint32_t baudrate;
};

class SpiConfig
{
public:
  bool operator==(const SpiConfig& other) {return (peripheral == other.peripheral);}
public:
  uint32_t peripheral;
  uint32_t base;
  uint32_t clock;
  uint32_t interrupt;
  uint32_t mode;
  uint32_t protocol;
  uint32_t datawidth;
  uint32_t baudrate;
};

class UartConfig
{
public:
  bool operator==(const UartConfig& other) {return (peripheral == other.peripheral);}
public:
  uint32_t peripheral;
  uint32_t base;
  uint32_t clock;
  uint32_t interrupt;
  uint32_t baudrate;
  uint32_t mode;
};

/*=============================== variables =================================*/

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

/*================================ private ==================================*/

#endif  /* PLATFORM_TYPES_HPP_ */
