/**
 * @file       GpioOut.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       November, 2018
 * @brief
 *
 * @copyright  Copyright 2018, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include "Gpio.hpp"

#include "platform_includes.h"
#include "platform_types.hpp"

/*================================ define ===================================*/

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

GpioOut::GpioOut(const GpioConfig& config):
  Gpio(config)
{
  /* Set the pin as output */
  GPIOPinTypeGPIOOutput(config_.port, config_.pin);

  /* By default the pin is off */
  off();
}

void GpioOut::on(void)
{
  if (config_.invert == 1)
  {
    /* Write a zero to set pin on */
    GPIOPinWrite(config_.port, config_.pin, 0);
  }
  else
  {
    /* Write a one to set pin on */
    GPIOPinWrite(config_.port, config_.pin, config_.pin);
  }
}

void GpioOut::off(void)
{
  if (config_.invert == 1)
  {
    /* Write a one to set pin off */
    GPIOPinWrite(config_.port, config_.pin, config_.pin);
  }
  else
  {
    /* Write a zero to set pin off */
    GPIOPinWrite(config_.port, config_.pin, 0);
  }
}

void GpioOut::toggle(void)
{
  /* Read the old status of the pin */
  uint32_t status = GPIOPinRead(config_.port, config_.pin);

  /* Change the status of the pin */
  status = (~status) & config_.pin;

  /* Set the new status of the pin */
  GPIOPinWrite(config_.port, config_.pin, status);
}

void GpioOut::high(void)
{
  /* Write a one to set pin high */
  GPIOPinWrite(config_.port, config_.pin, config_.pin);
}

void GpioOut::low(void)
{
  /* Write a zero to set pin low */
  GPIOPinWrite(config_.port, config_.pin, 0);
}

uint32_t GpioOut::status(void)
{
  /* Read the pin status */
  uint32_t status = GPIOPinRead(config_.port, config_.pin);

  /* Return the pin status */
  return (status & config_.pin);
}

/*=============================== protected =================================*/

/*================================ private ==================================*/
