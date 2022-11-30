/**
 * @file       GpioIn.cpp
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
#include "InterruptHandler.hpp"

#include "platform_includes.h"
#include "platform_types.hpp"

/*================================ define ===================================*/

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

GpioIn::GpioIn(const GpioConfig& config):
  Gpio(config), callback_(nullptr)
{
  /* Set the pin as input */
  GPIOPinTypeGPIOInput(config_.port, config_.pin);

  /* Set the edge of the interrupt */
  GPIOIntTypeSet(config_.port, config_.pin, config_.edge);

  /* Configure as peripheral */
  if (config_.ioc != IOC_OVERRIDE_DIS)
  {
    IOCPadConfigSet(config_.port, config_.pin, config_.ioc);
  }
}

GpioInPow::GpioInPow(const GpioConfig& config):
  GpioIn(config)
{
  /* Set the pin as input */
  GPIOPinTypeGPIOInput(config_.port, config_.pin);

  /* Set the edge of the interrupt */
  GPIOPowIntTypeSet(config_.port, config_.pin, config_.edge);

  /* Enable the interrupt wakeup capability of the port */
  if(config_.port == GPIO_A_BASE)
  {
    GPIOIntWakeupEnable(GPIO_IWE_PORT_A);
  }
  else if(config_.port == GPIO_B_BASE)
  {
    GPIOIntWakeupEnable(GPIO_IWE_PORT_B);
  }
  else if(config_.port == GPIO_C_BASE)
  {
    GPIOIntWakeupEnable(GPIO_IWE_PORT_C);
  }
  else if(config_.port == GPIO_D_BASE)
  {
    GPIOIntWakeupEnable(GPIO_IWE_PORT_D);
  }
  else
  {
    while(true)
      ;
  }
}

bool GpioIn::read(void)
{
  uint32_t state;
  
  /* Read the pin value */
  state = GPIOPinRead(config_.port, config_.pin);
  
  return (bool)state;
}

void GpioIn::setCallback(Callback* callback)
{
  /* Save the pointer to the callback function */
  callback_ = callback;

  /* Get a reference to the interruptHandler object */
  InterruptHandler& interruptHandler = InterruptHandler::getInstance();

  /* Register to the interruptHandler by passing a pointer to the object */
  interruptHandler.setInterruptHandler(*this);
}

void GpioIn::clearCallback(void)
{
  /* Clear the pointer to the callback function */
  callback_ = nullptr;
}

void GpioIn::enableInterrupts(void)
{
  /* Clear the interrupt */
  GPIOPinIntClear(config_.port, config_.pin);

  /* Enable the interrupt */
  GPIOPinIntEnable(config_.port, config_.pin);
}

void GpioIn::disableInterrupts(void)
{
  /* Disable the interrupt */
  GPIOPinIntDisable(config_.port, config_.pin);
}

void GpioInPow::enableInterrupts(void)
{
  /* Clear the power interrupt */
  GPIOPowIntClear(config_.port, config_.pin);

  /* Enable the power intrrupt */
  GPIOPowIntEnable(config_.port, config_.pin);
}

void GpioInPow::disableInterrupts(void)
{
  /* Enable the power intrrupt */
  GPIOPowIntDisable(config_.port, config_.pin);
}

void GpioIn::interruptHandler(void)
{
  /* Call the interrupt handler if it is NOT null */
  if (callback_ != nullptr)
  {
      callback_->execute();
  }
}

/*=============================== protected =================================*/

/*================================ private ==================================*/
