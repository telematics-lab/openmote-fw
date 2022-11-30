/**
 * @file       GpioAdc.cpp
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

GpioAdc::GpioAdc(const GpioConfig& gpioConfig, AdcConfig& adcConfig):
    Gpio(gpioConfig), callback_(nullptr), adcConfig_(adcConfig)
{
  /* Configure pin as hardware */
  GPIODirModeSet(config_.port, config_.pin, GPIO_DIR_MODE_HW);

  /* Configure ADC with resolution and reference */
  SOCADCSingleConfigure(adcConfig_.resolution, adcConfig_.reference);
}

void GpioAdc::start()
{
  /* Trigger single conversion on internal temperature sensor */
  SOCADCSingleStart(adcConfig_.channel);
}

void GpioAdc::poll(void)
{
  /* Wait until conversion completes */
  while(!SOCADCEndOfCOnversionGet())
      ;
}

uint32_t GpioAdc::read(void)
{
  uint32_t shift;
  uint32_t mask;
  uint32_t value;

  /* Select shift/mask based on ADC resolution */
  switch (adcConfig_.resolution)
  {
    case SOCADC_7_BIT: 
      shift = SOCADC_7_BIT_RSHIFT;
      mask  = SOCADC_7_BIT_MASK;
      break;
    case SOCADC_9_BIT: 
      shift = SOCADC_9_BIT_RSHIFT;
      mask  = SOCADC_9_BIT_MASK;
      break;
    case SOCADC_10_BIT: 
      shift = SOCADC_10_BIT_RSHIFT;
      mask  = SOCADC_10_BIT_MASK;
      break;
    case SOCADC_12_BIT: 
      shift = SOCADC_12_BIT_RSHIFT;
      mask  = SOCADC_12_BIT_MASK;
      break;
    default:
      while(true);
      break;
  }

  /* Get the ADC value, mask it and shift it according to resolution */
  value = (SOCADCDataGet() & mask ) >> shift;

  return value;
}

void GpioAdc::setCallback(Callback* callback)
{
  /* Save the pointer to the callback function */
  callback_ = callback;

  /* Get a reference to the interruptHandler object */
  InterruptHandler& interruptHandler = InterruptHandler::getInstance();

  /* Register to the interruptHandler by passing a pointer to the object */
  interruptHandler.setInterruptHandler(*this);
}

void GpioAdc::clearCallback(void)
{
  /* Clear the pointer to the callback function */
  callback_ = nullptr;
}

void GpioAdc::enableInterrupts(void)
{
  /* Enable the interrupt */
  IntEnable(INT_ADC0);
}

void GpioAdc::disableInterrupts(void)
{
  /* Disable the interrupt */
  IntDisable(INT_ADC0);
}

void GpioAdc::interruptHandler(void)
{
  /* Call the interrupt handler if it is NOT null */
  if (callback_ != nullptr) {
    callback_->execute();
  }
}

/*=============================== protected =================================*/

/*================================ private ==================================*/
