/**
 * @file       TemperatureSensor.hpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       January, 2016
 * @brief
 *
 * @copyright  Copyright 2016, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include "TemperatureSensor.hpp"

#include "platform_includes.h"

/*================================ define ===================================*/

#define CONST                   ( 0.58134 )
#define OFFSET_DATASHEET_25C    ( 827 )
#define TEMP_COEFF              ( CONST * 4.2 )
#define OFFSET_0C               ( OFFSET_DATASHEET_25C - ( 25 * TEMP_COEFF ) )

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

TemperatureSensor::TemperatureSensor(void):
  temperature_(0)
{
}

bool TemperatureSensor::enable(void)
{
  // Enable RF Core (needed to enable temp sensor) */
  SysCtrlPeripheralEnable(SYS_CTRL_PERIPH_RFC);
  
  // Connect temp sensor to ADC */
  HWREG(CCTEST_TR0) |= CCTEST_TR0_ADCTM;
  
  // Enable the temperature sensor */
  HWREG(RFCORE_XREG_ATEST) = 0x01;
  
  // Configure ADC, Internal reference, 512 decimation rate (12 bit) */
  SOCADCSingleConfigure(SOCADC_12_BIT, SOCADC_REF_INTERNAL);

  return true;
}

bool TemperatureSensor::disable(void) {
  // Enable the temperature sensor */
  HWREG(RFCORE_XREG_ATEST) = 0x00;
  
  // Connect temp sensor to ADC */
  HWREG(CCTEST_TR0) ^= CCTEST_TR0_ADCTM;

  // Enable RF Core (needed to enable temp sensor) */
  SysCtrlPeripheralDisable(SYS_CTRL_PERIPH_RFC);

  return true;
}

bool TemperatureSensor::suspend(void) {
    return false;
}

bool TemperatureSensor::wakeup(void) {
    return false;
}

bool TemperatureSensor::reset(void) {
    return false;
}

bool TemperatureSensor::isPresent(void) {
    return true;
}

bool TemperatureSensor::readTemperature(void) {    
  /* Trigger single conversion on internal temp sensor */
  SOCADCSingleStart(SOCADC_TEMP_SENS);
  
  /* Wait until conversion is completed */
  while(!SOCADCEndOfCOnversionGet())
    ;

  /* Get data and shift down based on decimation rate */
  temperature_ = SOCADCDataGet() >> SOCADC_12_BIT_RSHIFT;
  
  return true;
}

float TemperatureSensor::getTemperature(void) {
  float result;

  result = (((temperature_ * CONST) - OFFSET_0C) / TEMP_COEFF);

  return result;
}

uint16_t TemperatureSensor::getTemperatureRaw(void)
{
  return temperature_;
}

/*=============================== protected =================================*/

/*================================ private ==================================*/
