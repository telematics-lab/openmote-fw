/**
 * @file       Ina226.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2016
 * @brief
 *
 * @copyright  Copyright 2016, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include "Ina226.hpp"
#include "Ina226_regs.h"

#include "BoardImplementation.hpp"

#include "platform_types.h"

/*================================ define ===================================*/

#define INA226_ADDRESS      ( 0x40 )

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

extern BoardImplementation board;

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

Ina226::Ina226(I2c& i2c, GpioIn& irq):
  i2c_(i2c), irq_(irq), callback_(this, &Ina226::interruptHandler), semaphore_(false)
{
  irq_.setCallback(&callback_);
}

void Ina226::init(uint8_t address)
{ 
  address_ = address;
}

bool Ina226::reset(void)
{
  uint8_t buffer[3];
  bool status;

  buffer[0] = INA226_CONFIG;
  buffer[1] = 0xC0;
  buffer[2] = 0x00;

  /* Obtain the mutex of the I2C driver */
  i2c_.lock();

  /* Force software reset*/
  status = i2c_.writeByte(address_, buffer, 3);
  if (!status)
  {
    goto error;
  }

  /* Release the mutex of the I2C driver */
  i2c_.unlock();

  return true;

error:
  i2c_.unlock();
  return false;
}

bool Ina226::check(void)
{
  return false;
}

bool Ina226::configure(Ina226Config& config)
{
  uint8_t buffer[3];
  bool status;

  /* Split configuration */
  buffer[0] = INA226_CONFIG;
  buffer[1] = (uint8_t) ((config.config & 0xFF00) >> 8);
  buffer[2] = (uint8_t) ((config.config & 0x00FF) >> 0);

  /* Obtain the mutex of the I2C driver */
  i2c_.lock();

  /* Write chip configuration */
  status = i2c_.writeByte(address_, buffer, 3);
  if (!status)
  {
    goto error;
  }

  /* Split configuration */
  buffer[0] = INA226_CALIBRATION;
  buffer[1] = (uint8_t) ((config.calibration & 0xFF00) >> 8);
  buffer[2] = (uint8_t) ((config.calibration & 0x00FF) >> 0);

  /* Write calibration configuration */
  status = i2c_.writeByte(address_, buffer, 3);
  if (!status)
  {
    goto error;
  }

  /* Release the mutex of the I2C driver */
  i2c_.unlock();

  return true;

error:
  /* Release the mutex of the I2C driver */
  i2c_.unlock();
  return false;
}

bool Ina226::start(void)
{
  uint8_t buffer[3];
  bool status;

  buffer[0] = INA226_MASK_ENABLE;
  buffer[1] = 0x04;
  buffer[2] = 0x00;

  /* Obtain the mutex of the I2C driver */
  i2c_.lock();

  /* Write status */
  status = i2c_.writeByte(address_, buffer, 3);
  if (!status)
  {
    goto error;
  }

  /* Start device operation */
  irq_.enableInterrupts();

  /* Release the mutex of the I2C driver */
  i2c_.unlock();

  return true;
error:
  /* Release the mutex of the I2C driver */
  i2c_.unlock();

  return false;
}

bool Ina226::stop(void)
{
  uint8_t buffer[3];
  bool status;

  /* Stop device operation */
  irq_.disableInterrupts();

  buffer[0] = INA226_MASK_ENABLE;
  buffer[1] = 0x00;
  buffer[2] = 0x00;

  /* Obtain the mutex of the I2C driver */
  i2c_.lock();

  /* Write configuration */
  status = i2c_.writeByte(address_, buffer, 3);
  if (!status)
  {
    goto error;
  }

  /* Release the mutex of the I2C driver */
  i2c_.unlock();

  return true;
error:
  /* Release the mutex of the I2C driver */
  i2c_.unlock();

  return false;
}

bool Ina226::read(Ina226Data& data, uint8_t mask, uint16_t timeout_ms)
{
  uint16_t scratch;
  bool status;
  
  /* Try to get a measurement */
  status = semaphore_.take(timeout_ms);
  if (status != true)
  {
    return false;
  }

  /* Obtain the mutex of the I2C driver */
  i2c_.lock();

  /* Read mask/enable register to clear conversion ready flag */
  status = readRegister(INA226_MASK_ENABLE, &scratch);
  if (!status)
  {
    goto error;
  }
  
  /* Measure shunt voltage */
  if (mask & INA226_MEASURE_SHUNT == INA226_MEASURE_SHUNT)
  {
    status = readRegister(INA226_SHUNT_VOLTAGE, &data.shunt);
    if (!status)
    {
      goto error;
    }
  }
  
  /* Measure bus voltage */
  if (mask & INA226_MEASURE_BUS == INA226_MEASURE_BUS)
  {
    status = readRegister(INA226_SHUNT_VOLTAGE, &data.bus);
    if (!status)
    {
      goto error;
    }
  }
  
  /* Measure power */
  if (mask & INA226_MEASURE_POWER == INA226_MEASURE_POWER)
  {
    status = readRegister(INA226_POWER, &data.power);
    if (!status)
    {
      goto error;
    }
  }
  
  /* Measure current */
  if (mask & INA226_MEASURE_CURRENT == INA226_MEASURE_CURRENT)
  {
    status = readRegister(INA226_CURRENT, &data.current);
    if (!status)
    {
      goto error;
    }
  }

  /* Release the mutex of the I2C driver */
  i2c_.unlock();

  return true;
error:
  /* Release the mutex of the I2C driver */
  i2c_.unlock();
  return false;
}

bool Ina226::test(void)
{
  uint8_t buffer[3];
  bool status;

  buffer[0] = INA226_CONFIG;
  
  /* Obtain the mutex of the I2C driver */
  i2c_.lock();

  /* Read Configuration register */
  status = i2c_.readByte(address_, buffer, 3);
  if (!status)
  {
    goto error;
  }

  /* Check default configuration */
  if (buffer[1] != INA226_CONFIG_DEFAULT_MSB ||
      buffer[2] != INA226_CONFIG_DEFAULT_LSB)
  {
    goto error;
  }

  
  buffer[0] = INA226_MANUFACTURER_ID;
  
  /* Read Manufacturer register */
  status = i2c_.readByte(address_, buffer, 3);
  if (!status)
  {
    goto error;
  }

  /* Check manufacturer ID */
  if (buffer[1] != INA226_MANUFACTURER_ID_MSB ||
      buffer[2] != INA226_MANUFACTURER_ID_LSB)
  {
    goto error;
  }

  /* Release the mutex of the I2C driver */
  i2c_.unlock();

  return true;

error:
  /* Release the mutex of the I2C driver */
  i2c_.unlock();
  return false;
}

/*=============================== protected =================================*/

/*================================ private ==================================*/

bool Ina226::readRegister(uint8_t address, uint16_t* result)
{
  uint8_t buffer[2];
  bool status;
  
  /* Write the register address */
  status = i2c_.writeByte(address_, address);
  if (!status)
  {
    return false;
  }
  
  /* Read the resuter value */
  status = i2c_.readByte(address_, buffer, 2);
  if (!status)
  {
    return false;
  }
  
  /* Convert the result */
  *result = (buffer[0] << 8 | buffer[1] << 0);
  
  return true;
}

void Ina226::interruptHandler(void)
{
  semaphore_.giveFromInterrupt();
}
