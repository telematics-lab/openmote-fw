/**
 * @file       Bme280.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include "Scheduler.hpp"
#include "Task.hpp"

#include "Bme280.hpp"

#include "bme280_bosch.h"

/*================================ define ===================================*/

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

Bme280::Bme280(I2c& i2c, uint8_t address):
    i2c_(i2c), address_(address)
{
}

bool Bme280::init(void)
{
    uint8_t settings_sel;
    int8_t result;

    /* Set-up device */
    dev.dev_id = address_;
    dev.intf = BME280_I2C_INTF;
    dev.bme280 = (void *) this;

    /* Initialize BME280 sensor */
    result = bme280_init(&dev);
    if (result < 0) {
    	return false;
    }

    /* Prepare recommended configuration */
    dev.settings.osr_h = BME280_OVERSAMPLING_1X;
    dev.settings.osr_p = BME280_OVERSAMPLING_16X;
    dev.settings.osr_t = BME280_OVERSAMPLING_2X;
    dev.settings.filter = BME280_FILTER_COEFF_16;

    /* Prepare measurement settings */
    settings_sel  = BME280_OSR_PRESS_SEL;
    settings_sel |= BME280_OSR_TEMP_SEL;
    settings_sel |= BME280_OSR_HUM_SEL;
    settings_sel |= BME280_FILTER_SEL;

    /* Set sensor settings */
    result = bme280_set_sensor_settings(settings_sel, &dev);
    if (result < 0)
    {
        return false;
    }

    return true;
}

bool Bme280::reset(void)
{
    int8_t status;

    /* Perform software reset */
    status = bme280_soft_reset(&dev);
    if (status != 0)
    {
        return false;
    }

    return true;
}

bool Bme280::read(Bme280Data* data)
{
    struct bme280_data comp_data;
    int8_t result;

	/* Set BME280 in normal operation mode */
    result = bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);
    if (result != 0) {
        return false;
    }

    /* Wait for measurement to complete */
    delay_ms(96);

    /* Convert BME280 measurement */
    result = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
    if (result != 0) {
        return false;
    }

    /* Scale and return sensor values */
    data->temperature = comp_data.temperature / 100.0f;
    data->humidity = comp_data.humidity / 1024.0f;
    data->pressure = comp_data.pressure / 100.0f;

    return true;
}

/*=============================== protected =================================*/

/*================================ private ==================================*/

int8_t Bme280::i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t* buffer, uint16_t length)
{
  bool result;

  /* Obtain the mutex of the I2C driver */
  i2c_.lock();

  if (length == 1)
  {
    result = i2c_.writeByte(dev_id, reg_addr);
    if (!result) goto error;

    result = i2c_.readByte(dev_id, buffer);
    if (!result) goto error;
  }
  else
  {
    result = i2c_.writeByte(dev_id, reg_addr);
    if (!result) goto error;

    result = i2c_.readByte(dev_id, buffer, length);
    if (!result) goto error;
  }

  /* Release the mutex of the I2C driver */
  i2c_.unlock();

  return 0;
error:
  /* Release the mutex of the I2C driver */
  i2c_.unlock();
  return -1;
}

int8_t Bme280::i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t* buffer, uint16_t length)
{
  uint8_t scratch[32];
  uint8_t scratch_len  = 0;
  bool status;

  /* Obtain the mutex of the I2C driver */
  i2c_.lock();

  scratch[scratch_len] = reg_addr;
  scratch_len++;

  for (uint8_t i = 0; i < length; i++)
  {
    scratch[scratch_len] = buffer[i];
    scratch_len++;
  }

  /* Write the scratch buffer */
  status = i2c_.writeByte(dev_id, scratch, scratch_len);
  if (!status) goto error;

  /* Release the mutex of the I2C driver */
  i2c_.unlock();

  return 0;

error:
  /* Release the mutex of the I2C driver */
  i2c_.unlock();
  return -1;
}

void Bme280::delay_ms(uint16_t ms)
{
  Scheduler::delay_ms(ms);
}
