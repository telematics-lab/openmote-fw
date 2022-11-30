/**
 * @file       Opt3001.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include <math.h>

#include "Task.hpp"

#include "Opt3001.hpp"

#include "I2c.hpp"

/*================================ define ===================================*/

/* Register addresses */
#define REG_RESULT                      ( 0x00 )
#define REG_CONFIGURATION               ( 0x01 )
#define REG_LOW_LIMIT                   ( 0x02 )
#define REG_HIGH_LIMIT                  ( 0x03 )

/* Manufacturer/device values */
#define REG_MANUFACTURER_ID             ( 0x7E )
#define REG_DEVICE_ID                   ( 0x7F )

/* Register values */
#define MANUFACTURER_ID_MSB             ( 0x54 )
#define MANUFACTURER_ID_LSB             ( 0x49 )
#define DEVICE_ID_MSB                   ( 0x30 )
#define DEVICE_ID_LSB                   ( 0x01 )
#define CONFIG_RESET_MSB                ( 0xC8 )
#define CONFIG_RESET_LSB                ( 0x10 )
#define CONFIG_TEST_MSB                 ( 0xCC )
#define CONFIG_TEST_LSB                 ( 0x10 )
#define CONFIG_ENABLE_MSB               ( 0xC4 )
#define CONFIG_ENABLE_LSB               ( 0x10 )
#define CONFIG_DISABLE_MSB              ( 0xC0 )
#define CONFIG_DISABLE_LSB              ( 0x10 )

/* Bit values */
#define DATA_RDY_BIT_MSB                ( 0x00 )
#define DATA_RDY_BIT_LSB                ( 0x80 )

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

Opt3001::Opt3001(I2c& i2c, uint8_t address):
    i2c_(i2c), address_(address)
{
}

bool Opt3001::init(void)
{
    bool success;

    success = disable();

    return success;
}

bool Opt3001::enable(void)
{
    uint8_t scratch[3];
    bool success;

    /* Prepare write buffer */
    scratch[0] = REG_CONFIGURATION;
    scratch[1] = CONFIG_ENABLE_MSB;
    scratch[2] = CONFIG_ENABLE_LSB;

    /* Obtain the mutex of the I2C driver */
    i2c_.lock();

    /* Write the enable configuration */
    success = i2c_.writeByte(address_, scratch, sizeof(scratch));
    if (!success)
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

bool Opt3001::disable(void)
{
    uint8_t scratch[3];
    bool success;

    /* Prepare write buffer */
    scratch[0] = REG_CONFIGURATION;
    scratch[1] = CONFIG_DISABLE_MSB;
    scratch[2] = CONFIG_DISABLE_LSB;

    /* Obtain the mutex of the I2C driver */
    i2c_.lock();

    /* Write the disable configuration */
    success = i2c_.writeByte(address_, scratch, sizeof(scratch));
    if (!success)
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

bool Opt3001::read(uint16_t* raw)
{
    uint8_t scratch[2];
    bool success;

    /* Obtain the mutex of the I2C driver */
    i2c_.lock();

    /* Set the read register */
    success = i2c_.writeByte(address_, REG_CONFIGURATION);
    if (!success)
    {
        goto error;
    }

    /* Read the register */
    success = i2c_.readByte(address_, scratch, sizeof(scratch));
    if (!success)
    {
        goto error;
    }

    /* Check if data is ready */
    success = ((scratch[1] & DATA_RDY_BIT_LSB) == DATA_RDY_BIT_MSB);
    if (!success)
    {
        goto error;
    }

    /* Set the read register */
    success = i2c_.writeByte(address_, REG_RESULT);
    if (!success)
    {
        goto error;
    }

    /* Read the data register */
    success = i2c_.readByte(address_, scratch, sizeof(scratch));
    if (!success)
    {
        goto error;
    }

    /* Copy result */
    *raw  = ((scratch[0] << 8) | (scratch[1] << 0));

    /* Release the mutex of the I2C driver */
    i2c_.unlock();

    return true;

error:
    /* Release the mutex of the I2C driver */
    i2c_.unlock();

    return false;
}

void Opt3001::convert(uint16_t raw, float* lux)
{
    uint16_t e;
    uint16_t m;

    m = raw & 0x0FFF;
    e = (raw & 0xF000) >> 12;

    *lux = m * (0.01 * exp2(e));
}

bool Opt3001::test(void)
{
    uint8_t scratch[2];
    bool success;

    /* Obtain the mutex of the I2C driver */
    i2c_.lock();

    /* Set the read register */
    success = i2c_.writeByte(address_, REG_MANUFACTURER_ID);
    if (!success)
    {
        goto error;
    }

    /* Perform I2C read transaction */
    success = i2c_.readByte(address_, scratch, sizeof(scratch));
    if (!success)
    {
        goto error;
    }

    /* Check manufacturer ID */
    if ((scratch[0] != MANUFACTURER_ID_MSB) ||
        (scratch[1] != MANUFACTURER_ID_LSB))
    {
        goto error;
    }

    /* Set the read register */
    success = i2c_.writeByte(address_, REG_DEVICE_ID);
    if (!success)
    {
        goto error;
    }

    /* Perform I2C read transaction */
    success = i2c_.readByte(address_, scratch, sizeof(scratch));
    if (!success)
    {
        goto error;
    }

    /* Check device ID */
    if ((scratch[0] != DEVICE_ID_MSB) ||
        (scratch[1] != DEVICE_ID_LSB))
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

