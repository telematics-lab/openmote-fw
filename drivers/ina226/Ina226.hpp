/**
 * @file       Ina226.hpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       February, 2019
 * @brief
 *
 * @copyright  Copyright 2019, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef INA226_HPP_
#define INA226_HPP_

#include "Gpio.hpp"
#include "I2c.hpp"
#include "Callback.hpp"

class Ina226;

typedef GenericCallback<Ina226> Ina226Callback;

struct Ina226Config
{
  uint16_t config;
  uint16_t calibration;
};

struct Ina226Data
{
  uint16_t shunt;
  uint16_t bus;
  uint16_t power;
  uint16_t current;
};

class Ina226
{

friend class InterruptHandler;

#define INA226_MEASURE_SHUNT    ( 0x01 )
#define INA226_MEASURE_BUS      ( 0x02 )
#define INA226_MEASURE_POWER    ( 0x04 )
#define INA226_MEASURE_CURRENT  ( 0x08 )
#define INA226_MEASURE_ALL      ( 0x0F )

public:
  Ina226(I2c& i2c, GpioIn& irq);
  void init(uint8_t address);
  bool reset(void);
  bool check(void);
  bool configure(Ina226Config& config);
  bool start(void);
  bool stop(void);
  bool read(Ina226Data& data, uint8_t mask, uint16_t timeout_ms);
  bool test(void);
protected:
  void interruptHandler(void);
private:
  bool readRegister(uint8_t address, uint16_t* result);
private:
  I2c& i2c_;
  GpioIn& irq_;
  uint8_t address_;
  Ina226Callback callback_;
  SemaphoreBinary semaphore_;
};

#endif /* INA226_HPP_ */
