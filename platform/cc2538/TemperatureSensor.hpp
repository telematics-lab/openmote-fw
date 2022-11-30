/**
 * @file       platfrom_temperature.hpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       November, 2018
 * @brief
 *
 * @copyright  Copyright 2018, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef TEMPERATURE_SENSOR_HPP_
#define TEMPERATURE_SENSOR_HPP_

#include <stdint.h>
#include <stdbool.h>

class TemperatureSensor
{
public:
  TemperatureSensor();
  bool enable(void);
  bool disable(void);
  bool suspend(void);
  bool wakeup(void);
  bool reset(void);
  bool isPresent(void);
  bool readTemperature(void);
  uint16_t getTemperatureRaw(void);
  float getTemperature(void);
private:
  uint16_t temperature_;
};

#endif /* TEMPERATURE_SENSOR_HPP_ */

