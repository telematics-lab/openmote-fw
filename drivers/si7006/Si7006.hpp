/**
 * @file       Si7006.hpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef SI7006_HPP_
#define SI7006_HPP_

#include "I2c.hpp"

class Si7006
{
public:
    Si7006(I2c& i2c);
    bool enable(void);
    bool suspend(void){return false;}
    bool wakeup(void){return false;}
    bool reset(void);
    bool isPresent(void);
    bool read(void);
    bool readTemperature(void);
    bool readHumidity(void);
    float getTemperature(void);
    uint16_t getTemperatureRaw(void);
    float getHumidity(void);
    uint16_t getHumidityRaw(void);
private:
    void isInitialized(void);
private:
    I2c& i2c_;

    uint16_t temperature;
    uint16_t humidity;
};

#endif /* SI7006_HPP_ */
