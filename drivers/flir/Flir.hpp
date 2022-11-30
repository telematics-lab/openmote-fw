/**
 * @file       Flir.hpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       February, 2019
 * @brief
 *
 * @copyright  Copyright 2019, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef FLIR_HPP_
#define FLIR_HPP_

#include "Gpio.hpp"
#include "Spi.hpp"
#include "Callback.hpp"

class Flir
{
public:
	Flir(Spi& spi, GpioOut& cs);
  void init(void);
  void wakeup(void);
  bool readFirst(uint8_t* buffer, uint16_t length);
  void readRemaining(uint8_t* buffer, uint16_t length);
private:
  bool readPacket(uint8_t* buffer_payload, uint16_t length);
  bool isDiscardPacket(void);
private:
	Spi& spi_;
  GpioOut& cs_;
};

#endif /* FLIR_HPP_ */
