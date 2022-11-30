/**
 * @file       Serial.hpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef SERIAL_HPP_
#define SERIAL_HPP_

#include <stdint.h>
#include <stdbool.h>

#include "Uart.hpp"

#include "Buffer.hpp"
#include "Hdlc.hpp"
#include "Mutex.hpp"

#define SERIAL_BUFFER_TX_LENGTH       ( 256 )
#define SERIAL_BUFFER_RX_LENGTH       ( 256 )

class Serial;

typedef GenericCallback<Serial> SerialCallback;

class Serial
{
public:
  Serial(Uart& uart);
  void init(void);
  void write(uint8_t* data, uint32_t length, bool useDma = false);
  uint32_t read(uint8_t* buffer, uint32_t length);
public:
  bool operator==(const Serial& other);
private:
  void rxCallback(void);
  void txCallback(void);
private:
  Uart& uart_;

  uint8_t rx_buffer_[SERIAL_BUFFER_RX_LENGTH];
  Buffer rxBuffer_;

  uint8_t tx_buffer_[SERIAL_BUFFER_TX_LENGTH];
  Buffer txBuffer_;
  
  Mutex txMutex_;
  Mutex rxMutex_;

  SerialCallback rxCallback_;
  SerialCallback txCallback_;
  
  bool useDma_;
  bool rxError_;
};

#endif /* SERIAL_HPP_ */
