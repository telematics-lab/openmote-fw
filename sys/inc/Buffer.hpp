/**
 * @file       Buffer.hpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef BUFFER_HPP_
#define BUFFER_HPP_

#include <stdint.h>
#include <stdbool.h>

#include "Mutex.hpp"

class Buffer
{
public:
  Buffer(uint8_t* buffer, uint32_t length, uint32_t count = 0);
  void reset(void);
  uint8_t* getHead(void);
  uint32_t getCapacity(void);
  uint32_t getSize(void);
  bool isEmpty(void);
  bool isFull(void);
  bool readByte(uint8_t* data, bool interrupt = false);
  bool readBytes(uint8_t* buffer, uint32_t length, bool interrupt = false);
  bool writeByte(uint8_t data, bool interrupt = false);
  bool writeBytes(const uint8_t* data, uint32_t length, bool interrupt = false);
  bool copy(uint8_t* buffer, uint32_t length);
private:
  uint8_t* buffer_;
  uint32_t length_;
  uint32_t count_;
  uint8_t* head_;
  uint8_t* tail_;
};

#endif /* BUFFER_HPP_ */
