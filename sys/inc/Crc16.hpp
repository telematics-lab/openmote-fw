/**
 * @file       Crc16.hpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef CRC16_HPP_
#define CRC16_HPP_

#include <stdint.h>

class Crc16
{
public:
  Crc16();
  void init(void);
  uint16_t get(void);
  void set(uint8_t byte);
  bool check(void);
private:
  uint16_t crc_;
};

#endif /* CRC16_HPP_ */
