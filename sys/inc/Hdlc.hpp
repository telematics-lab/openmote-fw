/**
 * @file       Hdlc.hpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef HDLC_HPP_
#define HDLC_HPP_

#include <stdint.h>
#include <stdbool.h>

#include "Buffer.hpp"
#include "Crc16.hpp"

enum HdlcResult : int32_t
{
  HdlcResult_Error = -1,
  HdlcResult_Ok    =  0,
};

enum HdlcStatus : int32_t
{
  HdlcStatus_Idle  =  1,
  HdlcStatus_Busy  =  2,
  HdlcStatus_Done  =  3,
};

class Hdlc
{
public:
  Hdlc(Buffer& buffer);

  HdlcResult rxOpen(void);
  HdlcResult rxPut(uint8_t byte);
  HdlcResult rxPut(Buffer& buffer);
  HdlcResult rxClose(void);
  HdlcStatus getRxStatus(void);

  HdlcResult txOpen(void);
  HdlcResult txPut(uint8_t byte);
  HdlcResult txPut(Buffer& buffer);
  HdlcResult txClose(void);

private:
  HdlcResult rxParse(uint8_t byte);

private:
  Buffer& buffer_;
  Crc16 crc_;

  HdlcStatus rxStatus;
  uint8_t rxLastByte;
  bool rxIsEscaping;
};

#endif /* HDLC_HPP_ */
