/**
 * @file       Crc16.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       February, 2019
 * @brief
 *
 * @copyright  Copyright 2019, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include "Crc16.hpp"

#include "platform_includes.h"
#include "platform_types.h"

/*================================ define ===================================*/

#define CRC16_SEED                  ( 0x0000 )
#define CRC16_OK                    ( 0x0000 )

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

Crc16::Crc16(void):
  crc_(CRC16_SEED)
{
}

void Crc16::init(void)
{
  /* CC2538 implements the CRC-16 BUYPASS with 0x8005 polynomial and 0x0000 seed */
  HWREG(SOC_ADC_RNDL) = 0x00;
  HWREG(SOC_ADC_RNDL) = 0x00;
}

uint16_t Crc16::get(void)
{
  uint8_t msb, lsb;
  msb = HWREG(SOC_ADC_RNDH);
  lsb = HWREG(SOC_ADC_RNDL);
  crc_ = (msb << 8) | (lsb << 0);
  return crc_;
}

void Crc16::set(uint8_t byte)
{
  HWREG(SOC_ADC_RNDH) = byte;
}

bool Crc16::check(void)
{
  uint8_t msb, lsb;
  msb = HWREG(SOC_ADC_RNDH);
  lsb = HWREG(SOC_ADC_RNDL);
  crc_ = (msb << 8) | (lsb << 0);
  return (crc_ == CRC16_OK);
}

/*=============================== protected =================================*/

/*================================ private ==================================*/
