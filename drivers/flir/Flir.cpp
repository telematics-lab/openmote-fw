/**
 * @file       Flir.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2019
 * @brief
 *
 * @copyright  Copyright 2019, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include "Flir.hpp"
#include "Flir_regs.h"

#include "BoardImplementation.hpp"

/*================================ define ===================================*/

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

static uint8_t buffer_header[FLIR_HEADER_SIZE];

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

Flir::Flir(Spi& spi, GpioOut& cs):
  spi_(spi), cs_(cs)
{
}

void Flir::init(void)
{
}

void Flir::wakeup(void)
{
  cs_.low();
  board.delayMilliseconds(1);
  cs_.high();
  board.delayMilliseconds(200);
}

bool Flir::readFirst(uint8_t* buffer, uint16_t length)
{
  bool is_synchronized = false;
  
  do
  {
    /* Read VoSPI packet from SPI */
    readPacket(buffer, FLIR_DATA_SIZE);
    
    /* Packet is not discard packet and packet number is 0 */
    if (!isDiscardPacket())
    {
        is_synchronized = true;
    }
  } while (!is_synchronized);
  
  return is_synchronized;
}

void Flir::readRemaining(uint8_t* buffer, uint16_t length)
{  
  uint8_t* ptr;
  
  /* Read the remaining packets of the frame */
  for (uint8_t i = 0; i < FLIR_ROW_SIZE - 1; i++)
  {
  	/* Point to the next frame line */ 
    ptr = (buffer + i * FLIR_DATA_SIZE);
    
    /* Read the next packet of the frame */
    readPacket(ptr, FLIR_DATA_SIZE);
  }
}

/*=============================== protected =================================*/

/*================================ private ==================================*/

bool Flir::readPacket(uint8_t* buffer_payload, uint16_t length)
{
  /* Check that the length is valid */
  if (length < FLIR_DATA_SIZE)
  {
    return false;
  }
  
  /* Read VoSPI header and data from SPI */
  cs_.low();
  spi_.rwByteDma(nullptr, 0, buffer_header, FLIR_HEADER_SIZE);
  spi_.rwByteDma(nullptr, 0, buffer_payload, length);
  cs_.high();
  
  return true;
}

bool Flir::isDiscardPacket(void)
{ 
  /* Check if it is not a discard packet */
  if (((buffer_header[0] & 0x0F) != 0x0F) && 
  	   (buffer_header[1] == 0x00))
  {
    return false;
  }
  
  return true;
}
