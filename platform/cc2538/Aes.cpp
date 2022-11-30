/**
 * @file       Aes.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2016
 * @brief
 *
 * @copyright  Copyright 2016, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include <string.h>

#include "Aes.hpp"
#include "InterruptHandler.hpp"

#include "Semaphore.hpp"

#include "platform_includes.h"
#include "platform_types.hpp"

/*================================ define ===================================*/

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

Aes::Aes(void):
  enabled_(false)
{
}

void Aes::enable(void)
{
	if (enabled_)
  { 
    /* Disable */
    SysCtrlPeripheralDisable(SYS_CTRL_PERIPH_AES);
  }
	
  /* Enable */
  SysCtrlPeripheralEnable(SYS_CTRL_PERIPH_AES);
  SysCtrlPeripheralSleepEnable(SYS_CTRL_PERIPH_AES);
  SysCtrlPeripheralDeepSleepDisable(SYS_CTRL_PERIPH_AES);
  
  /* Get a reference to the interruptHandler object */
  InterruptHandler& interruptHandler = InterruptHandler::getInstance();

  /* Register to the interruptHandler by passing a pointer to the object */
  interruptHandler.setInterruptHandler(*this);
  
  /* Reset */
  SysCtrlPeripheralReset(SYS_CTRL_PERIPH_AES);    
  
  enabled_ = true;
}

void Aes::reset(void)
{
	enabled_ = false;
}

bool Aes::sleep(void)
{
	return true;
}

bool Aes::wakeup(void)
{
	loadKey(key_, 16);

	return true;
}

bool Aes::loadKey(uint8_t* key, uint16_t length)
{
	uint8_t status;
  
  if (length != 16)
  {
    return false;
  }

	/* Store the key in non-volatile RAM */
	memcpy(key_, key, 16);

	/* Load the key at a given location */
	status = AESLoadKey((uint8_t *) key_, KEY_AREA_0);
	
	/* Check for AES status */
	if (status != AES_SUCCESS)
	{
		return false;
	}
	else
	{
		return true;
	}
}

bool Aes::encrypt(uint8_t* input, uint8_t* output, uint16_t* length)
{
	/* Encrypt, payload should be multiple of 16! */
	return Aes::processBuffer(input, output, length, true);
}

bool Aes::decrypt(uint8_t* input, uint8_t* output, uint16_t* length)
{
	/* Decrypt, payload should be multiple of 16 */
  return Aes::processBuffer(input, output, length, false);
}

/*=============================== protected =================================*/

void Aes::interruptHandler(void)
{
}

/*================================ private ==================================*/

bool Aes::processBuffer(uint8_t* input, uint8_t* output, uint16_t* length, bool encrypt)
{
    uint8_t blocks;
    bool status = false;

    /* Number blocks has to be multiple of 16 */
    if (encrypt)
    {
      blocks = (*length >> 4) + 1;
    }
    else
    {
      blocks = (*length >> 4);
    }
    
    /* For each block encrypt or decrypt */
    for (uint8_t n = 0; n < blocks; n++) {
      uint8_t scratch[16] = {0};
      uint16_t position;
      
      /* Calculate buffer position */
      position = 16 * n;
      
      /* Copy input buffer to scratch buffer, zero-pad to be multiple of 16 */
      memcpy(scratch, &input[position], 16);
      
      /* Encrypt or decrypt from scratch to output buffer */
      status = Aes::processBlock(scratch, &output[position], KEY_AREA_0, encrypt);
      if (!status) break;
    }
    
    /* Update the length status to be multiple of 16 */
    if (status && encrypt)
    {
      *length = blocks << 4;
    }

    return status;
}

bool Aes::processBlock(uint8_t* input, uint8_t* output, uint8_t key, bool encrypt)
{
	/* Encrypt a data block */
	if (AESECBStart(input, output, key, encrypt, false) == AES_SUCCESS)
	{
    /* Wait for operation to complete */
		do
    {
			ASM_NOP;
		} while (!(AESECBCheckResult()));
    
    /* Check operation result */
		if (AESECBGetResult() == AES_SUCCESS)
		{
			return true;
		}
	}

	return false;
}
