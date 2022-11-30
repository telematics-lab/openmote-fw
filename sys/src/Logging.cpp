/**
 * @file       Console.cpp
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

#include "CriticalSection.hpp"
#include "Logging.hpp"

#include "Uart.hpp"

/*================================ define ===================================*/

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

Logging::Logging(Uart& uart) :
	uart_(uart), txCallback_(this, &Logging::txCallback),
  transmit_buffer_{0}, txBuffer_(transmit_buffer_, sizeof(transmit_buffer_)),
	level_(LogLevel::None), initialized_(false)
{
}

void Logging::init(void) 
{
	CriticalSection cs;

	if (!initialized_)
  {
    /* Enable the UART */
    uart_.enable();

    /* Register UART callbacks */
    uart_.setTxCallback(&txCallback_);

    /* Enable UART interrupts */
    uart_.enableInterrupts();

    /* Mark as initialized */
    initialized_ = true;
	}
}

void Logging::setLevel(LogLevel level)
{	
	CriticalSection cs;

	level_ = level;
}

void Logging::log(char const * message)
{
	bool status;
	uint8_t byte;
	uint32_t length;

	/* Take the logging mutex */
	mutex_.take();

	/* Take the UART lock */
	uart_.txLock();

	/* Count number of characters */
	length = strlen(message);

	/* Reset the transmit buffer */
	txBuffer_.reset();

	/* Write characters into transmit buffer */
	txBuffer_.writeBytes((uint8_t *) message, length);

 	/* Read first byte from the UART transmit buffer */
  status = txBuffer_.readByte(&byte);
  if (status == true)
  {
    uart_.writeByte(byte);
  }
  else
  {
    uart_.txUnlock();
  }
}

void Logging::call(char const * message)
{
	if (level_ >= LogLevel::Call)
  {
		log(message);
	}
}

void Logging::info(char const * message)
{
	if (level_ >= LogLevel::Info)
  {
		log(message);
	}
}

void Logging::warn(char const * message)
{
	if (level_ >= LogLevel::Warn)
  {
		log(message);
	}
}

void Logging::error(char const * message)
{
	if (level_ >= LogLevel::Error)
  {
		log(message);
	}
}

/*=============================== protected =================================*/

/*================================ private ==================================*/

void Logging::txCallback(void)
{
	uint8_t byte;

  /* Read byte from the UART transmit buffer */
  if (txBuffer_.readByte(&byte) == true)
  {
    /* Write byte to the UART */
    uart_.writeByte(byte);
  }
  else
  {
    /* Once done, free the UART lock */
    uart_.txUnlockFromInterrupt();

    /* Give the logging mutex */
    mutex_.give();
  }
}
