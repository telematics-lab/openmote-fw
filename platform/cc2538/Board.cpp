/**
 * @file       Board.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       November, 2018
 * @brief
 *
 * @copyright  Copyright 2018, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include <string.h>

#include "Gpio.hpp"
#include "Board.hpp"

#include "BoardImplementation.hpp"

#include "platform_includes.h"
#include "platform_types.hpp"

/*================================ define ===================================*/

/* Defines for the EUI64 addresses in Flash */
#define CC2538_EUI64_ADDRESS_HI_H               ( 0x0028002F )
#define CC2538_EUI64_ADDRESS_HI_L               ( 0x0028002C )
#define CC2538_EUI64_ADDRESS_LO_H               ( 0x0028002B )
#define CC2538_EUI64_ADDRESS_LO_L               ( 0x00280028 )

/* Defines the CCA address in Flash */
#define CC2538_FLASH_CCA_ADDRESS                ( 0x0027F800 )

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

extern GpioInPow button_user;

const uint32_t Board::BOARD_TICKS_PER_US = 31;

const uint32_t BOARD_BUTTON_DELAY_US    = 100000;
const uint32_t BOARD_BUTTON_DELAY_TICKS = (BOARD_BUTTON_DELAY_US / Board::BOARD_TICKS_PER_US);

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

Board::Board():
    sleepMode_(SleepMode_None)
{
}

void Board::init(const BoardParams& params)
{ 
  /* Configure the 32 kHz clock pins (PD6, PD7) for crystal operation */
  GPIODirModeSet(GPIO_D_BASE, GPIO_PIN_6, GPIO_DIR_MODE_IN);
  GPIODirModeSet(GPIO_D_BASE, GPIO_PIN_7, GPIO_DIR_MODE_IN);
  IOCPadConfigSet(GPIO_D_BASE, GPIO_PIN_6, IOC_OVERRIDE_ANA);
  IOCPadConfigSet(GPIO_D_BASE, GPIO_PIN_7, IOC_OVERRIDE_ANA);

  /* Set the system clocks */
  SysCtrlClockSet(params.bExternalOsc32k, params.bInternalOsc32M, params.SysClkDivider);

  /* Set the peripherals clock */
  SysCtrlIOClockSet(params.IOClkDivider);

  /* If using the 32 MHz crystal wait until it becomes stable */
  if (params.bInternalOsc32M == false)
  {
    while (!((HWREG(SYS_CTRL_CLOCK_STA)) & (SYS_CTRL_CLOCK_STA_XOSC_STB)))
      ;
  }

  /* If using the 32 kHz oscillator wait until it becomes stable */
  if (params.bExternalOsc32k == true)
  {
    while(HWREG(SYS_CTRL_CLOCK_STA) & SYS_CTRL_CLOCK_STA_SYNC_32K)
      ;
    while(!(HWREG(SYS_CTRL_CLOCK_STA) & SYS_CTRL_CLOCK_STA_SYNC_32K))
      ;
  }
}

void Board::reset(void)
{
  /* Force a software reset */
  SysCtrlReset();
}

void Board::error(void)
{
  while(true)
  {
    led_red.on();
    delayMilliseconds(100);
    led_red.off();
    delayMilliseconds(100);
  }
}

uint32_t Board::getClock(void)
{
  /* Return the current CPU clock */
  return SysCtrlClockGet();
}

void Board::setSleepMode(SleepMode sleepMode)
{
  /* Set the current sleep mode */
  sleepMode_ = sleepMode;
}

void Board::sleep(void)
{
  if (sleepMode_ == SleepMode_None)
  {
    /* Put the board to sleep */
    SysCtrlSleep();
  }
  else
  {
    /* Put the board to deep sleep */
    SysCtrlPowerModeSet(sleepMode_);
    SysCtrlDeepSleep();
  }
}

void Board::wakeup(void)
{
  /* Wake-up peripherals */
  if (sleepMode_ != SleepMode_None)
  {
    /* To-Do */
  }
}

void Board::enableInterrupts(void)
{
  /* Enable global interrupts */
  IntMasterEnable();
}

void Board::disableInterrupts(void)
{
  /* Disable global interrupts */
  IntMasterDisable();
}

uint32_t Board::getCurrentTicks(void)
{
  /* Get the current number of ticks */
  return SleepModeTimerCountGet();
}

bool Board::isExpiredTicks(uint32_t future)
{
  uint32_t current;
  int32_t remaining;

  /* Get the current number of ticks */
  current = getCurrentTicks();

  /* Calculte the number of remaining ticks */
  remaining = (int32_t) (future - current);

  /* Returns true if it has expired */
  return (remaining < 0);
}

void Board::delayMicroseconds(uint32_t microseconds)
{
  uint32_t current, timeout;

  if (microseconds <= Board::BOARD_TICKS_PER_US)
  {
    microseconds = 1;
  }
  
  /* Get the current number of ticks */
  current = getCurrentTicks();
  
  /* Calculate the time in the future */
  timeout = current + (microseconds / Board::BOARD_TICKS_PER_US);

  /* Active-wait until we reach the timeout */
  while(!isExpiredTicks(timeout))
      ;
}

void Board::delayMicroseconds2(uint32_t microseconds)
{
	uint32_t startTicks = SysTickValueGet();
	uint32_t numTicks = ((microseconds - 3) * (SysCtrlClockGet() / 1000)) / 1000;
	while (startTicks - SysTickValueGet() < numTicks);
}

void Board::delayMilliseconds(uint32_t milliseconds)
{
  delayMicroseconds(1000 * milliseconds);
}

void Board::getEUI48(uint8_t* address)
{
  uint8_t temp[8];

  /* Get EUI64 */
  getEUI64(temp);

  /* Generate EUI48 from EUI64 */
  memcpy(&address[0], &temp[0], 3);
  memcpy(&address[3], &temp[5], 3);
}

void Board::getEUI64(uint8_t* address)
{
  uint8_t* eui64_flash;

  eui64_flash = (uint8_t*) CC2538_EUI64_ADDRESS_LO_H;
  while (eui64_flash >= (uint8_t*) CC2538_EUI64_ADDRESS_LO_L)
  {
    *address++ = *eui64_flash--;
  }

  eui64_flash = (uint8_t*) CC2538_EUI64_ADDRESS_HI_H;
  while (eui64_flash >= (uint8_t*) CC2538_EUI64_ADDRESS_HI_L)
  {
    *address++ = *eui64_flash--;
  }
}

/*=============================== protected =================================*/

/*================================ private ==================================*/
