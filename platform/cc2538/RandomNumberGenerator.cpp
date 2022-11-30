/**
 * @file       platform_prng.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       November, 2018
 * @brief
 *
 * @copyright  Copyright 2018, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include <stdlib.h>

#include "Board.hpp"
#include "RandomNumberGenerator.hpp"

#include "platform_includes.h"
#include "platform_defines.h"

/*================================ define ===================================*/

#define SOC_ADC_ADCCON1_RCTRL0                  ( 0x00000004 )
#define SOC_ADC_ADCCON1_RCTRL1                  ( 0x00000008 )

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

extern BoardImplementation board;

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

NumberGenerator::NumberGenerator()
{
}

void NumberGenerator::init(void)
{
  return this->init();
}

uint16_t NumberGenerator::get(void)
{
  return this->get();
}

RandomNumberGenerator::RandomNumberGenerator()
{
}

void RandomNumberGenerator::init(void)
{
  uint32_t i;
  uint16_t s = 0;

  /* Make sure the RNG is on */
  HWREG(SOC_ADC_ADCCON1) &= ~(SOC_ADC_ADCCON1_RCTRL1 | SOC_ADC_ADCCON1_RCTRL0);

  /* Enable clock for the RF Core */
  HWREG(SYS_CTRL_RCGCRFC) = 1;

  /* Wait for the clock ungating to take effect */
  while(HWREG(SYS_CTRL_RCGCRFC) != 1);

  /* Infinite RX - FRMCTRL0[3:2] = 10 (this will mess with radio operation) */
  HWREG(RFCORE_XREG_FRMCTRL0) = 0x00000008;

  /* Turn RF on */
  CC2538_RF_CSP_ISRXON();

  /*
   * Wait until "the chip has been in RX long enough for the transients to
   * have died out. A convenient way to do this is to wait for the RSSI-valid
   * signal to go high."
   */
  while(!(HWREG(RFCORE_XREG_RSSISTAT) & RFCORE_XREG_RSSISTAT_RSSI_VALID));

  /*
   * Form the seed by concatenating bits from IF_ADC in the RF receive path.
   * Keep sampling until we have read at least 16 bits AND the seed is valid
   *
   * Invalid seeds are 0x0000 and 0x8003 and should not be used.
   */
  while(s == 0x0000 || s == 0x8003)
  {
    for(i = 0; i < 16; i++)
    {
      s |= (HWREG(RFCORE_XREG_RFRND) & RFCORE_XREG_RFRND_IRND);
      s <<= 1;
    }
  }

  /* High byte first */
  HWREG(SOC_ADC_RNDL) = (s >> 8) & 0x00FF;
  HWREG(SOC_ADC_RNDL) = s & 0xFF;

  /* Turn RF off */
  CC2538_RF_CSP_ISRFOFF();
}

uint16_t RandomNumberGenerator::get(void)
{
  volatile uint32_t random;

  /* Clock the RNG LSFR once */
  HWREG(SOC_ADC_ADCCON1) |= SOC_ADC_ADCCON1_RCTRL0;

  random = HWREG(SOC_ADC_RNDL) | (HWREG(SOC_ADC_RNDH) << 8);

  return random;
}

DeterministicNumberGenerator::DeterministicNumberGenerator()
{
}

void DeterministicNumberGenerator::init(void)
{
  uint32_t scratch = 0;
  uint8_t eui64[8];
  
  board.getEUI64(eui64);
  
  scratch |= eui64[4] << 24;
  scratch |= eui64[5] << 16;
  scratch |= eui64[6] <<  8;
  scratch |= eui64[7] <<  0;
  
  srand(scratch);
}

uint16_t DeterministicNumberGenerator::get(void)
{
  uint32_t random;
  
  random = rand();
  
  return random;
}



/*=============================== protected =================================*/

/*================================ private ==================================*/
