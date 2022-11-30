/**
 * @file       platform_defines.h
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef PLATFORM_DEFINES_H_
#define PLATFORM_DEFINES_H_

/*================================ include ==================================*/

/*================================ define ===================================*/

// Defines for the transmit power
#define CC2538_RF_TX_POWER_DEFAULT              ( 0xFF )

// Defines for the channel
#define CC2538_RF_CHANNEL_MIN                   ( 11 )
#define CC2538_RF_CHANNEL_MAX                   ( 26 )
#define CC2538_RF_CHANNEL_DEFAULT               ( 17 )
#define CC2538_RF_CHANNEL_SPACING               ( 5 )

// Defines for the RSSI
#define CC2538_RF_RSSI_OFFSET                   ( 73 )

// Defines for the CRC and LQI
#define CC2538_RF_CRC_BITMASK                   ( 0x80 )
#define CC2538_RF_LQI_BITMASK                   ( 0x7F )

// Defines for the packet
#define CC2538_RF_MAX_PACKET_LEN                ( 127 )
#define CC2538_RF_MIN_PACKET_LEN                ( 3 )

// Defines for the CCA (Clear Channel Assessment)
#define CC2538_RF_CCA_CLEAR                     ( 0x01 )
#define CC2538_RF_CCA_BUSY                      ( 0x00 )
#define CC2538_RF_CCA_THRESHOLD                 ( 0xF8 )

// Defines for the CSP (Command Strobe Processor)
#define CC2538_RF_CSP_OP_ISRXON                 ( 0xE3 )
#define CC2538_RF_CSP_OP_ISTXON                 ( 0xE9 )
#define CC2538_RF_CSP_OP_ISTXONCCA              ( 0xEA )
#define CC2538_RF_CSP_OP_ISRFOFF                ( 0xEF )
#define CC2538_RF_CSP_OP_ISFLUSHRX              ( 0xED )
#define CC2538_RF_CSP_OP_ISFLUSHTX              ( 0xEE )

// Send an RX ON command strobe to the CSP
#define CC2538_RF_CSP_ISRXON()    \
  do { HWREG(RFCORE_SFR_RFST) = CC2538_RF_CSP_OP_ISRXON; } while(0)

// Send a TX ON command strobe to the CSP
#define CC2538_RF_CSP_ISTXON()    \
  do { HWREG(RFCORE_SFR_RFST) = CC2538_RF_CSP_OP_ISTXON; } while(0)

// Send a RF OFF command strobe to the CSP
#define CC2538_RF_CSP_ISRFOFF()   \
  do { HWREG(RFCORE_SFR_RFST) = CC2538_RF_CSP_OP_ISRFOFF; } while(0)

// Flush the RX FIFO
#define CC2538_RF_CSP_ISFLUSHRX()  do { \
  HWREG(RFCORE_SFR_RFST) = CC2538_RF_CSP_OP_ISFLUSHRX; \
  HWREG(RFCORE_SFR_RFST) = CC2538_RF_CSP_OP_ISFLUSHRX; \
} while(0)

// Flush the TX FIFO
#define CC2538_RF_CSP_ISFLUSHTX()  do { \
  HWREG(RFCORE_SFR_RFST) = CC2538_RF_CSP_OP_ISFLUSHTX; \
  HWREG(RFCORE_SFR_RFST) = CC2538_RF_CSP_OP_ISFLUSHTX; \
} while(0)

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

/*================================ private ==================================*/

#endif  /* PLATFORM_DEFINES_H_ */
