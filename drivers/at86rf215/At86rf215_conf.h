/**
 * @file       At86rf215_conf.hpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2016
 * @brief
 *
 * @copyright  Copyright 2016, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef AT86RF215_CONF_HPP_
#define AT86RF215_CONF_HPP_

#include <stdint.h>

typedef struct
{
  uint16_t address;
  uint8_t value;
} at86rf215_register_t;

typedef struct
{
  const at86rf215_register_t* rf_registers;
  uint16_t rf_elements;
  const at86rf215_register_t* bbc_registers;
  uint16_t bbc_elements;
} radio_settings_t;

typedef struct
{
  uint32_t frequency0;
  uint16_t channel_spacing;
  uint16_t channel_min;
  uint16_t channel_max;
} frequency_settings_t;

/**
 * IEEE 802.15.4g MR-OFDM Reference
 * http://rfmw.em.keysight.com/wireless/helpfiles/n7610b/Content/Main/802.15.4g%20MR_OFDM.htm
 */
enum {
  CONFIG_FSK_OPTION1 = 0, /* 2-FSK,  50 kbps */
  CONFIG_FSK_OPTION2,     /* 2-FSK, 100 kbps */
  CONFIG_FSK_OPTION3,     /* 4-FSK, 200 kbps */
  CONFIG_OQPSK_RATE1,     /* OQPSK-DSSS,  100 kchips/s,   6.25 kbps */
  CONFIG_OQPSK_RATE2,     /* OQPSK-DSSS,  100 kchips/s,  12.50 kbps */
  CONFIG_OQPSK_RATE3,     /* OQPSK-DSSS,  100 kchips/s,  25.00 kbps */
  CONFIG_OQPSK_RATE4,     /* OQPSK-DSSS,  100 kchips/s,  50.00 kbps */
  CONFIG_OQPSK_RATE5,     /* OQPSK-DSSS, 2000 kchips/s, 250 kbps, legacy */
  CONFIG_OFDM1_MCS0,      /* BPSK,   rate 1/2, 4x repetition,  100 kbps */
  CONFIG_OFDM1_MCS1,      /* BPSK,   rate 1/2, 2x repetition,  200 kbps */
  CONFIG_OFDM1_MCS2,      /* QPSK,   rate 1/2, 2x repetition,  400 kbps */
  CONFIG_OFDM1_MCS3,      /* QPSK,   rate 1/2, 0x repetition,  800 kbps */
  CONFIG_OFDM1_MCS4,      /* QPSK,   rate 3/4, 0x repetition, 1200 kbps */
  CONFIG_OFDM1_MCS5,      /* 16-QAM, rate 1/2, 0x repetition, 1600 kbps */
  CONFIG_OFDM1_MCS6,      /* 16-QAM, rate 3/4, 0x repetition, 2400 kbps */
  CONFIG_OFDM2_MCS0,      /* BPSK,   rate 1/2, 4x repetition,   50 kbps */
  CONFIG_OFDM2_MCS1,      /* BPSK,   rate 1/2, 2x repetition,  100 kbps */
  CONFIG_OFDM2_MCS2,      /* QPSK,   rate 1/2, 2x repetition,  200 kbps */
  CONFIG_OFDM2_MCS3,      /* QPSK,   rate 1/2, 0x repetition,  400 kbps */
  CONFIG_OFDM2_MCS4,      /* QPSK,   rate 3/4, 0x repetition,  600 kbps */
  CONFIG_OFDM2_MCS5,      /* 16-QAM, rate 1/2, 0x repetition,  800 kbps */
  CONFIG_OFDM2_MCS6,      /* 16-QAM, rate 3/4, 0x repetition, 1200 kbps */
  CONFIG_OFDM3_MCS1,      /* BPSK,   rate 1/2, 2x repetition,   50 kbps */
  CONFIG_OFDM3_MCS2,      /* QPSK,   rate 1/2, 2x repetition,  100 kbps */
  CONFIG_OFDM3_MCS3,      /* QPSK,   rate 1/2, 0x repetition,  200 kbps */
  CONFIG_OFDM3_MCS4,      /* QPSK,   rate 3/4, 0x repetition,  300 kbps */
  CONFIG_OFDM3_MCS5,      /* 16-QAM, rate 1/2, 0x repetition,  400 kbps */
  CONFIG_OFDM3_MCS6,      /* 16-QAM, rate 3/4, 0x repetition,  600 kbps */
  CONFIG_OFDM4_MCS2,      /* QPSK,   rate 1/2, 2x repetition,   50 kbps */
  CONFIG_OFDM4_MCS3,      /* QPSK,   rate 1/2, 0x repetition,  100 kbps */
  CONFIG_OFDM4_MCS4,      /* QPSK,   rate 3/4, 0x repetition,  150 kbps */
  CONFIG_OFDM4_MCS5,      /* 16-QAM, rate 1/2, 0x repetition,  200 kbps */
  CONFIG_OFDM4_MCS6,      /* 16-QAM, rate 3/4, 0x repetition,  300 kbps */
  CONFIG_ELEMENTS
};

enum {
  FREQUENCY_09_FSK1 = 0, /* FSK Mode 1,   200 kHz */
  FREQUENCY_09_FSK2,     /* FSK Mode 2,   400 kHz */
  FREQUENCY_09_FSK3,     /* FSK Mode 3,   400 kHz */
  FREQUENCY_09_OQPSK,    /* OQPSK,        600 kHz */
  FREQUENCY_09_OFDM1,    /* OFDM Mode 1, 1200 kHz */
  FREQUENCY_09_OFDM2,    /* OFDM Mode 2,  800 kHz */
  FREQUENCY_09_OFDM3,    /* OFDM Mode 3,  400 kHz */
  FREQUENCY_09_OFDM4,    /* OFDM Mode 4,  200 kHz */
  FREQUENCY_09_ELEMENTS
};

enum {
  FREQUENCY_24_FSK1 = 0, /* FSK Mode 1,   200 kHz */
  FREQUENCY_24_FSK2,     /* FSK Mode 2,   400 kHz */
  FREQUENCY_24_FSK3,     /* FSK Mode 3,   400 kHz */
  FREQUENCY_24_OQPSK,    /* OQPSK,       5000 kHz */
  FREQUENCY_24_OFDM1,    /* OFDM Mode 1, 1200 kHz */
  FREQUENCY_24_OFDM2,    /* OFDM Mode 2,  800 kHz */
  FREQUENCY_24_OFDM3,    /* OFDM Mode 3,  400 kHz */
  FREQUENCY_24_OFDM4,    /* OFDM Mode 4,  200 kHz */
  FREQUENCY_24_ELEMENTS
};

extern const radio_settings_t radio_settings[CONFIG_ELEMENTS];
extern const frequency_settings_t frequency_settings_09[FREQUENCY_09_ELEMENTS];
extern const frequency_settings_t frequency_settings_24[FREQUENCY_24_ELEMENTS];

#endif /* AT86RF215_CONF_HPP_ */
