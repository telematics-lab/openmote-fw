/**
 * @file       At86rf215_conf.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2016
 * @brief
 *
 * @copyright  Copyright 2016, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include "At86rf215_conf.h"
#include "At86rf215_regs.h"

/*================================ define ===================================*/

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

/*=============================== protected =================================*/

/*================================ private ==================================*/

const at86rf215_register_t fsk_option1_rf[] = 
{
  {RFn_RXBWC,       0x00},
  {RFn_RXDFE,       0x2A},
  {RFn_AGCC,        0x01},
  {RFn_EDD,         0x7A},
  {RFn_TXCUTC,      0xC0},
  {RFn_TXDFE,       0x98},
};
const at86rf215_register_t fsk_option1_bbc[] = 
{
  {BBCn_PC,         0x15},
  {BBCn_FSKDM,      0x01},
  {BBCn_FSKC0,      0xD6},
  {BBCn_FSKC1,      0x00},
  {BBCn_FSKC2,      0x41},
  {BBCn_FSKC3,      0x85},
  {BBCn_FSKC4,      0x0A},
  {BBCn_FSKPE0,     0x02},
  {BBCn_FSKPE1,     0x03},
  {BBCn_FSKPE2,     0xFC},
  {BBCn_FSKPHRTX,   0x00},
};
const radio_settings_t fsk_option1_cfg = {
                                          fsk_option1_rf,
                                          sizeof(fsk_option1_rf) / sizeof(at86rf215_register_t),
                                          fsk_option1_bbc,
                                          sizeof(fsk_option1_bbc) / sizeof(at86rf215_register_t)
                                         };

/******************************************************************************/

const at86rf215_register_t fsk_option2_rf[] =
{
  {RFn_RXBWC,       0x03},
  {RFn_RXDFE,       0x15},
  {RFn_AGCC,        0x01},
  {RFn_EDD,         0x7A},
  {RFn_TXCUTC,      0x83},
  {RFn_TXDFE,       0x94},
};
const at86rf215_register_t fsk_option2_bbc[] =
{
  {BBCn_PC,         0x1D},
  {BBCn_FSKDM,      0x03},
  {BBCn_FSKC0,      0xD6},
  {BBCn_FSKC1,      0x01},
  {BBCn_FSKC2,      0x40},
  {BBCn_FSKC3,      0x85},
  {BBCn_FSKC4,      0x0A},
  {BBCn_FSKPE0,     0x13},
  {BBCn_FSKPE1,     0x29},
  {BBCn_FSKPE2,     0xC7},
  {BBCn_FSKPHRTX,   0x00},
};
const radio_settings_t fsk_option2_cfg = {
                                          fsk_option2_rf,
                                          sizeof(fsk_option2_rf) / sizeof(at86rf215_register_t),
                                          fsk_option2_bbc,
                                          sizeof(fsk_option2_bbc) / sizeof(at86rf215_register_t)
                                         };

/******************************************************************************/

const at86rf215_register_t fsk_option3_rf[] =
{
  {RFn_RXBWC,       0x03},
  {RFn_RXDFE,       0x24},
  {RFn_AGCC,        0x01},
  {RFn_EDD,         0x7A},
  {RFn_TXCUTC,      0x84},
  {RFn_TXDFE,       0x12},
};
const at86rf215_register_t fsk_option3_bbc[] =
{
  {BBCn_PC,         0x15},
  {BBCn_FSKDM,      0x03},
  {BBCn_FSKC0,      0xC1},
  {BBCn_FSKC1,      0x03},
  {BBCn_FSKC2,      0x00},
  {BBCn_FSKC3,      0x85},
  {BBCn_FSKC4,      0x00},
  {BBCn_FSKPE0,     0x74},
  {BBCn_FSKPE1,     0x7F},
  {BBCn_FSKPE2,     0x80},
  {BBCn_FSKPHRTX,   0x00},
};
const radio_settings_t fsk_option3_cfg = {
                                          fsk_option3_rf,
                                          sizeof(fsk_option3_rf) / sizeof(at86rf215_register_t),
                                          fsk_option3_bbc,
                                          sizeof(fsk_option3_bbc) / sizeof(at86rf215_register_t)
                                         };

/******************************************************************************/

const at86rf215_register_t oqpsk_rate1_rf[] =
{
  {RFn_RXBWC,       0x00},
  {RFn_RXDFE,       0x2A},
  {RFn_AGCC,        0x21},
  {RFn_EDD,         0x2B},
  {RFn_TXCUTC,      0xC7},
  {RFn_TXDFE,       0x7A},
};
const at86rf215_register_t oqpsk_rate1_bbc[] =
{
  {BBCn_PC,         0x17},
  {BBCn_OQPSKPHRTX, 0x00},
  {BBCn_OQPSKC0,    0x10},
};
const radio_settings_t oqpsk_rate1_cfg = {
                                          oqpsk_rate1_rf,
                                          sizeof(oqpsk_rate1_rf) / sizeof(at86rf215_register_t),
                                          oqpsk_rate1_bbc,
                                          sizeof(oqpsk_rate1_bbc) / sizeof(at86rf215_register_t)
                                         };

/******************************************************************************/

const at86rf215_register_t oqpsk_rate2_rf[] =
{
  {RFn_RXBWC,       0x00},
  {RFn_RXDFE,       0x2A},
  {RFn_AGCC,        0x21},
  {RFn_EDD,         0x2B},
  {RFn_TXCUTC,      0xC7},
  {RFn_TXDFE,       0x7A},
};
const at86rf215_register_t oqpsk_rate2_bbc[] =
{
  {BBCn_PC,         0x17},
  {BBCn_OQPSKPHRTX, 0x02},
  {BBCn_OQPSKC0,    0x10},
};
const radio_settings_t oqpsk_rate2_cfg = {
                                          oqpsk_rate2_rf,
                                          sizeof(oqpsk_rate2_rf) / sizeof(at86rf215_register_t),
                                          oqpsk_rate2_bbc,
                                          sizeof(oqpsk_rate2_bbc) / sizeof(at86rf215_register_t)
                                         };

/******************************************************************************/

const at86rf215_register_t oqpsk_rate3_rf[] =
{
  {RFn_RXBWC,       0x00},
  {RFn_RXDFE,       0x2A},
  {RFn_AGCC,        0x21},
  {RFn_EDD,         0x2B},
  {RFn_TXCUTC,      0xC7},
  {RFn_TXDFE,       0x7A},
};
const at86rf215_register_t oqpsk_rate3_bbc[] =
{
  {BBCn_PC,         0x17},
  {BBCn_OQPSKPHRTX, 0x04},
  {BBCn_OQPSKC0,    0x10},
};
const radio_settings_t oqpsk_rate3_cfg = {
                                          oqpsk_rate3_rf,
                                          sizeof(oqpsk_rate3_rf) / sizeof(at86rf215_register_t),
                                          oqpsk_rate3_bbc,
                                          sizeof(oqpsk_rate3_bbc) / sizeof(at86rf215_register_t)
                                         };

/******************************************************************************/

const at86rf215_register_t oqpsk_rate4_rf[] =
{
  {RFn_RXBWC,       0x00},
  {RFn_RXDFE,       0x2A},
  {RFn_AGCC,        0x21},
  {RFn_EDD,         0x2B},
  {RFn_TXCUTC,      0xC7},
  {RFn_TXDFE,       0x7A},
};
const at86rf215_register_t oqpsk_rate4_bbc[] =
{
  {BBCn_PC,         0x17},
  {BBCn_OQPSKPHRTX, 0x06},
  {BBCn_OQPSKC0,    0x10},
};
const radio_settings_t oqpsk_rate4_cfg = {
                                          oqpsk_rate4_rf,
                                          sizeof(oqpsk_rate4_rf) / sizeof(at86rf215_register_t),
                                          oqpsk_rate4_bbc,
                                          sizeof(oqpsk_rate4_bbc) / sizeof(at86rf215_register_t)
                                         };

/******************************************************************************/

const at86rf215_register_t oqpsk_250kbps_rf[] =
{
  {RFn_RXBWC,       0x0B},
  {RFn_RXDFE,       0x41},
  {RFn_AGCC,        0x01},
  {RFn_EDD,         0x13},
  {RFn_TXCUTC,      0x0B},
  {RFn_TXDFE,       0x81},
};
const at86rf215_register_t oqpsk_250kbps_bbc[] =
{
  {BBCn_PC,         0x1F},
  {BBCn_OQPSKPHRTX, 0x09},
  {BBCn_OQPSKC0,    0x03},
  {BBCn_OQPSKC1,    0x97},
  {BBCn_OQPSKC2,    0x05},
  {BBCn_OQPSKC3,    0x00},
};
const radio_settings_t oqpsk_250kbps_cfg = {
                                          oqpsk_250kbps_rf,
                                          sizeof(oqpsk_250kbps_rf) / sizeof(at86rf215_register_t),
                                          oqpsk_250kbps_bbc,
                                          sizeof(oqpsk_250kbps_bbc) / sizeof(at86rf215_register_t)
                                         };

/******************************************************************************/

const at86rf215_register_t ofdm1_mcs0_rf[] =
{
  {RFn_RXBWC,       0x19},
  {RFn_RXDFE,       0x83},
  {RFn_AGCC,        0x11},
  {RFn_EDD,         0x7A},
  {RFn_TXCUTC,      0x0A},
  {RFn_TXDFE,       0x83},
};
const at86rf215_register_t ofdm1_mcs0_bbc[] =
{
  {BBCn_PC,         0x16},
  {BBCn_OFDMC,      0x00},
  {BBCn_OFDMPHRTX,  0x00},
  {BBCn_OFDMSW,     0xB0},
};
const radio_settings_t ofdm1_mcs0_cfg = {
                                          ofdm1_mcs0_rf,
                                          sizeof(ofdm1_mcs0_rf) / sizeof(at86rf215_register_t),
                                          ofdm1_mcs0_bbc,
                                          sizeof(ofdm1_mcs0_bbc) / sizeof(at86rf215_register_t)
                                         };

/******************************************************************************/

const at86rf215_register_t ofdm1_mcs1_rf[] =
{
  {RFn_RXBWC,       0x19},
  {RFn_RXDFE,       0x83},
  {RFn_AGCC,        0x11},
  {RFn_EDD,         0x7A},
  {RFn_TXCUTC,      0x0A},
  {RFn_TXDFE,       0x83},
};
const at86rf215_register_t ofdm1_mcs1_bbc[] =
{
  {BBCn_PC,         0x16},
  {BBCn_OFDMC,      0x00},
  {BBCn_OFDMPHRTX,  0x01},
  {BBCn_OFDMSW,     0xB0},
};
const radio_settings_t ofdm1_mcs1_cfg = {
                                          ofdm1_mcs1_rf,
                                          sizeof(ofdm1_mcs1_rf) / sizeof(at86rf215_register_t),
                                          ofdm1_mcs1_bbc,
                                          sizeof(ofdm1_mcs1_bbc) / sizeof(at86rf215_register_t)
                                         };

/******************************************************************************/

const at86rf215_register_t ofdm1_mcs2_rf[] =
{
  {RFn_RXBWC,       0x19},
  {RFn_RXDFE,       0x83},
  {RFn_AGCC,        0x11},
  {RFn_EDD,         0x7A},
  {RFn_TXCUTC,      0x0A},
  {RFn_TXDFE,       0x83},
};
const at86rf215_register_t ofdm1_mcs2_bbc[] =
{
  {BBCn_PC,         0x16},
  {BBCn_OFDMC,      0x00},
  {BBCn_OFDMPHRTX,  0x02},
  {BBCn_OFDMSW,     0xB0},
};
const radio_settings_t ofdm1_mcs2_cfg = {
                                          ofdm1_mcs2_rf,
                                          sizeof(ofdm1_mcs2_rf) / sizeof(at86rf215_register_t),
                                          ofdm1_mcs2_bbc,
                                          sizeof(ofdm1_mcs2_bbc) / sizeof(at86rf215_register_t)
                                         };

/******************************************************************************/

const at86rf215_register_t ofdm1_mcs3_rf[] =
{
  {RFn_RXBWC,       0x19},
  {RFn_RXDFE,       0x83},
  {RFn_AGCC,        0x11},
  {RFn_EDD,         0x7A},
  {RFn_TXCUTC,      0x0A},
  {RFn_TXDFE,       0x83},
};
const at86rf215_register_t ofdm1_mcs3_bbc[] =
{
  {BBCn_PC,         0x16},
  {BBCn_OFDMC,      0x00},
  {BBCn_OFDMPHRTX,  0x03},
  {BBCn_OFDMSW,     0xB0},
};
const radio_settings_t ofdm1_mcs3_cfg = {
                                          ofdm1_mcs3_rf,
                                          sizeof(ofdm1_mcs3_rf) / sizeof(at86rf215_register_t),
                                          ofdm1_mcs3_bbc,
                                          sizeof(ofdm1_mcs3_bbc) / sizeof(at86rf215_register_t)
                                         };

/******************************************************************************/

const at86rf215_register_t ofdm1_mcs4_rf[] =
{
  {RFn_RXBWC,       0x19},
  {RFn_RXDFE,       0x83},
  {RFn_AGCC,        0x11},
  {RFn_EDD,         0x7A},
  {RFn_TXCUTC,      0x0A},
  {RFn_TXDFE,       0x83},
};
const at86rf215_register_t ofdm1_mcs4_bbc[] =
{
  {BBCn_PC,         0x16},
  {BBCn_OFDMC,      0x00},
  {BBCn_OFDMPHRTX,  0x04},
  {BBCn_OFDMSW,     0xB0},
};
const radio_settings_t ofdm1_mcs4_cfg = {
                                          ofdm1_mcs4_rf,
                                          sizeof(ofdm1_mcs4_rf) / sizeof(at86rf215_register_t),
                                          ofdm1_mcs4_bbc,
                                          sizeof(ofdm1_mcs4_bbc) / sizeof(at86rf215_register_t)
                                         };

/******************************************************************************/

const at86rf215_register_t ofdm1_mcs5_rf[] =
{
  {RFn_RXBWC,       0x19},
  {RFn_RXDFE,       0x83},
  {RFn_AGCC,        0x11},
  {RFn_EDD,         0x7A},
  {RFn_TXCUTC,      0x0A},
  {RFn_TXDFE,       0x83},
};
const at86rf215_register_t ofdm1_mcs5_bbc[] =
{
  {BBCn_PC,         0x16},
  {BBCn_OFDMC,      0x00},
  {BBCn_OFDMPHRTX,  0x05},
  {BBCn_OFDMSW,     0xB0},
};
const radio_settings_t ofdm1_mcs5_cfg = {
                                          ofdm1_mcs5_rf,
                                          sizeof(ofdm1_mcs5_rf) / sizeof(at86rf215_register_t),
                                          ofdm1_mcs5_bbc,
                                          sizeof(ofdm1_mcs5_bbc) / sizeof(at86rf215_register_t)
                                         };

/******************************************************************************/

const at86rf215_register_t ofdm1_mcs6_rf[] =
{
  {RFn_RXBWC,       0x19},
  {RFn_RXDFE,       0x83},
  {RFn_AGCC,        0x11},
  {RFn_EDD,         0x7A},
  {RFn_TXCUTC,      0x0A},
  {RFn_TXDFE,       0x83},
};
const at86rf215_register_t ofdm1_mcs6_bbc[] =
{
  {BBCn_PC,         0x16},
  {BBCn_OFDMC,      0x00},
  {BBCn_OFDMPHRTX,  0x06},
  {BBCn_OFDMSW,     0xB0},
};
const radio_settings_t ofdm1_mcs6_cfg = {
                                          ofdm1_mcs6_rf,
                                          sizeof(ofdm1_mcs6_rf) / sizeof(at86rf215_register_t),
                                          ofdm1_mcs6_bbc,
                                          sizeof(ofdm1_mcs6_bbc) / sizeof(at86rf215_register_t)
                                         };

/******************************************************************************/

const at86rf215_register_t ofdm2_mcs0_rf[] =
{
  {RFn_RXBWC,       0x17},
  {RFn_RXDFE,       0x43},
  {RFn_AGCC,        0x11},
  {RFn_EDD,         0x7A},
  {RFn_TXCUTC,      0x08},
  {RFn_TXDFE,       0x63},
};
const at86rf215_register_t ofdm2_mcs0_bbc[] =
{
  {BBCn_PC,         0x16},
  {BBCn_OFDMC,      0x01},
  {BBCn_OFDMPHRTX,  0x00},
  {BBCn_OFDMSW,     0xB0},
};
const radio_settings_t ofdm2_mcs0_cfg = {
                                          ofdm2_mcs0_rf,
                                          sizeof(ofdm2_mcs0_rf) / sizeof(at86rf215_register_t),
                                          ofdm2_mcs0_bbc,
                                          sizeof(ofdm2_mcs0_bbc) / sizeof(at86rf215_register_t)
                                         };

/******************************************************************************/

const at86rf215_register_t ofdm2_mcs1_rf[] =
{
  {RFn_RXBWC,       0x17},
  {RFn_RXDFE,       0x43},
  {RFn_AGCC,        0x11},
  {RFn_EDD,         0x7A},
  {RFn_TXCUTC,      0x08},
  {RFn_TXDFE,       0x63},
};
const at86rf215_register_t ofdm2_mcs1_bbc[] =
{
  {BBCn_PC,         0x16},
  {BBCn_OFDMC,      0x01},
  {BBCn_OFDMPHRTX,  0x01},
  {BBCn_OFDMSW,     0xB0},
};
const radio_settings_t ofdm2_mcs1_cfg = {
                                          ofdm2_mcs1_rf,
                                          sizeof(ofdm2_mcs1_rf) / sizeof(at86rf215_register_t),
                                          ofdm2_mcs1_bbc,
                                          sizeof(ofdm2_mcs1_bbc) / sizeof(at86rf215_register_t)
                                         };

/******************************************************************************/

const at86rf215_register_t ofdm2_mcs2_rf[] =
{
  {RFn_RXBWC,       0x17},
  {RFn_RXDFE,       0x43},
  {RFn_AGCC,        0x11},
  {RFn_EDD,         0x7A},
  {RFn_TXCUTC,      0x08},
  {RFn_TXDFE,       0x63},
};
const at86rf215_register_t ofdm2_mcs2_bbc[] =
{
  {BBCn_PC,         0x16},
  {BBCn_OFDMC,      0x01},
  {BBCn_OFDMPHRTX,  0x02},
  {BBCn_OFDMSW,     0xB0},

};
const radio_settings_t ofdm2_mcs2_cfg = {
                                          ofdm2_mcs2_rf,
                                          sizeof(ofdm2_mcs2_rf) / sizeof(at86rf215_register_t),
                                          ofdm2_mcs2_bbc,
                                          sizeof(ofdm2_mcs2_bbc) / sizeof(at86rf215_register_t)
                                         };

/******************************************************************************/

const at86rf215_register_t ofdm2_mcs3_rf[] =
{
  {RFn_RXBWC,       0x17},
  {RFn_RXDFE,       0x43},
  {RFn_AGCC,        0x11},
  {RFn_EDD,         0x7A},
  {RFn_TXCUTC,      0x08},
  {RFn_TXDFE,       0x63},
};
const at86rf215_register_t ofdm2_mcs3_bbc[] =
{
  {BBCn_PC,         0x16},
  {BBCn_OFDMC,      0x01},
  {BBCn_OFDMPHRTX,  0x03},
  {BBCn_OFDMSW,     0xB0},
};
const radio_settings_t ofdm2_mcs3_cfg = {
                                          ofdm2_mcs3_rf,
                                          sizeof(ofdm2_mcs3_rf) / sizeof(at86rf215_register_t),
                                          ofdm2_mcs3_bbc,
                                          sizeof(ofdm2_mcs3_bbc) / sizeof(at86rf215_register_t)
                                         };

/******************************************************************************/

const at86rf215_register_t ofdm2_mcs4_rf[] =
{
  {RFn_RXBWC,       0x17},
  {RFn_RXDFE,       0x43},
  {RFn_AGCC,        0x11},
  {RFn_EDD,         0x7A},
  {RFn_TXCUTC,      0x08},
  {RFn_TXDFE,       0x63},
};
const at86rf215_register_t ofdm2_mcs4_bbc[] =
{ 
  {BBCn_PC,         0x16},
  {BBCn_OFDMC,      0x01},
  {BBCn_OFDMPHRTX,  0x04},
  {BBCn_OFDMSW,     0xB0},
};
const radio_settings_t ofdm2_mcs4_cfg = {
                                          ofdm2_mcs4_rf,
                                          sizeof(ofdm2_mcs4_rf) / sizeof(at86rf215_register_t),
                                          ofdm2_mcs4_bbc,
                                          sizeof(ofdm2_mcs4_bbc) / sizeof(at86rf215_register_t)
                                         };

/******************************************************************************/

const at86rf215_register_t ofdm2_mcs5_rf[] =
{
  {RFn_RXBWC,       0x17},
  {RFn_RXDFE,       0x43},
  {RFn_AGCC,        0x11},
  {RFn_EDD,         0x7A},
  {RFn_TXCUTC,      0x08},
  {RFn_TXDFE,       0x63},
};
const at86rf215_register_t ofdm2_mcs5_bbc[] =
{
  {BBCn_PC,         0x16},
  {BBCn_OFDMC,      0x01},
  {BBCn_OFDMPHRTX,  0x05},
  {BBCn_OFDMSW,     0xB0},
};
const radio_settings_t ofdm2_mcs5_cfg = {
                                          ofdm2_mcs5_rf,
                                          sizeof(ofdm2_mcs5_rf) / sizeof(at86rf215_register_t),
                                          ofdm2_mcs5_bbc,
                                          sizeof(ofdm2_mcs5_bbc) / sizeof(at86rf215_register_t)
                                         };

/******************************************************************************/

const at86rf215_register_t ofdm2_mcs6_rf[] =
{
  {RFn_RXBWC,       0x17},
  {RFn_RXDFE,       0x43},
  {RFn_AGCC,        0x11},
  {RFn_EDD,         0x7A},
  {RFn_TXCUTC,      0x08},
  {RFn_TXDFE,       0x63},
};
const at86rf215_register_t ofdm2_mcs6_bbc[] =
{
  {BBCn_PC,         0x16},
  {BBCn_OFDMC,      0x01},
  {BBCn_OFDMPHRTX,  0x06},
  {BBCn_OFDMSW,     0xB0},
};
const radio_settings_t ofdm2_mcs6_cfg = {
                                          ofdm2_mcs6_rf,
                                          sizeof(ofdm2_mcs6_rf) / sizeof(at86rf215_register_t),
                                          ofdm2_mcs6_bbc,
                                          sizeof(ofdm2_mcs6_bbc) / sizeof(at86rf215_register_t)
                                         };

/******************************************************************************/

const at86rf215_register_t ofdm3_mcs1_rf[] =
{
  {RFn_RXBWC,       0x04},
  {RFn_RXDFE,       0x46},
  {RFn_AGCC,        0x11},
  {RFn_EDD,         0x7A},
  {RFn_TXCUTC,      0x05},
  {RFn_TXDFE,       0x66},
};
const at86rf215_register_t ofdm3_mcs1_bbc[] =
{
  {BBCn_PC,         0x16},
  {BBCn_OFDMC,      0x02},
  {BBCn_OFDMPHRTX,  0x01},
  {BBCn_OFDMSW,     0x70},
};
const radio_settings_t ofdm3_mcs1_cfg = {
                                          ofdm3_mcs1_rf,
                                          sizeof(ofdm3_mcs1_rf) / sizeof(at86rf215_register_t),
                                          ofdm3_mcs1_bbc,
                                          sizeof(ofdm3_mcs1_bbc) / sizeof(at86rf215_register_t)
                                         };

/******************************************************************************/

const at86rf215_register_t ofdm3_mcs2_rf[] =
{
  {RFn_RXBWC,       0x04},
  {RFn_RXDFE,       0x46},
  {RFn_AGCC,        0x11},
  {RFn_EDD,         0x7A},
  {RFn_TXCUTC,      0x05},
  {RFn_TXDFE,       0x66},
};
const at86rf215_register_t ofdm3_mcs2_bbc[] =
{
  {BBCn_PC,         0x16},
  {BBCn_OFDMC,      0x02},
  {BBCn_OFDMPHRTX,  0x02},
  {BBCn_OFDMSW,     0x70},
};
const radio_settings_t ofdm3_mcs2_cfg = {
                                          ofdm3_mcs2_rf,
                                          sizeof(ofdm2_mcs3_rf) / sizeof(at86rf215_register_t),
                                          ofdm3_mcs2_bbc,
                                          sizeof(ofdm3_mcs2_bbc) / sizeof(at86rf215_register_t)
                                         };

/******************************************************************************/

const at86rf215_register_t ofdm3_mcs3_rf[] =
{
  {RFn_RXBWC,       0x04},
  {RFn_RXDFE,       0x46},
  {RFn_AGCC,        0x11},
  {RFn_EDD,         0x7A},
  {RFn_TXCUTC,      0x05},
  {RFn_TXDFE,       0x66},
};
const at86rf215_register_t ofdm3_mcs3_bbc[] =
{
  {BBCn_PC,         0x16},
  {BBCn_OFDMC,      0x02},
  {BBCn_OFDMPHRTX,  0x03},
  {BBCn_OFDMSW,     0x70},
};
const radio_settings_t ofdm3_mcs3_cfg = {
                                          ofdm3_mcs3_rf,
                                          sizeof(ofdm3_mcs3_rf) / sizeof(at86rf215_register_t),
                                          ofdm3_mcs3_bbc,
                                          sizeof(ofdm3_mcs3_bbc) / sizeof(at86rf215_register_t)
                                         };

/******************************************************************************/

const at86rf215_register_t ofdm3_mcs4_rf[] =
{
  {RFn_RXBWC,       0x04},
  {RFn_RXDFE,       0x46},
  {RFn_AGCC,        0x11},
  {RFn_EDD,         0x7A},
  {RFn_TXCUTC,      0x05},
  {RFn_TXDFE,       0x66},
};
const at86rf215_register_t ofdm3_mcs4_bbc[] =
{
  {BBCn_PC,         0x16},
  {BBCn_OFDMC,      0x02},
  {BBCn_OFDMPHRTX,  0x04},
  {BBCn_OFDMSW,     0x70},
};
const radio_settings_t ofdm3_mcs4_cfg = {
                                          ofdm3_mcs4_rf,
                                          sizeof(ofdm3_mcs4_rf) / sizeof(at86rf215_register_t),
                                          ofdm3_mcs4_bbc,
                                          sizeof(ofdm3_mcs4_bbc) / sizeof(at86rf215_register_t)
                                         };

/******************************************************************************/

const at86rf215_register_t ofdm3_mcs5_rf[] =
{
  {RFn_RXBWC,       0x04},
  {RFn_RXDFE,       0x46},
  {RFn_AGCC,        0x11},
  {RFn_EDD,         0x7A},
  {RFn_TXCUTC,      0x05},
  {RFn_TXDFE,       0x66},
};
const at86rf215_register_t ofdm3_mcs5_bbc[] =
{
  {BBCn_PC,         0x16},
  {BBCn_OFDMC,      0x02},
  {BBCn_OFDMPHRTX,  0x05},
  {BBCn_OFDMSW,     0x70},
};
const radio_settings_t ofdm3_mcs5_cfg = {
                                          ofdm3_mcs5_rf,
                                          sizeof(ofdm3_mcs5_rf) / sizeof(at86rf215_register_t),
                                          ofdm3_mcs5_bbc,
                                          sizeof(ofdm3_mcs5_bbc) / sizeof(at86rf215_register_t)
                                         };

/******************************************************************************/

const at86rf215_register_t ofdm3_mcs6_rf[] =
{
  {RFn_RXBWC,       0x04},
  {RFn_RXDFE,       0x46},
  {RFn_AGCC,        0x11},
  {RFn_EDD,         0x7A},
  {RFn_TXCUTC,      0x05},
  {RFn_TXDFE,       0x66},
};
const at86rf215_register_t ofdm3_mcs6_bbc[] =
{
  {BBCn_PC,         0x16},
  {BBCn_OFDMC,      0x02},
  {BBCn_OFDMPHRTX,  0x06},
  {BBCn_OFDMSW,     0x70},
};
const radio_settings_t ofdm3_mcs6_cfg = {
                                          ofdm3_mcs6_rf,
                                          sizeof(ofdm3_mcs6_rf) / sizeof(at86rf215_register_t),
                                          ofdm3_mcs6_bbc,
                                          sizeof(ofdm3_mcs6_bbc) / sizeof(at86rf215_register_t)
                                         };

/******************************************************************************/

const at86rf215_register_t ofdm4_mcs2_rf[] =
{
  {RFn_RXBWC,       0x12},
  {RFn_RXDFE,       0x26},
  {RFn_AGCC,        0x11},
  {RFn_EDD,         0x7A},
  {RFn_TXCUTC,      0x03},
  {RFn_TXDFE,       0x46},
};
const at86rf215_register_t ofdm4_mcs2_bbc[] =
{
  {BBCn_PC,         0x16},
  {BBCn_OFDMC,      0x03},
  {BBCn_OFDMPHRTX,  0x02},
  {BBCn_OFDMSW,     0x70},
};
const radio_settings_t ofdm4_mcs2_cfg = {
                                          ofdm4_mcs2_rf,
                                          sizeof(ofdm4_mcs2_rf) / sizeof(at86rf215_register_t),
                                          ofdm4_mcs2_bbc,
                                          sizeof(ofdm4_mcs2_bbc) / sizeof(at86rf215_register_t)
                                         };

/******************************************************************************/

const at86rf215_register_t ofdm4_mcs3_rf[] =
{
  {RFn_RXBWC,       0x12},
  {RFn_RXDFE,       0x26},
  {RFn_AGCC,        0x11},
  {RFn_EDD,         0x7A},
  {RFn_TXCUTC,      0x03},
  {RFn_TXDFE,       0x46},
};
const at86rf215_register_t ofdm4_mcs3_bbc[] =
{
  {BBCn_PC,         0x16},
  {BBCn_OFDMC,      0x03},
  {BBCn_OFDMPHRTX,  0x03},
  {BBCn_OFDMSW,     0x70},
};
const radio_settings_t ofdm4_mcs3_cfg = {
                                          ofdm4_mcs3_rf,
                                          sizeof(ofdm4_mcs3_rf) / sizeof(at86rf215_register_t),
                                          ofdm4_mcs3_bbc,
                                          sizeof(ofdm4_mcs3_bbc) / sizeof(at86rf215_register_t)
                                         };

/******************************************************************************/

const at86rf215_register_t ofdm4_mcs4_rf[] =
{
  {RFn_RXBWC,       0x12},
  {RFn_RXDFE,       0x26},
  {RFn_AGCC,        0x11},
  {RFn_EDD,         0x7A},
  {RFn_TXCUTC,      0x03},
  {RFn_TXDFE,       0x46},
};
const at86rf215_register_t ofdm4_mcs4_bbc[] =
{
  {BBCn_PC,         0x16},
  {BBCn_OFDMC,      0x03},
  {BBCn_OFDMPHRTX,  0x04},
  {BBCn_OFDMSW,     0x70},
};
const radio_settings_t ofdm4_mcs4_cfg = {
                                          ofdm4_mcs4_rf,
                                          sizeof(ofdm4_mcs4_rf) / sizeof(at86rf215_register_t),
                                          ofdm4_mcs4_bbc,
                                          sizeof(ofdm4_mcs4_bbc) / sizeof(at86rf215_register_t)
                                         };

/******************************************************************************/

const at86rf215_register_t ofdm4_mcs5_rf[] =
{
  {RFn_RXBWC,       0x12},
  {RFn_RXDFE,       0x26},
  {RFn_AGCC,        0x11},
  {RFn_EDD,         0x7A},
  {RFn_TXCUTC,      0x03},
  {RFn_TXDFE,       0x46},
};
const at86rf215_register_t ofdm4_mcs5_bbc[] =
{
  {BBCn_PC,         0x16},
  {BBCn_OFDMC,      0x03},
  {BBCn_OFDMPHRTX,  0x05},
  {BBCn_OFDMSW,     0x70},
};
const radio_settings_t ofdm4_mcs5_cfg = {
                                          ofdm4_mcs5_rf,
                                          sizeof(ofdm4_mcs5_rf) / sizeof(at86rf215_register_t),
                                          ofdm4_mcs5_bbc,
                                          sizeof(ofdm4_mcs5_bbc) / sizeof(at86rf215_register_t)
                                         };

/******************************************************************************/

const at86rf215_register_t ofdm4_mcs6_rf[] =
{
  {RFn_RXBWC,       0x12},
  {RFn_RXDFE,       0x26},
  {RFn_AGCC,        0x11},
  {RFn_EDD,         0x7A},
  {RFn_TXCUTC,      0x03},
  {RFn_TXDFE,       0x46},
};
const at86rf215_register_t ofdm4_mcs6_bbc[] =
{
  {BBCn_PC,         0x16},
  {BBCn_OFDMC,      0x03},
  {BBCn_OFDMPHRTX,  0x06},
  {BBCn_OFDMSW,     0x70},
};
const radio_settings_t ofdm4_mcs6_cfg = {
                                          ofdm4_mcs6_rf,
                                          sizeof(ofdm4_mcs6_rf) / sizeof(at86rf215_register_t),
                                          ofdm4_mcs6_bbc,
                                          sizeof(ofdm4_mcs6_bbc) / sizeof(at86rf215_register_t)
                                         };

/******************************************************************************/

const radio_settings_t radio_settings[CONFIG_ELEMENTS] =
{
  fsk_option1_cfg,
  fsk_option2_cfg,
  fsk_option3_cfg,
  oqpsk_rate1_cfg,
  oqpsk_rate2_cfg,
  oqpsk_rate3_cfg,
  oqpsk_rate4_cfg,
  oqpsk_250kbps_cfg,
  ofdm1_mcs0_cfg,
  ofdm1_mcs1_cfg,
  ofdm1_mcs2_cfg,
  ofdm1_mcs3_cfg,
  ofdm1_mcs4_cfg,
  ofdm1_mcs5_cfg,
  ofdm1_mcs6_cfg,
  ofdm2_mcs0_cfg,
  ofdm2_mcs1_cfg,
  ofdm2_mcs2_cfg,
  ofdm2_mcs3_cfg,
  ofdm2_mcs4_cfg,
  ofdm2_mcs5_cfg,
  ofdm2_mcs6_cfg,
  ofdm3_mcs1_cfg,
  ofdm3_mcs2_cfg,
  ofdm3_mcs3_cfg,
  ofdm3_mcs4_cfg,
  ofdm3_mcs5_cfg,
  ofdm3_mcs6_cfg,
  ofdm4_mcs2_cfg,
  ofdm4_mcs3_cfg,
  ofdm4_mcs4_cfg,
  ofdm4_mcs5_cfg,
  ofdm4_mcs6_cfg
};

const frequency_settings_t frequency_settings_09[FREQUENCY_09_ELEMENTS] =
{
  {863125, 200,  0, 33},
  {868225, 400,  0, 16},
  {868225, 400,  0, 16},
  {868225, 600,  0,  0},
  {868625, 1200, 0,  4},
  {868425, 800,  0,  7},
  {868225, 400,  0, 16},
  {868125, 200,  0, 33},
};

const frequency_settings_t frequency_settings_24[FREQUENCY_24_ELEMENTS] =
{
  {2400200, 200,  0, 415},
  {2400400, 400,  0, 206},
  {2400400, 400,  0, 206},
  {2405000, 5000, 0,  15},
  {2401200, 1200, 0,  63},
  {2400800, 800,  0,  96},
  {2400400, 400,  0, 206},
  {2400200, 200,  0, 415},
};
