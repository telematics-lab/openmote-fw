/**
 * @file       At86rf215.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2016
 * @brief
 *
 * @copyright  Copyright 2016, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include <stdlib.h>

#include "At86rf215.hpp"
#include "At86rf215_regs.h"

#include "BoardImplementation.hpp"

#include "platform_types.h"

/*================================ define ===================================*/

#define AT86RF215_RSSI_MAX_VALUE        ( 127 )

#define AT86RF215_ED_MIN_VALUE          ( -127 )
#define AT86RF215_ED_MAX_VALUE          ( 4 )

#define AT86RF215_DELAY_MS              ( 1 )
#define AT86RF215_DELAY_US              ( 100 )

#define AT86RF215_RFn_IRQM              ( 0x00 )
#define AT86RF215_BBCn_IRQM             ( 0x13 )

#define AT86RF215_RF_CFG_DEFAULT        ( 0x08 )
#define AT86RF215_RF_CLK0_DEFAULT       ( 0x00 ) 


#define AT86RF215_RFn_PAC_PACUR_MASK    ( 0x06 << 4)
#define AT86RF215_RFn_PAC_PACUR_3dB     ( 0x00 )
#define AT86RF215_RFn_PAC_PACUR_2dB     ( 0x01 )
#define AT86RF215_RFn_PAC_PACUR_1dB     ( 0x02 )
#define AT86RF215_RFn_PAC_PACUR_0dB     ( 0x03 )
#define AT86RF215_RFn_PAC_PACUR_SHIFT   ( 5 )

#define AT86RF215_RFn_PAC_TXPWR_MASK    ( 0x1F )

#define AT86RF215_BBCn_PC_CTX_MASK      ( 0x80 )
#define AT86RF215_BBCn_PC_FCST_MASK     ( 0x08 )
#define AT86RF215_BBCn_PC_FCSOK_MASK    ( 0x20 )

#define AT86RF215_BBCn_IRQS_RXFS_MASK   ( 0x01 )
#define AT86RF215_BBCn_IRQS_RXFE_MASK   ( 0x02 )
#define AT86RF215_BBCn_IRQS_TXFE_MASK   ( 0x10 )

#define AT86RF215_RFn_IRQS_EDC_MASK     ( 0x02 )

#define AT86RF215_BBCn_PC_BBEN_MASK     ( 0x01 << 2 )

#define AT86RF215_RFn_EDC_EDM_AUTO      ( 0x00 << 0 )
#define AT86RF215_RFn_EDC_EDM_SINGLE    ( 0x01 << 0 )
#define AT86RF215_RFn_EDC_EDM_CONT      ( 0x02 << 0 )
#define AT86RF215_RFn_EDC_EDM_OFF       ( 0x03 << 0 )

#define AT86RF215_RFn_EDD_DF_(x)        ( ( x & 0x3F ) << 2 )
#define AT86RF215_RFn_EDD_DTB_2_US      ( 0x00 << 0 )
#define AT86RF215_RFn_EDD_DTB_8_US      ( 0x01 << 0 )
#define AT86RF215_RFn_EDD_DTB_32_US     ( 0x02 << 0 )
#define AT86RF215_RFn_EDD_DTB_128_US    ( 0x03 << 0 )

#define AT86RF215_BBCn_PC_FCST_16_BIT   ( 2 )
#define AT86RF215_BBCn_PC_FCST_32_BIT   ( 4 )

#define AT86RF215_EDV_INVALID           ( 127 )

#define AT86RF215_CCA_RETRIES           ( 2 )
#define AT86RF215_CCA_DELAY_US          ( 200 )

#define AT86RF215_CSMA_RETRIES          ( 3 )
#define AT86RF215_CSMA_DELAY_US         ( 100 )
#define AT86RF215_CSMA_MODULUS          ( 100 )

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

extern BoardImplementation board;

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

At86rf215::At86rf215(Spi& spi, GpioOut& pwr, GpioOut& rst, GpioOut& cs, GpioIn& irq):
  spi_(spi), pwr_(pwr), rst_(rst), cs_(cs), irq_(irq),
  callback_(this, &At86rf215::interruptHandler),
  rx09Init_(nullptr), rx09Done_(nullptr), tx09Init_(nullptr), tx09Done_(nullptr),
  rx24Init_(nullptr), rx24Done_(nullptr), tx24Init_(nullptr), tx24Done_(nullptr),
  rf09_irqm(0), rf24_irqm(0), bbc0_irqm(0), bbc1_irqm(0), useDma_(false)
{
  /* Ensure radio is off */
  off();
  
  /* Set IRQ callback */
  irq_.setCallback(&callback_);
}

void At86rf215::on(void)
{
  /* While on, all pins are high and wait */
  cs_.high();
  rst_.high();
  pwr_.high();
  
  /* Delay until board is on */
  board.delayMilliseconds(AT86RF215_DELAY_MS);
}

void At86rf215::off(void)
{
  /* While off, all pins are low */
  cs_.low();
  rst_.low();
  pwr_.low();
}

void At86rf215::hardReset(void)
{
  /* For reset, first low and wait */
  rst_.low();
  board.delayMilliseconds(AT86RF215_DELAY_MS);
  
  /* To eanble, first high and wait */
  rst_.high();
  board.delayMilliseconds(AT86RF215_DELAY_MS);
}

void At86rf215::softReset(RadioCore rc)
{
  /* Execute soft reset and wait for reset state */
  goToState(rc, RadioCommand::CMD_RESET, RadioState::STATE_RESET);
}

bool At86rf215::check(void)
{
  bool status = false;

  uint8_t pn = 0;
  uint8_t ver = 0;

  /* Read part number register */
  singleAccessRead(RF_PN, &pn);

  if (pn == PN_215)
  {
    /* Read version number register */
    singleAccessRead(RF_VN, &ver);

    if (ver == VN_1_1 || ver == VN_1_3)
    {
      status = true;
    }
  }

  return status;
}

void At86rf215::configure(RadioCore rc, const radio_settings_t* radio_settings, const frequency_settings_t* frequency_settings, uint16_t channel)
{
  uint16_t rf_base, bbc_base;
  uint32_t frequency;
  uint16_t address;
  uint8_t value;

  at86rf215_register_t irq_settings[] =
  {
    {RF09_BASE + RFn_IRQM,  AT86RF215_RFn_IRQM},
    {BBC0_BASE + BBCn_IRQM, AT86RF215_BBCn_IRQM},
    {RF24_BASE + RFn_IRQM,  AT86RF215_RFn_IRQM},
    {BBC1_BASE + BBCn_IRQM, AT86RF215_BBCn_IRQM},
  };
  uint8_t* irq_mask[] = {&rf09_irqm, &bbc0_irqm, &rf24_irqm, &bbc1_irqm};
  
  /* Calculate frequency offset depending on radio core */
  if (rc == RadioCore::CORE_RF09)
  {
    frequency = frequency_settings->frequency0;
  }
  else
  {
    frequency = frequency_settings->frequency0 - 1500000;
  }  
  
  /* Ensure channel does not go above the limit */
  if (channel > frequency_settings->channel_max)
  {
    channel = frequency_settings->channel_max;
  }
  
  at86rf215_register_t channel_settings[] = 
  {
    {RFn_CCF0L, (uint8_t) ((frequency / 25) % 256)},
    {RFn_CCF0H, (uint8_t) ((frequency / 25) / 256)},
    {RFn_CS,    (uint8_t) (frequency_settings->channel_spacing / 25)},
    {RFn_CNL,   (uint8_t) (channel % 256)},
    {RFn_CNM,   (uint8_t) (channel / 256)},
  };

  /* Get RFn and BBCn base address based on radio core */
  rf_base  = getRFRegisterAddress(rc, 0x00);
  bbc_base = getBBCRegisterAddress(rc, 0x00);
  
  /* Reduce output driver strength */
  singleAccessWrite(RF_CFG, AT86RF215_RF_CFG_DEFAULT);
  
  /* Disable clock output */
  singleAccessWrite(RF_CLKO, AT86RF215_RF_CLK0_DEFAULT);

  /* Configure default settings */
  for (uint16_t i = 0; i < sizeof(irq_settings)/sizeof(irq_settings[0]); i++)
  {
    /* Calculate register address and set value */
    address = irq_settings[i].address;
    value   = irq_settings[i].value;
    
    /* Write default IRQ settings to radio */
    singleAccessWrite(address, value);
    
    /* Store IRQ settings */
    *irq_mask[i] = value;
  }

  /* Configure RF settings */
  for (uint16_t i = 0; i < radio_settings->rf_elements; i++)
  {
    /* Calculate register address and set value */
    address = rf_base + radio_settings->rf_registers[i].address;
    value   = radio_settings->rf_registers[i].value;

    /* Write RF settings to radio */
    singleAccessWrite(address, value); 
  }

  /* Configure BBC settings */
  for (uint16_t i = 0; i < radio_settings->bbc_elements; i++)
  {
    /* Calculate register address and set value */
    address = bbc_base + radio_settings->bbc_registers[i].address;
    value   = radio_settings->bbc_registers[i].value;

    /* Write BBC settings to radio */
    singleAccessWrite(address, value); 
  }

  /* Configure channel settings */
  for (uint16_t i = 0; i < sizeof(channel_settings)/sizeof(channel_settings[0]); i++)
  {
    /* Calculate register address and set value */
    address = rf_base + channel_settings[i].address;
    value   = channel_settings[i].value;
    
    /* Write channel settings to radio */
    singleAccessWrite(address, value);
  }
  
  /* Get CRC length based on radio configuration */
  crc_length = getCRCLength(rc);
}

void At86rf215::sleep(RadioCore rc)
{
  goToState(rc, RadioCommand::CMD_TRXOFF, RadioState::STATE_TRXOFF);
}

void At86rf215::wakeup(RadioCore rc)
{
  goToState(rc, RadioCommand::CMD_TXPREP, RadioState::STATE_TXPREP);
}

void At86rf215::transmit(RadioCore rc)
{
  goToState(rc, RadioCommand::CMD_TX, RadioState::STATE_TX);
  
  if (rc == CORE_RF09 && tx09Init_ != nullptr)
  {
    tx09Init_->execute();
  } 
  else if (rc == CORE_RF24 && tx24Init_ != nullptr)
  {
    tx24Init_->execute();
  }
}

void At86rf215::receive(RadioCore rc)
{
  goToState(rc, RadioCommand::CMD_RX, RadioState::STATE_RX);
}

bool At86rf215::cca(RadioCore rc, int8_t cca_threshold, int8_t* rssi, bool* rssi_valid)
{
  uint16_t bbcn_pc_address;
  uint16_t rfn_edv_address, rfn_edc_address;
  uint16_t irqs_address;
  bool cca_status = false;
  int8_t edv;
  
  /* Set CORE, BBCn and RFn addresses */
  irqs_address    = At86rf215::getCORERegisterAddress(rc);
  bbcn_pc_address = At86rf215::getBBCRegisterAddress(rc, BBCn_PC);
  rfn_edc_address = At86rf215::getRFRegisterAddress(rc, RFn_EDC);  
  rfn_edv_address = At86rf215::getRFRegisterAddress(rc, RFn_EDV); 
  
  /* Disable BBCn_PC.BBEN bit */
  disableBitFromRegister(bbcn_pc_address, AT86RF215_BBCn_PC_BBEN_MASK);
  
  /* Go to receive mode */
  receive(rc);
      
  /* Write EDC address */
  singleAccessWrite(rfn_edc_address, AT86RF215_RFn_EDC_EDM_SINGLE);
  
  /* Wait until EDC bit is set */
  bool status;
  do
  {
    uint8_t rf_irqs;
    
    /* Read RF and BBC interrupt status */
    singleAccessRead(irqs_address, &rf_irqs);
    
    /* Check if EDC bit is set */
    status = ((rf_irqs & AT86RF215_RFn_IRQS_EDC_MASK) == AT86RF215_RFn_IRQS_EDC_MASK);
  } while(!status);
  
  /* Go back to idle mode */
  wakeup(rc);
    
  /* Read EDV address */
  singleAccessRead(rfn_edv_address, (uint8_t *) &edv);
  
  /* Read EDV address again in case it fails */
  if (edv == AT86RF215_EDV_INVALID)
  {
    /* Read EDV address */
    singleAccessRead(rfn_edv_address, (uint8_t *) &edv);
  }
  
  /* Process measurement only if valid */
  if (edv != AT86RF215_EDV_INVALID)
  {
    /* Copy CCA result */
    *rssi = edv;
    *rssi_valid = true;
    
    /* If read energy is below threshold */
    if (edv < cca_threshold)
    {
      /* CCA was successful */
      cca_status = true;
    }
  }
  else
  {
    *rssi_valid = false;
  }
  
  /* Re-enable BBCn_PC.BBEN bit */
  enableBitFromRegister(bbcn_pc_address, AT86RF215_BBCn_PC_BBEN_MASK);
  
  return cca_status;
}

bool At86rf215::csma(RadioCore rc, int8_t cca_threshold, uint8_t* retries, int8_t* rssi)
{ 
  int8_t rssi_values[AT86RF215_CSMA_RETRIES] = {0};
  uint8_t rssi_position = 0;
  int8_t rssi_mean;
  
  /* Set number of CSMA retries and CSMA status */
  uint8_t csma_retries = AT86RF215_CSMA_RETRIES;
  bool cca_status = false;
  
  /* Repeat until CCA succeeds and we have retries left */
  do {
    int8_t rssi;
    bool rssi_valid;
    
    /* Perform CCA and store RSSI value */
    cca_status = cca(rc, cca_threshold, &rssi, &rssi_valid);
    
    /* Check if RSSI is valid */
    if (rssi_valid)
    {
      /* Update RSSI measurement */
      rssi_values[rssi_position] = rssi;
      
      /* Increment RSSI position */
      rssi_position++;
    }
    
    /* Decrement CSMA retries */
    csma_retries--;
    
    /* If CCA not successful and more CSMA retries*/
    if (!cca_status && csma_retries > 0)
    {
      uint32_t delay_us;
      uint16_t slots;
      
      /* Calculate slots to multiply */
      slots = prng.get() % AT86RF215_CSMA_MODULUS;
      
      /* Calculate delay based on slots */
      delay_us = slots * AT86RF215_CSMA_DELAY_US;
      
      /* Delay microseconds */
      board.delayMicroseconds(delay_us);
    }
  } while(!cca_status && csma_retries > 0);

  /* If CCA is successful and we more CSMA retries */
  if (cca_status && csma_retries > 0)
  {
    /* Calculate RSSI mean */
    int32_t accumul = 0;
    for (uint8_t i = 0; i < rssi_position; i++)
    {
      accumul += rssi_values[i];
    }
    rssi_mean = accumul / rssi_position;
    
    /* Update CSMA retries and RSSI */
    *retries = rssi_position;
    *rssi = rssi_mean;
  }
  
  return cca_status;
}

void At86rf215::setRxCallbacks(RadioCore rc, Callback* rxInit, Callback* rxDone)
{
  if (rc == RadioCore::CORE_RF09)
  {
    rx09Init_ = rxInit;
    rx09Done_ = rxDone;
  } 
  else if  (rc == RadioCore::CORE_RF24)
  {
    rx24Init_ = rxInit;
    rx24Done_ = rxDone;
  }
}

void At86rf215::setTxCallbacks(RadioCore rc, Callback* txInit, Callback* txDone)
{
  if (rc == RadioCore::CORE_RF09)
  {
    tx09Init_ = txInit;
    tx09Done_ = txDone;
  } 
  else if  (rc == RadioCore::CORE_RF24)
  {
    tx24Init_ = txInit;
    tx24Done_ = txDone;
  }
}

void At86rf215::enableInterrupts(void)
{
  irq_.enableInterrupts();
}

void At86rf215::disableInterrupts(void)
{
  irq_.disableInterrupts();
}

At86rf215::RadioResult At86rf215::getRSSI(RadioCore rc, int8_t* rssi)
{
  uint16_t address;
  int8_t value;
  
  /* Select RSSI register address */
  address = getRFRegisterAddress(rc, RFn_RSSI);
  
  /* Read RSSI value */
  singleAccessRead(address, (uint8_t *)&value);

  /* Check that the RSSI value is valid */
  if (value == AT86RF215_RSSI_MAX_VALUE)
  {
    *rssi = 0;
    return RadioResult::Error;
  }
  else
  {
    *rssi = value;
    return RadioResult::Success;
  }
}

At86rf215::RadioResult At86rf215::getED(RadioCore rc, int8_t* ed)
{
  uint16_t address;
  int8_t value;

  /* Select ED register address */
  address = getRFRegisterAddress(rc, RFn_EDV);
  
  /* Read ED value */
  singleAccessRead(address, (uint8_t *)&value);

  /* Check that ED value is valid */
  if (value < AT86RF215_ED_MIN_VALUE || value > AT86RF215_ED_MAX_VALUE)
  {
    *ed = 0;
    return RadioResult::Error;
  }
  else
  {
    *ed = value;
    return RadioResult::Success;
  }
}

At86rf215::RadioResult At86rf215::setTransmitPower(RadioCore rc, TransmitPower power)
{
  uint16_t address;
  int8_t value;
  
  /* Select PAC register address */
  address = getRFRegisterAddress(rc, RFn_PAC);
  
  /* Read PAC register */
  singleAccessRead(address, (uint8_t *)&value);
    
  /* Clear the PAC.PACUR bits */
  value &= (~AT86RF215_RFn_PAC_PACUR_MASK);
  
  /* Clear the PAC.TXPWR bits */
  value &= (~AT86RF215_RFn_PAC_TXPWR_MASK);
  
  /* Set the PAC.PACUR bits */
  value |= (AT86RF215_RFn_PAC_PACUR_3dB << AT86RF215_RFn_PAC_PACUR_SHIFT);
  
  /* Set the TXPWR bits */
  value |= (power & AT86RF215_RFn_PAC_TXPWR_MASK);
  
  /* Write PAC register */
  singleAccessWrite(address, value);
  
  return RadioResult::Success;
}

At86rf215::RadioResult At86rf215::loadPacket(RadioCore rc, uint8_t* data, uint16_t length)
{
  uint16_t bbc_txfll;
  uint16_t bbc_fbtxs;
  uint8_t scratch[2];
  
  /* Get BBC and FB register address */
  bbc_txfll = getBBCRegisterAddress(rc, BBCn_TXFLL);
  bbc_fbtxs = getFBRegisterAddress(rc, BBCn_FBTXS);
    
  /* Account for CRC length */
  length += crc_length;
  
  /* Set packet length */
  scratch[0] = (uint8_t)((length >> 0) & 0xFF); /* low byte */
  scratch[1] = (uint8_t)((length >> 8) & 0x07); /* high byte */
   
  /* Write the packet length */
  blockAccessWrite(bbc_txfll, scratch, 2);
  
  /* Send the packet payload */
  blockAccessWrite(bbc_fbtxs, data, length);
  
  return RadioResult::Success;
}

At86rf215::RadioResult At86rf215::getPacket(RadioCore rc, uint8_t* buffer, uint16_t* length, int8_t* rssi, int8_t* lqi, bool* crc)
{
  uint16_t bbc_rxfll, bbc_pc;
  uint16_t rf_rssi, rf_edv;
  uint16_t bbc_fbrxs;
  uint16_t scratch;
  uint8_t scratch_buffer[2];
  uint8_t byte;
  
  /* Get BBC, RF and FB register address */
  bbc_rxfll = getBBCRegisterAddress(rc, BBCn_RXFLL);
  bbc_pc    = getBBCRegisterAddress(rc, BBCn_PC);
  rf_rssi   = getRFRegisterAddress(rc, RFn_RSSI);
  rf_edv    = getRFRegisterAddress(rc, RFn_EDV);
  bbc_fbrxs = getFBRegisterAddress(rc, BBCn_FBRXS);
  
  /* Read the packet packet length */
  blockAccessRead(bbc_rxfll, scratch_buffer, 2);
  
  /* Compute packet length */
  scratch  = (scratch_buffer[0] << 0);
  scratch |= (scratch_buffer[1] << 8);
  
  /* Account for CRC length */
  scratch -= crc_length;
  
  /* Avoid overflowing the buffer */
  if (scratch > *length)
  {
    return RadioResult::Error;
  }
  else
  {
    /* Update packet length */
    *length = scratch;
  }
  
  /* Read the packet payload */
  blockAccessRead(bbc_fbrxs, buffer, scratch);
  
  /* Read packet RSSI */
  singleAccessRead(rf_rssi, &byte);
  *rssi = byte;

  /* Read packet LQI */
  singleAccessRead(rf_edv, &byte);
  *lqi = byte;
  
  /* Read packet CRC */
  singleAccessRead(bbc_pc, &byte);
  *crc = (bool)((byte & AT86RF215_BBCn_PC_FCSOK_MASK) == AT86RF215_BBCn_PC_FCSOK_MASK);
  
  return RadioResult::Success;
}

void At86rf215::setContinuousTransmission(RadioCore rc, bool enable)
{
  uint16_t bbc_pc;
  uint8_t value;
  
  /* Get BBC register address */
  bbc_pc = getBBCRegisterAddress(rc, BBCn_PC); 
  
  /* Read BBCn_PC register */
  singleAccessRead(bbc_pc, &value);
  
  /* Enable or disable the continuous transmission bit */
  if (enable)
  {
    value |= AT86RF215_BBCn_PC_CTX_MASK;
  }
  else
  {
    value ^= AT86RF215_BBCn_PC_CTX_MASK;
  }
  
  /* Write BBCn_PC register */
  singleAccessWrite(bbc_pc, value);
}

/*=============================== protected =================================*/

void At86rf215::interruptHandler(void)
{
  /* Read interrupt registers */
  uint8_t rf09_irqs, rf24_irqs;
  uint8_t bbc0_irqs, bbc1_irqs;

  /* Read RF and BBC interrupt status */
  singleAccessRead(RF09_IRQS, &rf09_irqs);
  singleAccessRead(RF24_IRQS, &rf24_irqs);
  singleAccessRead(BBC0_IRQS, &bbc0_irqs);
  singleAccessRead(BBC1_IRQS, &bbc1_irqs);

  /* Call RF09 interrupt handler */
  if ((rf09_irqs & rf09_irqm) || (bbc0_irqs & bbc0_irqm))
  {
    interruptHandler_rf09(rf09_irqs, bbc0_irqs);
  }

  /* Call RF24 interrupt handler */
  if ((rf24_irqs & rf24_irqm) || (bbc1_irqs & bbc1_irqm))
  {
    interruptHandler_rf24(rf24_irqs, bbc1_irqs);
  }
}

void At86rf215::interruptHandler_rf09(uint8_t rf_irqs, uint8_t bbc_irqs)
{
  /* Receiver start of frame */
  if ((bbc_irqs & AT86RF215_BBCn_IRQS_RXFS_MASK) && (rx09Init_ != nullptr))
  {
    rx09Init_->execute();
  }

  /* Receiver end of frame */
  if ((bbc_irqs & AT86RF215_BBCn_IRQS_RXFE_MASK) && (rx09Done_ != nullptr))
  {
    rx09Done_->execute();
  }

  /* Transmitter start of frame */
  if (tx09Init_ != nullptr)
  {
    /* tx09Init_->execute(); */
  }

  /* Transmitter end of frame */
  if ((bbc_irqs & AT86RF215_BBCn_IRQS_TXFE_MASK) && (tx09Done_ != nullptr))
  {
    tx09Done_->execute();
  }
}

void At86rf215::interruptHandler_rf24(uint8_t rf_irqs, uint8_t bbc_irqs)
{
  /* Receiver start of frame */
  if ((bbc_irqs & AT86RF215_BBCn_IRQS_RXFS_MASK) && (rx24Init_ != nullptr))
  {
    rx24Init_->execute();
  }

  /* Receiver end of frame */
  if ((bbc_irqs & AT86RF215_BBCn_IRQS_RXFE_MASK) && (rx24Done_ != nullptr))
  {
    rx24Done_->execute();
  }
  
  /* Transmitter start of frame */
  if (tx24Init_ != nullptr)
  {
    /* tx24Init_->execute(); */
  }

  /* Transmitter end of frame */
  if ((bbc_irqs & AT86RF215_BBCn_IRQS_TXFE_MASK) && (tx24Done_ != nullptr))
  {
    tx24Done_->execute();
  }
}

/*================================ private ==================================*/

At86rf215::RadioState At86rf215::getState(RadioCore rc)
{
  RadioState state;
  uint16_t address;
  
  /* Select registers to use */
  address = getRFRegisterAddress(rc, RFn_STATE);
  
  /* Read radio state */
  singleAccessRead(address, (uint8_t*) &state); 

  return state;
}

void At86rf215::goToState(RadioCore rc, RadioCommand cmd, RadioState target)
{
  RadioState current;

  /* Write command to radio core */
  writeCmd(rc, cmd);

  /* Repeat until the core is at target state */
  do {
    /* Read radio core state */
    current = getState(rc);
    if (target != current)
    {
      board.delayMicroseconds(AT86RF215_DELAY_US);
    }
  } while (target != current);
}

void At86rf215::writeCmd(RadioCore rc, RadioCommand cmd)
{
  uint16_t address;
  
  /* Select registers to use */
  address = getRFRegisterAddress(rc, RFn_CMD);
  
  /* Write radio command */
  singleAccessWrite(address, (uint8_t) cmd);
}

void At86rf215::singleAccessRead(uint16_t address, uint8_t* value)
{
  uint8_t spi_tx_transaction[3];
  uint8_t spi_rx_transaction[3];
  uint8_t address_hi, address_lo;
     
  /* Prepare device address */
  address_hi = (uint8_t) ((address & 0xFF00) >> 8);
  address_lo = (uint8_t) ((address & 0x00FF) >> 0);

  /* Prepare buffer for SPI transaction */
  spi_tx_transaction[0] = READ_CMD | address_hi;
  spi_tx_transaction[1] = address_lo;
  spi_tx_transaction[2] = 0x00;

  /* Activate CS */
  cs_.low();

  /* Write the SPI transaction */
  spi_.rwByte(spi_tx_transaction, 3, spi_rx_transaction, 3, useDma_);

  /* Deactivate CS */
  cs_.high();

  /* Return transaction value */
  *value = spi_rx_transaction[2];
}

void At86rf215::singleAccessWrite(uint16_t address, uint8_t value)
{
  uint8_t spi_tx_transaction[3];
  uint8_t spi_rx_transaction[3];
  uint8_t address_hi, address_lo;

  /* Prepare device address */
  address_hi = (uint8_t) ((address & 0xFF00) >> 8);
  address_lo = (uint8_t) ((address & 0x00FF) >> 0);

  /* Prepare buffer for SPI transaction */
  spi_tx_transaction[0] = WRITE_CMD | address_hi;
  spi_tx_transaction[1] = address_lo;
  spi_tx_transaction[2] = value;

  /* Activate CS */
  cs_.low();

  /* Write the SPI transaction */
  spi_.rwByte(spi_tx_transaction, 3, spi_rx_transaction, 3, useDma_);

  /* Deactivate CS */
  cs_.high();
}

void At86rf215::blockAccessRead(uint16_t address, uint8_t* values, uint16_t length)
{
  uint8_t spi_tx_transaction[2];
  uint8_t spi_rx_transaction[2];
  uint8_t address_hi, address_lo;

  /* Prepare device address */
  address_hi = (uint8_t) ((address & 0xFF00) >> 8);
  address_lo = (uint8_t) ((address & 0x00FF) >> 0);

  /* Prepare buffer for SPI transaction */
  spi_tx_transaction[0] = READ_CMD | address_hi;
  spi_tx_transaction[1] = address_lo;

  /* Activate CS */
  cs_.low();
  
  /* Send device address */
  spi_.rwByte(spi_tx_transaction, 2, spi_rx_transaction, 2, useDma_);
  
  /* Receive device bytes */
  spi_.rwByte(NULL, 0, values, length, useDma_);

  /* Deactivate CS */
  cs_.high();
}

void At86rf215::blockAccessWrite(uint16_t address, uint8_t* values, uint16_t length)
{
  uint8_t spi_tx_transaction[2];
  uint8_t spi_rx_transaction[2];
  uint8_t address_hi, address_lo;

  /* Prepare device address */
  address_hi = (uint8_t) ((address & 0xFF00) >> 8);
  address_lo = (uint8_t) ((address & 0x00FF) >> 0);

  /* Prepare buffer for SPI transaction */
  spi_tx_transaction[0] = WRITE_CMD | address_hi;
  spi_tx_transaction[1] = address_lo;

  /* Activate CS */
  cs_.low();
  
  /* Send device address */
  spi_.rwByte(spi_tx_transaction, 2, spi_rx_transaction, 2, useDma_);
  
  /* Send payload */
  spi_.rwByte(values, length, NULL, 0, useDma_);
  
  /* Deactivate CS */
  cs_.high();
}

inline uint16_t At86rf215::getCORERegisterAddress(RadioCore rc)
{
  uint16_t address;
  
  /* Select CORE address */
  switch(rc)
  {
    case RadioCore::CORE_RF09:
      address = RF09_IRQS;
      break;
    case RadioCore::CORE_RF24:
      address = RF24_IRQS;
      break;
    default:
      address = 0;
      break;
  }
  
  return address;
}

inline uint16_t At86rf215::getRFRegisterAddress(RadioCore rc, uint16_t address)
{
  /* Select RF register address based on radio core */
  switch(rc)
  {
    case RadioCore::CORE_RF09:
      address = RF09_BASE + address;
      break;
    case RadioCore::CORE_RF24:
      address = RF24_BASE + address;
      break;
    default:
      address = 0;
      break;
  }
  
  return address;
}

inline uint16_t At86rf215::getBBCRegisterAddress(RadioCore rc, uint16_t address)
{
  /* Select BBC register address based on radio core */
  switch(rc)
  {
    case RadioCore::CORE_RF09:
      address = BBC0_BASE + address;
      break;
    case RadioCore::CORE_RF24:
      address = BBC1_BASE + address;
      break;
    default:
      return 0;
      break;
  }
  
  return address;
}

inline uint16_t At86rf215::getFBRegisterAddress(RadioCore rc, uint16_t address)
{
  /* Select FB register address based on radio core */
  switch(rc)
  {
    case RadioCore::CORE_RF09:
      address = BBC0_FB_BASE + address;
      break;
    case RadioCore::CORE_RF24:
      address = BBC1_FB_BASE + address;
      break;
    default:
      return 0;
      break;
  }
  
  return address;
}

inline uint16_t At86rf215::getCRCLength(RadioCore rc)
{
  uint16_t address;
  uint8_t value;
  
  /* Get BBC register address */
  address = getBBCRegisterAddress(rc, BBCn_PC);
  
  /* Read BBCn_PC register */
  singleAccessRead(address, &value);
  
  /* If FCST is active CRC = 16 bits, otherwise CRC = 32 bits */
  if ((value & AT86RF215_BBCn_PC_FCST_MASK) == AT86RF215_BBCn_PC_FCST_MASK)
  {
    return AT86RF215_BBCn_PC_FCST_16_BIT;
  }
  else
  {
    return AT86RF215_BBCn_PC_FCST_32_BIT;
  }
}

inline void At86rf215::enableBitFromRegister(uint16_t address, uint32_t mask)
{
  uint8_t scratch;
  singleAccessRead(address, &scratch);
  scratch |= mask;
  singleAccessWrite(address, scratch);
}

inline void At86rf215::disableBitFromRegister(uint16_t address, uint32_t mask)
{
  uint8_t scratch;
  singleAccessRead(address, &scratch);
  scratch &= ~mask;
  singleAccessWrite(address, scratch);
}
