/**
 * @file       At86rf215.hpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2016
 * @brief
 *
 * @copyright  Copyright 2016, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef AT86RF215_HPP_
#define AT86RF215_HPP_

#include "At86rf215_regs.h"
#include "At86rf215_conf.h"

#include "Gpio.hpp"
#include "Spi.hpp"
#include "Callback.hpp"

class At86rf215
{

friend class InterruptHandler;

public:

typedef enum
{
  CORE_RF09 = 0x00,
  CORE_RF24 = 0x01
} RadioCore;

typedef enum
{
  STATE_TRXOFF = RF_STATE_TRXOFF,
  STATE_TXPREP = RF_STATE_TXPREP,
  STATE_TX     = RF_STATE_TX,
  STATE_RX     = RF_STATE_RX,
  STATE_TRANS  = RF_STATE_TRANS,
  STATE_RESET  = RF_STATE_RESET
} RadioState;

typedef enum
{
  CMD_NOP    = RF_CMD_NOP,
  CMD_SLEEP  = RF_CMD_SLEEP,
  CMD_TRXOFF = RF_CMD_TRXOFF, 
  CMD_TXPREP = RF_CMD_TXPREP,
  CMD_TX     = RF_CMD_TX,
  CMD_RX     = RF_CMD_RX,
  CMD_RESET  = RF_CMD_RESET
} RadioCommand;

typedef enum
{
  TX_POWER_MIN = 0, 
  TX_POWER_0 = 0, TX_POWER_1, TX_POWER_2, TX_POWER_3, TX_POWER_4, TX_POWER_5,
  TX_POWER_6,  TX_POWER_7,  TX_POWER_8,  TX_POWER_9,  TX_POWER_10,
  TX_POWER_11, TX_POWER_12, TX_POWER_13, TX_POWER_14, TX_POWER_15,
  TX_POWER_16, TX_POWER_17, TX_POWER_18, TX_POWER_19, TX_POWER_20,
  TX_POWER_21, TX_POWER_22, TX_POWER_23, TX_POWER_24, TX_POWER_25,
  TX_POWER_26, TX_POWER_27, TX_POWER_28, TX_POWER_29, TX_POWER_30,
  TX_POWER_31 = 31, TX_POWER_MAX = 31
} TransmitPower;

typedef enum
{
  Error       = -1,
  Success     =  0
} RadioResult;

public:
  At86rf215(Spi& spi, GpioOut& pwr, GpioOut& rst, GpioOut& cs, GpioIn& irq);
  
  void on(void);
  void off(void);
  
  void hardReset(void);
  void softReset(RadioCore rc);

  bool check(void);
  
  void configure(RadioCore rc, const radio_settings_t* radio_settings, const frequency_settings_t* frequency_settings, uint16_t channel);
  
  void sleep(RadioCore rc);
  void wakeup(RadioCore rc);
  void transmit(RadioCore rc);
  void receive(RadioCore rc);
  
  bool cca(RadioCore rc, int8_t threshold, int8_t* rssi, bool* rssi_valid);
  bool csma(RadioCore rc, int8_t cca_threshold, uint8_t* retries, int8_t* rssi);

  void setRxCallbacks(RadioCore rc, Callback* rxInit, Callback* rxDone);
  void setTxCallbacks(RadioCore rc, Callback* txInit, Callback* txDone);
  void enableInterrupts(void);
  void disableInterrupts(void);
  
  RadioResult getRSSI(RadioCore rc, int8_t* rssi);
  RadioResult getED(RadioCore rc, int8_t* edv);
  RadioResult setTransmitPower(RadioCore rc, TransmitPower power);
  
  RadioResult loadPacket(RadioCore rc, uint8_t* data, uint16_t length);
  RadioResult getPacket(RadioCore rc, uint8_t* buffer, uint16_t* length, int8_t* rssi, int8_t* lqi, bool* crc);
public:
  void setContinuousTransmission(RadioCore rc, bool enable);
protected:
  void interruptHandler(void);
  void interruptHandler_rf24(uint8_t rf24_irqs, uint8_t bbc24_irqs); 
  void interruptHandler_rf09(uint8_t rf09_irqs, uint8_t bbc09_irqs);
private:
  RadioState getState(RadioCore rc);
  void goToState(RadioCore rc, RadioCommand cmd, RadioState target);
  void writeCmd(RadioCore rc, RadioCommand cmd);
private:
  void singleAccessRead(uint16_t address, uint8_t* value);
  void singleAccessWrite(uint16_t address, uint8_t value);
  void blockAccessRead(uint16_t address, uint8_t* values, uint16_t length);
  void blockAccessWrite(uint16_t address, uint8_t* values, uint16_t length);
private:
  uint16_t getCORERegisterAddress(RadioCore rc);
  uint16_t getRFRegisterAddress(RadioCore rc, uint16_t address);
  uint16_t getBBCRegisterAddress(RadioCore rc, uint16_t address);
  uint16_t getFBRegisterAddress(RadioCore rc, uint16_t address);
  uint16_t getCRCLength(RadioCore rc);
private:  
  void enableBitFromRegister(uint16_t address, uint32_t mask);
  void disableBitFromRegister(uint16_t address, uint32_t mask);
private:
  Spi& spi_;
  
  GpioOut& pwr_;
  GpioOut& rst_;
  GpioOut& cs_;
  GpioIn& irq_;

  GenericCallback<At86rf215> callback_;

  Callback* rx09Init_;
  Callback* rx09Done_;
  Callback* tx09Init_;
  Callback* tx09Done_;

  Callback* rx24Init_;
  Callback* rx24Done_;
  Callback* tx24Init_;
  Callback* tx24Done_;
  
  uint8_t rf09_irqm, rf24_irqm;
  uint8_t bbc0_irqm, bbc1_irqm;

  uint8_t crc_length;
  
  bool useDma_;
};

#endif /* AT86RF215_HPP_ */
