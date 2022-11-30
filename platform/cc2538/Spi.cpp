/**
 * @file       Spi.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       November, 2018
 * @brief
 *
 * @copyright  Copyright 2018, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include "Gpio.hpp"
#include "InterruptHandler.hpp"
#include "Spi.hpp"

#include "platform_includes.h"
#include "platform_types.hpp"

/*================================ define ===================================*/

#define SSI_PRIO                    ( 0xF0 )

#define SSI_DATA_REGISTER           (void *)(config_.base + SSI_O_DR)

#define SSI0_TX_CHANNEL             ( UDMA_CH11_SSI0TX ) 
#define SSI0_RX_CHANNEL             ( UDMA_CH10_SSI0RX )
#define SSI0_TX_CHANNEL_SEL         ( UDMA_CH11_SSI0TX | UDMA_PRI_SELECT ) 
#define SSI0_RX_CHANNEL_SEL         ( UDMA_CH10_SSI0RX | UDMA_PRI_SELECT )

#define SSI1_TX_CHANNEL             ( UDMA_CH25_SSI1TX ) 
#define SSI1_RX_CHANNEL             ( UDMA_CH24_SSI1RX )
#define SSI1_TX_CHANNEL_SEL         ( UDMA_CH25_SSI1TX | UDMA_PRI_SELECT ) 
#define SSI1_RX_CHANNEL_SEL         ( UDMA_CH24_SSI1RX | UDMA_PRI_SELECT )

#define SSI_RX_CONTROL_INCREMENT_0  ( UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_NONE | UDMA_ARB_1 )
#define SSI_RX_CONTROL_INCREMENT_8  ( UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8    | UDMA_ARB_1 )
#define SSI_TX_CONTROL_INCREMENT_0  ( UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_NONE | UDMA_ARB_1 )
#define SSI_TX_CONTROL_INCREMENT_8  ( UDMA_SIZE_8 | UDMA_SRC_INC_8    | UDMA_DST_INC_NONE | UDMA_ARB_1 )

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

Spi::Spi(Gpio& miso, Gpio& mosi, Gpio& clk, SpiConfig& config):
  miso_(miso), mosi_(mosi), clk_(clk), config_(config),
  rx_callback_(nullptr), tx_callback_(nullptr)
{
}

void Spi::enable(uint32_t baudrate)
{
  /* Get MISO, MOSI and CLK config */
  const GpioConfig& miso = miso_.getGpioConfig();
  const GpioConfig& mosi = mosi_.getGpioConfig();
  const GpioConfig& clk  = clk_.getGpioConfig();

  /* Store baudrate in configuration */
  if (baudrate != 0) {
    config_.baudrate = baudrate;
  }
  
  /* Enable peripheral except in deep sleep modes (e.g. LPM1, LPM2, LPM3) */
  SysCtrlPeripheralEnable(config_.peripheral);
  SysCtrlPeripheralSleepEnable(config_.peripheral);
  SysCtrlPeripheralDeepSleepDisable(config_.peripheral);

  /* Reset peripheral previous to configuring it */
  SSIDisable(config_.base);

  /* Set IO clock as SPI clock source */
  SSIClockSourceSet(config_.base, config_.clock);

  /* Configure the MISO, MOSI and CLK pins as peripheral */
  IOCPinConfigPeriphInput(miso.port, miso.pin, miso.ioc);
  IOCPinConfigPeriphOutput(mosi.port, mosi.pin, mosi.ioc);
  IOCPinConfigPeriphOutput(clk.port, clk.pin, clk.ioc);
  
  /* Configure MISO, MOSI and CLK GPIOs */
  GPIOPinTypeSSI(miso.port, miso.pin);
  GPIOPinTypeSSI(mosi.port, mosi.pin);
  GPIOPinTypeSSI(clk.port, clk.pin);
  
  /* Configure the SPI clock according to its input */
  if (config_.clock == SSI_CLOCK_SYSTEM)
  {
    uint32_t clock = SysCtrlClockGet();
    SSIConfigSetExpClk(config_.base, clock, config_.protocol, \
                       config_.mode, config_.baudrate, config_.datawidth);
  }
  else if (config_.clock == SSI_CLOCK_PIOSC)
  {
    uint32_t clock = SysCtrlIOClockGet();
    SSIConfigSetExpClk(config_.base, clock, config_.protocol, \
                       config_.mode, config_.baudrate, config_.datawidth);
  }
  else
  {
    while(true)
      ;
  }

  /* Enable the SPI module */
  SSIEnable(config_.base);
}

void Spi::sleep(void)
{
  /* Get MISO, MOSI and CLK config */
  const GpioConfig& miso = miso_.getGpioConfig();
  const GpioConfig& mosi = mosi_.getGpioConfig();
  const GpioConfig& clk  = clk_.getGpioConfig();

  /* Disable the SPI module */
  SSIDisable(config_.base);

  /* Configure the MISO, MOSI and CLK pins as output */
  GPIOPinTypeGPIOInput(miso.port, miso.pin);
  GPIOPinTypeGPIOInput(mosi.port, mosi.pin);
  GPIOPinTypeGPIOInput(clk.port, clk.pin);
}

void Spi::wakeup(void)
{
  enable();
}

void Spi::setRxCallback(Callback* callback)
{
  rx_callback_ = callback;
}

void Spi::setTxCallback(Callback* callback)
{
  tx_callback_ = callback;
}

void Spi::enableInterrupts(void)
{
  /* Register the interrupt handler */
  InterruptHandler::getInstance().setInterruptHandler(*this);

  /* Clear the SPI interrupts */
  SSIIntClear(config_.base, (SSI_TXFF | SSI_RXFF | SSI_RXTO | SSI_RXOR));
  
  /* Enable the SPI interrupt */
  SSIIntEnable(config_.base, (SSI_TXFF | SSI_RXFF | SSI_RXTO | SSI_RXOR));

  /* Set the SPI interrupt priority */
  IntPrioritySet(config_.interrupt, SSI_PRIO);
  
  /* Enable the SPI interrupt */
  IntEnable(config_.interrupt);
}

void Spi::disableInterrupts(void)
{
  /* Disable the SPI interrupt */
  SSIIntDisable(config_.base, (SSI_TXFF | SSI_RXFF | SSI_RXTO | SSI_RXOR));

  /* Disable the SPI interrupt */
  IntDisable(config_.interrupt);
}

uint8_t Spi::rwByte(uint8_t byte)
{
  uint32_t ret;

  /* Push a byte */
  SSIDataPut(config_.base, byte);

  /* Wait until it is complete */
  while(SSIBusy(config_.base))
    ;

  /* Read a byte */
  SSIDataGet(config_.base, &ret);

  return (uint8_t)(ret & 0xFF);
}

bool Spi::rwByte(uint8_t* transmitBuffer, uint32_t transmitLength, uint8_t* receiveBuffer, uint32_t receiveLength, bool dma)
{
  if(dma)
  {
    rwByteDma(transmitBuffer, transmitLength, receiveBuffer, receiveLength);
  }
  else
  {
    rwByteNoDma(transmitBuffer, transmitLength, receiveBuffer, receiveLength);
  }
}

/*=============================== protected =================================*/

SpiConfig& Spi::getConfig(void)
{
    return config_;
}

void Spi::interruptHandler(void)
{
  uint32_t status;
  
  /* Read interrupt source */
  status = SSIIntStatus(config_.base, true);

  /* Clear SPI interrupts */
  SSIIntClear(config_.interrupt, status);

  /* Process TX interrupt */
  if (status & SSI_TXFF) {
    interruptHandlerTx();
  }

  /* Process RX interrupt */
  if (status & SSI_RXFF) {
    interruptHandlerRx();
  }
}

/*================================ private ==================================*/

bool Spi::rwByteNoDma(uint8_t* transmitBuffer, uint32_t transmitLength, uint8_t* receiveBuffer, uint32_t receiveLength)
{
  uint32_t length = 0;
  
  /* Check transmit length */
  if (transmitLength == 0)
  {
    length = receiveLength;
  }
  else if (receiveLength == 0)
  {
    length = transmitLength;
  }
  else
  {
    length = transmitLength > receiveLength ? transmitLength : receiveLength;
  }
  
  /* Iterate over all positions in transmitBuffer */
  for (uint32_t i =  0; i < length; i++)
  {
    uint32_t data;
    uint8_t byte;
    
    /* Get next byte */
    byte = (transmitLength > 0 ? transmitBuffer[i] : 0xFF);
    
    /* Transmit next byte */
    SSIDataPut(config_.base, byte);

    /* Wait until it is complete */
    while(SSIBusy(config_.base))
        ;

    /* Read a byte */
    SSIDataGet(config_.base, &data);
    
    /* Store the result */
    receiveBuffer[i] = (receiveLength > 0 ? (uint8_t)(data & 0xFF) : 0x00);
  }

  return true;
}

bool Spi::rwByteDma(uint8_t* transmitBuffer, uint32_t transmitLength, uint8_t* receiveBuffer, uint32_t receiveLength)
{
  uint32_t tx_channel, rx_channel;
  uint32_t tx_channel_sel, rx_channel_sel;
  uint8_t scratch_tx = 0xFF;
  uint8_t scratch_rx = 0xFF;
  
  /* Check transmit and receive buffer are different */
  if (transmitBuffer == receiveBuffer)
  {
    return false;
  }
  
  /* Select DMA registers based on SPI peripheral */
  if (config_.base == SSI0_BASE)
  {
    tx_channel     = SSI0_TX_CHANNEL; 
    tx_channel_sel = SSI0_TX_CHANNEL_SEL;
    rx_channel     = SSI0_RX_CHANNEL;
    rx_channel_sel = SSI0_RX_CHANNEL_SEL;
    
  }
  else if (config_.base == SSI1_BASE)
  {
    tx_channel     = SSI1_TX_CHANNEL; 
    tx_channel_sel = SSI1_TX_CHANNEL_SEL;
    rx_channel     = SSI1_RX_CHANNEL;
    rx_channel_sel = SSI1_RX_CHANNEL_SEL;
  }
  else
  {
    return false;
  }
  
  /* Setup transmit DMA */
  if (transmitLength > 0)
  {
    SSIDMAEnable(config_.base, SSI_DMA_TX);
    uDMAChannelControlSet(tx_channel_sel, SSI_TX_CONTROL_INCREMENT_8);
    uDMAChannelTransferSet(tx_channel_sel, UDMA_MODE_BASIC, transmitBuffer, SSI_DATA_REGISTER, transmitLength);
  }
  else
  {
    SSIDMAEnable(config_.base, SSI_DMA_TX);
    uDMAChannelControlSet(tx_channel_sel, SSI_TX_CONTROL_INCREMENT_0);
    uDMAChannelTransferSet(tx_channel_sel, UDMA_MODE_BASIC, &scratch_tx, SSI_DATA_REGISTER, receiveLength);
  }
  
  /* Setup receive DMA */
  if (receiveLength > 0)
  {
    SSIDMAEnable(config_.base, SSI_DMA_RX);
    uDMAChannelControlSet(rx_channel_sel, SSI_RX_CONTROL_INCREMENT_8);
    uDMAChannelTransferSet(rx_channel_sel, UDMA_MODE_BASIC, SSI_DATA_REGISTER, receiveBuffer, receiveLength);
  }
  else
  {
    SSIDMAEnable(config_.base, SSI_DMA_RX);
    uDMAChannelControlSet(rx_channel_sel, SSI_RX_CONTROL_INCREMENT_0);
    uDMAChannelTransferSet(rx_channel_sel, UDMA_MODE_BASIC, SSI_DATA_REGISTER, &scratch_rx, transmitLength);
  }
  
  /* Enable both transmit and receive DMA channels */
  uDMAChannelEnable(rx_channel);
  uDMAChannelEnable(tx_channel);
  
  /* Busy-wait until there are no more bytes to be received */
  while(uDMAChannelSizeGet(rx_channel_sel) > 0);
  
  /* Disable DMA complete interrupt for SPI */
  SSIDMADisable(config_.base, SSI_DMA_RX | SSI_DMA_TX);
  
  return true;
}

void Spi::interruptHandlerRx(void)
{
  if (tx_callback_ != nullptr)
  {
    tx_callback_->execute();
  }
}

void Spi::interruptHandlerTx(void)
{
  if (rx_callback_ != nullptr)
  {
    rx_callback_->execute();
  }
}
