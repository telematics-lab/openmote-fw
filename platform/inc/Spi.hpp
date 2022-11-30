/**
 * @file       Spi.hpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef SPI_HPP_
#define SPI_HPP_

#include <stdint.h>

#include "Callback.hpp"
#include "Mutex.hpp"

class Gpio;
struct SpiConfig;

class Spi
{

friend class InterruptHandler;

public:
    Spi(Gpio& miso, Gpio& mosi, Gpio& clk, SpiConfig& config);
    void enable(uint32_t baudrate = 0);
    void sleep(void);
    void wakeup(void);
    void setRxCallback(Callback* callback);
    void setTxCallback(Callback* callback);
    void enableInterrupts(void);
    void disableInterrupts(void);
    uint8_t rwByte(uint8_t byte);
    bool rwByte(uint8_t* transmitBuffer, uint32_t transmitLength, uint8_t* receiveBuffer, uint32_t receiveLength, bool dma = false);
private:
    bool rwByteNoDma(uint8_t* transmitBuffer, uint32_t transmitLength, uint8_t* receiveBuffer, uint32_t receiveLength);
    bool rwByteDma(uint8_t* transmitBuffer, uint32_t transmitLength, uint8_t* receiveBuffer, uint32_t receiveLength);
protected:
    SpiConfig& getConfig(void);
    void interruptHandler(void);
private:
    void interruptHandlerRx();
    void interruptHandlerTx();
private:
    Gpio& miso_;
    Gpio& mosi_;
    Gpio& clk_;

    SpiConfig& config_;

    Callback* rx_callback_;
    Callback* tx_callback_;
};

#endif /* SPI_HPP_ */
