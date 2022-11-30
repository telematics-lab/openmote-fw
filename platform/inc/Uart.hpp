/**
 * @file       Uart.hpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       November, 2018
 * @brief
 *
 * @copyright  Copyright 2018, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef UART_HPP_
#define UART_HPP_

#include <stdint.h>

#include "Callback.hpp"
#include "Semaphore.hpp"

class Gpio;
class Buffer;

struct UartConfig;

class Uart
{

friend class InterruptHandler;

public:
    Uart(Gpio& rx, Gpio& tx, UartConfig& config);
    void enable(uint32_t baudrate = 0);
    void enable(uint32_t baudrate, uint32_t mode);
    void sleep(void);
    void wakeup(void);
    void setRxCallback(Callback* callback);
    void setTxCallback(Callback* callback);
    void enableInterrupts(void);
    void disableInterrupts(void);
    void enableDMA(void);
    void disableDMA(void);
    void rxLock(void);
    void txLock(void);
    void rxUnlock(void);
    void txUnlock(void);
    void rxUnlockFromInterrupt(void);
    void txUnlockFromInterrupt(void);

	void flush(void);
    
    bool readByte(uint8_t* byte);
    void writeByte(uint8_t byte);
    
    uint32_t readByte(uint8_t* buffer, uint32_t length);
    uint32_t writeByte(uint8_t* buffer, uint32_t length);
    
    uint32_t readBytes(uint8_t* buffer, uint32_t length);
    uint32_t writeBytes(const uint8_t* buffer, uint32_t length);
    
    bool readBytes(Buffer& buffer, bool& finished);
    bool writeBytes(Buffer& buffer);
public:
    bool operator==(const Uart& other);
protected:
    UartConfig& getConfig(void);
    void interruptHandler(void);
private:
    void interruptHandlerRx(void);
    void interruptHandlerTx(void);
private:
    Gpio& rx_;
    Gpio& tx_;
    UartConfig& config_;

    SemaphoreBinary rxSemaphore_;
    SemaphoreBinary txSemaphore_;
    
    SemaphoreBinary dmaComplete_;

    Callback* rx_callback_;
    Callback* tx_callback_;
    
    bool rx_timeout_;
    bool dma_finished_;
};

#endif /* UART_HPP_ */
