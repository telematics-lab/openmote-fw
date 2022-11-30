/**
 * @file       InterruptHandler.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       November, 2018
 * @brief
 *
 * @copyright  Copyright 2018, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include "Aes.hpp"
#include "Dma.hpp"
#include "Gpio.hpp"
#include "I2c.hpp"
#include "InterruptHandler.hpp"
#include "Radio.hpp"
#include "RadioTimer.hpp"
#include "SleepTimer.hpp"
#include "Spi.hpp"
#include "SysTick.hpp"
#include "Timer.hpp"
#include "Uart.hpp"

#include "platform_includes.h"
#include "platform_types.hpp"

/*================================ define ===================================*/

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

InterruptHandler InterruptHandler::instance_;

GpioIn* InterruptHandler::GPIOA_interruptVector_[8];
GpioIn* InterruptHandler::GPIOB_interruptVector_[8];
GpioIn* InterruptHandler::GPIOC_interruptVector_[8];
GpioIn* InterruptHandler::GPIOD_interruptVector_[8];

Timer* InterruptHandler::TIMER0_interruptVector_[2];
Timer* InterruptHandler::TIMER1_interruptVector_[2];
Timer* InterruptHandler::TIMER2_interruptVector_[2];
Timer* InterruptHandler::TIMER3_interruptVector_[2];

SleepTimer* InterruptHandler::SleepTimer_interruptVector_;
RadioTimer* InterruptHandler::RadioTimer_interruptVector_;

Uart* InterruptHandler::UART0_interruptVector_;
Uart* InterruptHandler::UART1_interruptVector_;

I2c* InterruptHandler::I2C_interruptVector_;

Spi* InterruptHandler::SPI0_interruptVector_;
Spi* InterruptHandler::SPI1_interruptVector_;

SysTick* InterruptHandler::SysTick_interruptVector_;

Radio* InterruptHandler::Radio_interruptVector_;

Aes* InterruptHandler::Aes_interruptVector_;

GpioAdc* InterruptHandler::Adc_interruptVector_;

Dma* InterruptHandler::Dma_interruptVector_;

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

InterruptHandler &InterruptHandler::getInstance(void)
{
  /* Returns the only instance of the InterruptHandler */
  return instance_;
}

void InterruptHandler::setInterruptHandler(GpioIn& gpio)
{
  /* Get the GPIO port and pin */
  uint32_t port = gpio.getGpioConfig().port;
  uint8_t pin   = gpio.getGpioConfig().pin;

  /* Select the pin number */
  if (pin == GPIO_PIN_0)
  {
    pin = 0;
  }
  else if (pin == GPIO_PIN_1)
  {
    pin = 1;
  }
  else if (pin == GPIO_PIN_2)
  {
    pin = 2;
  }
  else if (pin == GPIO_PIN_3)
  {
    pin = 3;
  }
  else if (pin == GPIO_PIN_4)
  {
    pin = 4;
  }
  else if (pin == GPIO_PIN_5)
  {
    pin = 5;
  }
  else if (pin == GPIO_PIN_6)
  {
    pin = 6;
  }
  else if (pin == GPIO_PIN_7)
  {
    pin = 7;
  }

  /* Store a pointer to the GPIO object in the interrupt vector */
  if (port == GPIO_A_BASE)
  {
    IntPrioritySet(INT_GPIOA, 0xF0);
    GPIOA_interruptVector_[pin] = &gpio;
  }
  else if (port == GPIO_B_BASE)
  {
    IntPrioritySet(INT_GPIOB, 0xF0);
    GPIOB_interruptVector_[pin] = &gpio;
  }
  else if (port == GPIO_C_BASE)
  {
    IntPrioritySet(INT_GPIOC, 0xF0);
    GPIOC_interruptVector_[pin] = &gpio;
  }
  else if (port == GPIO_D_BASE)
  {
    IntPrioritySet(INT_GPIOD, 0xF0);
    GPIOD_interruptVector_[pin] = &gpio;
  }
}

void InterruptHandler::clearInterruptHandler(GpioIn& gpio)
{
  /* Get the GPIO port and pin */
  uint32_t port = gpio.getGpioConfig().port;
  uint8_t pin   = gpio.getGpioConfig().pin;

  /* Select the pin number */
  if (pin == GPIO_PIN_0)
  {
    pin = 0;
  }
  else if (pin == GPIO_PIN_1)
  {
    pin = 1;
  }
  else if (pin == GPIO_PIN_2)
  {
    pin = 2;
  }
  else if (pin == GPIO_PIN_3)
  {
    pin = 3;
  }
  else if (pin == GPIO_PIN_4)
  {
    pin = 4;
  }
  else if (pin == GPIO_PIN_5)
  {
    pin = 5;
  }
  else if (pin == GPIO_PIN_6)
  {
    pin = 6;
  }
  else if (pin == GPIO_PIN_7)
  {
    pin = 7;
  }

  /* Remove the pointer to the GPIO object in the interrupt vector */
  if (port == GPIO_A_BASE)
  {
    GPIOA_interruptVector_[pin] = nullptr;
  }
  else if (port == GPIO_B_BASE)
  {
    GPIOB_interruptVector_[pin] = nullptr;
  }
  else if (port == GPIO_C_BASE)
  {
    GPIOC_interruptVector_[pin] = nullptr;
  }
  else if (port == GPIO_D_BASE)
  {
    GPIOD_interruptVector_[pin] = nullptr;
  }
}

void InterruptHandler::setInterruptHandler(Timer& timer_)
{
  /* Get the TIMER base */
  uint32_t base = timer_.getConfig().base;
  uint32_t source = timer_.getConfig().source;

  if (base == GPTIMER0_BASE)
  {
    if ( (source == GPTIMER_A) || \
         (source == GPTIMER_BOTH) )
    {
      TIMER0_interruptVector_[0] = &timer_;
    }
    else if (source == GPTIMER_B)
    {
      TIMER0_interruptVector_[1] = &timer_;
    }
  }
  else if (base == GPTIMER1_BASE)
  {
    if ( (source == GPTIMER_A) || \
         (source == GPTIMER_BOTH) )
    {
      TIMER1_interruptVector_[0] = &timer_;
    }
    else if (source == GPTIMER_B)
    {
      TIMER1_interruptVector_[1] = &timer_;
    }
  }
  else if (base == GPTIMER2_BASE)
  {
    if ( (source == GPTIMER_A) || \
         (source == GPTIMER_BOTH) )
    {
      TIMER2_interruptVector_[0] = &timer_;
    }
    else if (source == GPTIMER_B)
    {
      TIMER2_interruptVector_[1] = &timer_;
    }
  }
  else if (base == GPTIMER3_BASE)
  {
    if ( (source == GPTIMER_A) || \
         (source == GPTIMER_BOTH) )
    {
      TIMER3_interruptVector_[0] = &timer_;
    }
    else if (source == GPTIMER_B)
    {
      TIMER3_interruptVector_[1] = &timer_;
    }
  }
}

void InterruptHandler::clearInterruptHandler(Timer& timer_)
{
  /* Get the TIMER base */
  uint32_t base = timer_.getConfig().base;
  uint32_t source = timer_.getConfig().source;

  if (base == GPTIMER0_BASE)
  {
    if ( (source == GPTIMER_A) || \
         (source == GPTIMER_BOTH) )
    {
      TIMER0_interruptVector_[0] = nullptr;
    }
    else if (source == GPTIMER_B)
    {
      TIMER0_interruptVector_[1] = nullptr;
    }
  }
  else if (base == GPTIMER1_BASE)
  {
    if ( (source == GPTIMER_A) || \
         (source == GPTIMER_BOTH) )
    {
      TIMER1_interruptVector_[0] = nullptr;
    }
    else if (source == GPTIMER_B)
    {
      TIMER1_interruptVector_[1] = nullptr;
    }
  }
  else if (base == GPTIMER2_BASE)
  {
    if ( (source == GPTIMER_A) || \
         (source == GPTIMER_BOTH) )
    {
      TIMER2_interruptVector_[0] = nullptr;
    }
    else if (source == GPTIMER_B)
    {
      TIMER2_interruptVector_[1] = nullptr;
    }
  }
  else if (base == GPTIMER3_BASE)
  {
    if ( (source == GPTIMER_A) || \
         (source == GPTIMER_BOTH) )
    {
      TIMER3_interruptVector_[0] = nullptr;
    }
    else if (source == GPTIMER_B)
    {
        TIMER3_interruptVector_[1] = nullptr;
    }
  }
}

void InterruptHandler::setInterruptHandler(Uart& uart_)
{
  /* Get the UART base */
  uint32_t base = uart_.getConfig().base;

  /* Store a pointer to the UART object in the interrupt vector */
  if (base == UART0_BASE)
  {
    UART0_interruptVector_ = &uart_;
  }
  else if (base == UART1_BASE)
  {
    UART1_interruptVector_ = &uart_;
  }
}

void InterruptHandler::clearInterruptHandler(Uart& uart_)
{
  /* Get the UART base */
  uint32_t base = uart_.getConfig().base;

  /* Remove the pointer to the UART object in the interrupt vector */
  if (base == UART0_BASE)
  {
    UART0_interruptVector_ = nullptr;
  }
  else if (base == UART1_BASE)
  {
    UART1_interruptVector_ = nullptr;
  }
}

void InterruptHandler::setInterruptHandler(I2c& i2c_)
{
  /* Store a pointer to the I2C object in the interrupt vector */
  I2C_interruptVector_ = &i2c_;
}

void InterruptHandler::clearInterruptHandler(I2c& i2c_)
{
  /* Remvoe the pointer to the I2C object in the interrupt vector */
  I2C_interruptVector_ = nullptr;
}

void InterruptHandler::setInterruptHandler(Spi& spi_)
{
  /* Get the SPI base */
  uint32_t base = spi_.getConfig().base;

  /* Store a pointer to the SPI object in the interrupt vector */
  if (base == SSI0_BASE)
  {
    SPI0_interruptVector_ = &spi_;
  }
  else if (base == SSI1_BASE)
  {
    SPI1_interruptVector_ = &spi_;
  }
}

void InterruptHandler::clearInterruptHandler(Spi& spi_)
{
  /* Get the SPI base */
  uint32_t base = spi_.getConfig().base;

  /* Remove the pointer to the SPI object in the interrupt vector */
  if (base == SSI0_BASE)
  {
    SPI0_interruptVector_ = nullptr;
  }
  else if (base == SSI1_BASE)
  {
    SPI1_interruptVector_ = nullptr;
  }
}

void InterruptHandler::setInterruptHandler(Radio& radio_)
{
  /* Store a pointer to the RADIO object in the interrupt vector */
  Radio_interruptVector_ = &radio_;
}

void InterruptHandler::clearInterruptHandler(Radio& radio_)
{
  /* Remove the pointer to the RADIO object in the interrupt vector */
  Radio_interruptVector_ = nullptr;
}

void InterruptHandler::setInterruptHandler(SleepTimer& sleepTimer_)
{
  SleepTimer_interruptVector_ = &sleepTimer_;
}

void InterruptHandler::clearInterruptHandler(SleepTimer& sleepTimer_)
{
  SleepTimer_interruptVector_ = nullptr;
}

void InterruptHandler::setInterruptHandler(RadioTimer& radioTimer_)
{
  RadioTimer_interruptVector_ = &radioTimer_;
}

void InterruptHandler::clearInterruptHandler(RadioTimer& radioimer_)
{
  RadioTimer_interruptVector_ = nullptr;
}

void InterruptHandler::setInterruptHandler(Aes& aes_)
{
  Aes_interruptVector_ = &aes_;
  IntPrioritySet(INT_AES, 0xF0);
}

void InterruptHandler::clearInterruptHandler(Aes& aes_)
{
  Aes_interruptVector_ = nullptr;
}

void InterruptHandler::setInterruptHandler(GpioAdc& adc_)
{
  Adc_interruptVector_ = &adc_;
}

void InterruptHandler::clearInterruptHandler(GpioAdc& adc_)
{
  Adc_interruptVector_ = nullptr;
}

void InterruptHandler::setInterruptHandler(Dma& dma_)
{
  Dma_interruptVector_ = &dma_;
}

void InterruptHandler::clearInterruptHandler(Dma& dma_)
{
  Dma_interruptVector_ = nullptr;
}

/*=============================== protected =================================*/

/*================================ private ==================================*/

InterruptHandler::InterruptHandler()
{
  /* Disable UART interrupts, as it causes trouble during start-up */
  UARTIntDisable(UART0_BASE, 0xFFFF);
  UARTIntDisable(UART1_BASE, 0xFFFF);
  
  /* Registher the SysTick interrupt handler */
  // SysTickIntRegister(SysTick_InterruptHandler);

  /* Register the GPIO interrupt handlers */
  GPIOPortIntRegister(GPIO_A_BASE, GPIOA_InterruptHandler);
  GPIOPortIntRegister(GPIO_B_BASE, GPIOB_InterruptHandler);
  GPIOPortIntRegister(GPIO_C_BASE, GPIOC_InterruptHandler);
  GPIOPortIntRegister(GPIO_D_BASE, GPIOD_InterruptHandler);

  /* Register the TIMERx interrupt handlers */
  TimerIntRegister(GPTIMER0_BASE, GPTIMER_A, TIMER0_InterruptHandler);
  TimerIntRegister(GPTIMER0_BASE, GPTIMER_B, TIMER0_InterruptHandler);
  TimerIntRegister(GPTIMER0_BASE, GPTIMER_BOTH, TIMER0_InterruptHandler);
  TimerIntRegister(GPTIMER1_BASE, GPTIMER_A, TIMER1_InterruptHandler);
  TimerIntRegister(GPTIMER1_BASE, GPTIMER_B, TIMER1_InterruptHandler);
  TimerIntRegister(GPTIMER1_BASE, GPTIMER_BOTH, TIMER1_InterruptHandler);
  TimerIntRegister(GPTIMER2_BASE, GPTIMER_A, TIMER2_InterruptHandler);
  TimerIntRegister(GPTIMER2_BASE, GPTIMER_B, TIMER2_InterruptHandler);
  TimerIntRegister(GPTIMER2_BASE, GPTIMER_BOTH, TIMER2_InterruptHandler);
  TimerIntRegister(GPTIMER3_BASE, GPTIMER_A, TIMER3_InterruptHandler);
  TimerIntRegister(GPTIMER3_BASE, GPTIMER_B, TIMER3_InterruptHandler);
  TimerIntRegister(GPTIMER3_BASE, GPTIMER_BOTH, TIMER3_InterruptHandler);

  /* Register the UARTx interrupt handlers */
  UARTIntRegister(UART0_BASE, UART0_InterruptHandler);
  UARTIntRegister(UART1_BASE, UART1_InterruptHandler);

  /* Register the I2C interrupt handler */
  I2CIntRegister(I2C_InterruptHandler);

  /* Register the SPIx interrupt handler */
  SSIIntRegister(SSI0_BASE, SPI0_InterruptHandler);
  SSIIntRegister(SSI1_BASE, SPI1_InterruptHandler);

  /* Register the RF CORE and ERROR interrupt handlers */
  IntRegister(INT_RFCORERTX, RFCore_InterruptHandler);
  IntRegister(INT_RFCOREERR, RFError_InterruptHandler);

  /* Register the SleepTimer interrupt handler */
  // SleepModeIntRegister(SleepTimer_InterruptHandler);

  /* Register the RadioTimer interrupt handler */
  IntRegister(INT_MACTIMR, RadioTimer_InterruptHandler);

  /* Register the AES interrupt handler */
  IntRegister(INT_AES, Aes_InterruptHandler);

  /* Register the ADC interrupt handler */
  IntRegister(INT_ADC0, Adc_InterruptHandler);
  
  /* Register the DMA interrupt handler */
  uDMAIntRegister(UDMA_INT_SW, Dma_InterruptHandler);
  uDMAIntRegister(UDMA_INT_ERR, Dma_InterruptHandler);
}

void InterruptHandler::GPIO_InterruptHandler(uint32_t base, GpioIn* vector[8])
{
  uint32_t status;

  /* Read the GPIO interrupt status (both regular and power) */
  uint32_t pin_status = GPIOPinIntStatus(base, true);
  uint32_t pow_status = GPIOPowIntStatus(base, true);

  /* Clear the regular GPIO interrupt status */
  GPIOPinIntClear(base, pin_status);
  GPIOPowIntClear(base, pow_status);

  /* Status is both regular and power GPIO interrupt status */
  status = pin_status | pow_status;

  /* Call all the GPIO interrupt handlers */
  if ((status & GPIO_PIN_0) && (vector[0] != nullptr))
  {
    vector[0]->interruptHandler();
  }
  if ((status & GPIO_PIN_1) && (vector[1] != nullptr))
  {
    vector[1]->interruptHandler();
  }
  if ((status & GPIO_PIN_2) && (vector[2] != nullptr))
  {
    vector[2]->interruptHandler();
  }
  if ((status & GPIO_PIN_3) && (vector[3] != nullptr))
  {
    vector[3]->interruptHandler();
  }
  if ((status & GPIO_PIN_4) && (vector[4] != nullptr))
  {
    vector[4]->interruptHandler();
  }
  if ((status & GPIO_PIN_5) && (vector[5] != nullptr))
  {
    vector[5]->interruptHandler();
  }
  if ((status & GPIO_PIN_6) && (vector[6] != nullptr))
  {
    vector[6]->interruptHandler();
  }
  if ((status & GPIO_PIN_7) && (vector[7] != nullptr))
  {
    vector[7]->interruptHandler();
  }
}

void InterruptHandler::GPIOA_InterruptHandler(void)
{
  GPIO_InterruptHandler(GPIO_A_BASE, GPIOA_interruptVector_);
}

void InterruptHandler::GPIOB_InterruptHandler(void)
{
  GPIO_InterruptHandler(GPIO_B_BASE, GPIOB_interruptVector_);
}

void InterruptHandler::GPIOC_InterruptHandler(void)
{
  GPIO_InterruptHandler(GPIO_C_BASE, GPIOC_interruptVector_);
}

void InterruptHandler::GPIOD_InterruptHandler(void)
{
  GPIO_InterruptHandler(GPIO_D_BASE, GPIOD_interruptVector_);
}

void InterruptHandler::TIMER_InterruptHandler(uint32_t base, Timer* vector[2])
{
  uint32_t status;

  status = TimerIntStatus(GPTIMER0_BASE, true);
  TimerIntClear(GPTIMER0_BASE, status);

  if ((TIMER0_interruptVector_[0] != nullptr) && \
      ((status & GPTIMER_TIMA_MATCH) || \
       (status & GPTIMER_CAPA_EVENT) || \
       (status & GPTIMER_TIMA_TIMEOUT) || \
       (status & GPTIMER_CAPA_MATCH)))
  {
    TIMER0_interruptVector_[0]->interruptHandler();
  }

  if ((TIMER0_interruptVector_[1] != nullptr) && \
      ((status & GPTIMER_TIMB_MATCH) || \
       (status & GPTIMER_CAPB_EVENT) || \
       (status & GPTIMER_TIMB_TIMEOUT) || \
       (status & GPTIMER_CAPB_MATCH)))
  {
    TIMER0_interruptVector_[1]->interruptHandler();
  }
}

void InterruptHandler::TIMER0_InterruptHandler(void)
{
  TIMER_InterruptHandler(GPTIMER0_BASE, TIMER0_interruptVector_);
}

void InterruptHandler::TIMER1_InterruptHandler(void)
{
  TIMER_InterruptHandler(GPTIMER1_BASE, TIMER1_interruptVector_);
}

void InterruptHandler::TIMER2_InterruptHandler(void)
{
  TIMER_InterruptHandler(GPTIMER2_BASE, TIMER2_interruptVector_);
}

void InterruptHandler::TIMER3_InterruptHandler(void)
{
  TIMER_InterruptHandler(GPTIMER3_BASE, TIMER3_interruptVector_);
}

void InterruptHandler::UART0_InterruptHandler(void)
{
  if (UART0_interruptVector_ != nullptr)
  {
    /* Call the UART interrupt handler */
    UART0_interruptVector_->interruptHandler();
  }
}

void InterruptHandler::UART1_InterruptHandler(void)
{
  if (UART1_interruptVector_ != nullptr)
  {
    /* Call the UART interrupt handler */
    UART1_interruptVector_->interruptHandler();
  }
}

void InterruptHandler::I2C_InterruptHandler(void)
{
  if (I2C_interruptVector_ != nullptr)
  {
    /* Call the I2C interrupt handler */
    I2C_interruptVector_->interruptHandler();
  }
}

void InterruptHandler::SPI0_InterruptHandler(void)
{
  if (SPI0_interruptVector_ != nullptr)
  {
    /* Call the SPI interrupt handler */
    SPI0_interruptVector_->interruptHandler();
  }
}

void InterruptHandler::SPI1_InterruptHandler(void)
{
  if (SPI0_interruptVector_ != nullptr)
  {
    /* Call the SPI interrupt handler */
    SPI1_interruptVector_->interruptHandler();
  }
}

void InterruptHandler::SysTick_InterruptHandler(void)
{
  if (SysTick_interruptVector_ != nullptr)
  {
    /* Call the SPI interrupt handler */
    SysTick_interruptVector_->interruptHandler();
  }
}

void InterruptHandler::RFCore_InterruptHandler(void)
{
  if (Radio_interruptVector_ != nullptr)
  {
    /* Call the RF CORE interrupt handler */
    Radio_interruptVector_->interruptHandler();
  }
}

void InterruptHandler::RFError_InterruptHandler(void)
{
  if (Radio_interruptVector_ != nullptr)
  {
    /* Call the RF ERROR interrupt handler */
    Radio_interruptVector_->errorHandler();
  }
}

void InterruptHandler::SleepTimer_InterruptHandler(void)
{
  if (SleepTimer_interruptVector_ != nullptr)
  {
    /* Call the SleepTimer interrupt handler */
    SleepTimer_interruptVector_->interruptHandler();
  }
}

void InterruptHandler::RadioTimer_InterruptHandler(void)
{
  if (RadioTimer_interruptVector_ != nullptr)
  {
    /* Call the RadioTimer interrupt handler */
    RadioTimer_interruptVector_->interruptHandler();
  }
}

void InterruptHandler::Aes_InterruptHandler(void)
{
  if (Aes_interruptVector_ != nullptr)
  {
    /* Call the AES interrupt handler */
    Aes_interruptVector_->interruptHandler();
  }
}

void InterruptHandler::Adc_InterruptHandler(void)
{
  if (Adc_interruptVector_ != nullptr)
  {
    /* Call the ADC interrupt handler */
    Adc_interruptVector_->interruptHandler();
  }
}

void InterruptHandler::Dma_InterruptHandler(void)
{
  if (Dma_interruptVector_ != nullptr)
  {
    /* Call the AES interrupt handler */
    Dma_interruptVector_->interruptHandler();
  }
}

void InterruptHandler::DmaError_InterruptHandler(void)
{
  if (Dma_interruptVector_ != nullptr)
  {
    /* Call the AES interrupt handler */
    Dma_interruptVector_->errorHandler();
  }
}