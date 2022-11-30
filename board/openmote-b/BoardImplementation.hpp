/**
 * @file       BoardImplementation.hpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       November, 2018
 * @brief
 *
 * @copyright  Copyright 2018, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef BOARD_IMPLEMENTATION_HPP_
#define BOARD_IMPLEMENTATION_HPP_

/*================================ include ==================================*/

#include "Board.hpp"

#include "Aes.hpp"
#include "Dma.hpp"
#include "Gpio.hpp"
#include "I2c.hpp"
#include "Radio.hpp"
#include "RadioTimer.hpp"
#include "RandomNumberGenerator.hpp"
#include "SleepTimer.hpp"
#include "Spi.hpp"
#include "Uart.hpp"
#include "Watchdog.hpp"

#include "TemperatureSensor.hpp"

#include "at86rf215/At86rf215.hpp"

/*================================ define ===================================*/

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

/* Board */
extern BoardImplementation board;

/* LEDs */
extern GpioOut led_green;
extern GpioOut led_orange;
extern GpioOut led_red;
extern GpioOut led_yellow;

/* User button */
extern GpioInPow button_user;

/* Debug pins */
extern GpioOut debug0;
extern GpioOut debug1;
extern GpioOut debug2;
extern GpioOut debug3;
extern GpioOut debug4;
extern GpioOut debug5;

/* Timers */
extern Timer timer0a;

/* SleepTimer */
extern SleepTimer sleepTimer;

/* RadioTimer */
extern RadioTimer radioTimer;

/* Watchdog */
extern Watchdog watchdog;

/* I2C */
extern I2c i2c;

/* UART */
extern Uart uart0;
extern Uart uart1;

/* SPI */
extern Spi spi0;
extern Spi spi1;

/* Number Generator */
extern NumberGenerator prng;

/* IEEE 802.15.4 radio */
extern Radio radio;

/* AES module */
extern Aes aes;

/* CC2538 temperature sensor */
extern TemperatureSensor temp;

/* DMA controller */
extern Dma dma;

/* AT86RF215 radio transceiver */
extern At86rf215 at86rf215;

/* Antenna switch */
extern GpioOut antenna_at86rf215;
extern GpioOut antenna_cc2538;

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

/*================================ private ==================================*/

#endif /* BOARD_IMPLEMENTATION_HPP_ */
