/**
 * @file       BoardImplementation.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       November, 2018
 * @brief
 *
 * @copyright  Copyright 2018, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

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
#include "Timer.hpp"
#include "Uart.hpp"
#include "Watchdog.hpp"

#include "TemperatureSensor.hpp"

#include "at86rf215/At86rf215.hpp"

#include "platform_includes.h"
#include "platform_types.hpp"

/*================================ define ===================================*/

#define BOARD_USE_32KHZ_XTAL    ( true )
#define BOARD_USE_32MHZ_OSC     ( false )
#define SYSTEM_CLOCK_DIVIDER    ( SYS_CTRL_SYSDIV_32MHZ )
#define PERIPH_CLOCK_DIVIDER    ( SYS_CTRL_SYSDIV_32MHZ )

#define LED_RED_PORT            ( GPIO_C_BASE )
#define LED_RED_PIN             ( GPIO_PIN_4 )
#define LED_RED_IOC             ( IOC_MUX_OUT_SEL_GPT0_ICP1 )

#define LED_ORANGE_PORT         ( GPIO_C_BASE )
#define LED_ORANGE_PIN          ( GPIO_PIN_5 )
#define LED_ORANGE_IOC          ( IOC_MUX_OUT_SEL_GPT0_ICP1 )

#define LED_YELLOW_PORT         ( GPIO_C_BASE )
#define LED_YELLOW_PIN          ( GPIO_PIN_6 )
#define LED_YELLOW_IOC          ( IOC_MUX_OUT_SEL_GPT0_ICP1 )

#define LED_GREEN_PORT          ( GPIO_C_BASE )
#define LED_GREEN_PIN           ( GPIO_PIN_7 )
#define LED_GREEN_IOC           ( IOC_MUX_OUT_SEL_GPT0_ICP1 )

#define DEBUG0_PORT             ( GPIO_B_BASE )
#define DEBUG0_PIN              ( GPIO_PIN_0 )

#define DEBUG1_PORT             ( GPIO_B_BASE )
#define DEBUG1_PIN              ( GPIO_PIN_1 )

#define DEBUG2_PORT             ( GPIO_B_BASE )
#define DEBUG2_PIN              ( GPIO_PIN_2 )

#define DEBUG3_PORT             ( GPIO_B_BASE )
#define DEBUG3_PIN              ( GPIO_PIN_3 )

#define DEBUG4_PORT             ( GPIO_C_BASE )
#define DEBUG4_PIN              ( GPIO_PIN_3 )

#define DEBUG5_PORT             ( GPIO_A_BASE )
#define DEBUG5_PIN              ( GPIO_PIN_7 )
#define DEBUG5_ADC_RES          ( SOCADC_12_BIT )
#define DEBUG5_ADC_REF          ( SOCADC_REF_INTERNAL )
#define DEBUG5_ADC_CHAN         ( SOCADC_AIN7 )

#define BUTTON_USER_PORT        ( GPIO_D_BASE )
#define BUTTON_USER_PIN         ( GPIO_PIN_5 )
#define BUTTON_USER_EDGE        ( GPIO_FALLING_EDGE )

#define BOOTLOAD_PORT           ( GPIO_A_BASE )
#define BOOTLOAD_PIN            ( GPIO_PIN_6 )

#define ANTENNA_AT86RF215_PORT  ( GPIO_D_BASE )
#define ANTENNA_AT86RF215_PIN   ( GPIO_PIN_3 )
#define ANTENNA_CC2538_PORT     ( GPIO_D_BASE )
#define ANTENNA_CC2538_PIN      ( GPIO_PIN_4 )

#define TIMER0A_PERIPHERAL      ( SYS_CTRL_PERIPH_GPT0 )
#define TIMER0A_BASE            ( GPTIMER0_BASE )
#define TIMER0A_SOURCE          ( GPTIMER_BOTH )
#define TIMER0A_CONFIG          ( GPTIMER_CFG_ONE_SHOT_UP )
#define TIMER0A_INTERRUPT       ( INT_TIMER0A )
#define TIMER0A_INTERRUPT_MODE  ( GPTIMER_TIMA_TIMEOUT )

#define TIMER0B_PERIPHERAL      ( SYS_CTRL_PERIPH_GPT0 )
#define TIMER0B_BASE            ( GPTIMER0_BASE )
#define TIMER0B_SOURCE          ( GPTIMER_B )
#define TIMER0B_CONFIG          ( GPTIMER_CFG_SPLIT_PAIR |  GPTIMER_CFG_A_PWM | GPTIMER_CFG_B_PERIODIC )
#define TIMER0B_INTERRUPT       ( INT_TIMER0B )
#define TIMER0B_INTERRUPT_MODE  ( GPTIMER_TIMB_TIMEOUT )

#define TIMER1A_PERIPHERAL      ( SYS_CTRL_PERIPH_GPT1 )
#define TIMER1A_BASE            ( GPTIMER1_BASE )
#define TIMER1A_SOURCE          ( GPTIMER_A )
#define TIMER1A_CONFIG          ( GPTIMER_CFG_SPLIT_PAIR |  GPTIMER_CFG_A_PERIODIC | GPTIMER_CFG_B_PERIODIC )
#define TIMER1A_INTERRUPT       ( INT_TIMER1A )
#define TIMER1A_INTERRUPT_MODE  ( GPTIMER_TIMA_TIMEOUT )

#define TIMER1B_PERIPHERAL      ( SYS_CTRL_PERIPH_GPT1 )
#define TIMER1B_BASE            ( GPTIMER1_BASE )
#define TIMER1B_SOURCE          ( GPTIMER_B )
#define TIMER1B_CONFIG          ( GPTIMER_CFG_SPLIT_PAIR |  GPTIMER_CFG_A_PERIODIC | GPTIMER_CFG_B_PERIODIC )
#define TIMER1B_INTERRUPT       ( INT_TIMER1B )
#define TIMER1B_INTERRUPT_MODE  ( GPTIMER_TIMB_TIMEOUT )

#define TIMER2A_PERIPHERAL      ( SYS_CTRL_PERIPH_GPT2 )
#define TIMER2A_BASE            ( GPTIMER2_BASE )
#define TIMER2A_SOURCE          ( GPTIMER_A )
#define TIMER2A_CONFIG          ( GPTIMER_CFG_SPLIT_PAIR |  GPTIMER_CFG_A_PERIODIC | GPTIMER_CFG_B_PERIODIC )
#define TIMER2A_INTERRUPT       ( INT_TIMER2A )
#define TIMER2A_INTERRUPT_MODE  ( GPTIMER_TIMA_TIMEOUT )

#define TIMER2B_PERIPHERAL      ( SYS_CTRL_PERIPH_GPT2 )
#define TIMER2B_BASE            ( GPTIMER2_BASE )
#define TIMER2B_SOURCE          ( GPTIMER_B )
#define TIMER2B_CONFIG          ( GPTIMER_CFG_SPLIT_PAIR |  GPTIMER_CFG_A_PERIODIC | GPTIMER_CFG_B_PERIODIC )
#define TIMER2B_INTERRUPT       ( INT_TIMER2B )
#define TIMER2B_INTERRUPT_MODE  ( GPTIMER_TIMB_TIMEOUT )

#define TIMER3A_PERIPHERAL      ( SYS_CTRL_PERIPH_GPT3 )
#define TIMER3A_BASE            ( GPTIMER3_BASE )
#define TIMER3A_SOURCE          ( GPTIMER_A )
#define TIMER3A_CONFIG          ( GPTIMER_CFG_SPLIT_PAIR |  GPTIMER_CFG_A_PERIODIC | GPTIMER_CFG_B_PERIODIC )
#define TIMER3A_INTERRUPT       ( INT_TIMER3A )
#define TIMER3A_INTERRUPT_MODE  ( GPTIMER_TIMA_TIMEOUT )

#define TIMER3B_PERIPHERAL      ( SYS_CTRL_PERIPH_GPT3 )
#define TIMER3B_BASE            ( GPTIMER3_BASE )
#define TIMER3B_SOURCE          ( GPTIMER_B )
#define TIMER3B_CONFIG          ( GPTIMER_CFG_SPLIT_PAIR |  GPTIMER_CFG_A_PERIODIC | GPTIMER_CFG_B_PERIODIC )
#define TIMER3B_INTERRUPT       ( INT_TIMER3B )
#define TIMER3B_INTERRUPT_MODE  ( GPTIMER_TIMB_TIMEOUT )

#define SLEEP_TIMER_INTERRUPT   ( INT_SMTIM )

#define RADIO_TIMER_INTERRUPT   ( INT_MACTIMR )

#define UART0_PERIPHERAL        ( SYS_CTRL_PERIPH_UART0 )
#define UART0_BASE_ADDR         ( UART0_BASE )
#define UART0_CLOCK             ( UART_CLOCK_PIOSC )
#define UART0_INT               ( INT_UART0 )
#define UART0_BAUDRATE          ( 115200 )
#define UART0_MODE              ( UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE )

#define UART0_RX_PORT           ( GPIO_A_BASE )
#define UART0_RX_PIN            ( GPIO_PIN_0 )
#define UART0_RX_IOC            ( IOC_UARTRXD_UART0 )
#define UART0_TX_PORT           ( GPIO_A_BASE )
#define UART0_TX_PIN            ( GPIO_PIN_1 )
#define UART0_TX_IOC            ( IOC_MUX_OUT_SEL_UART0_TXD )

#define UART1_PERIPHERAL        ( SYS_CTRL_PERIPH_UART1 )
#define UART1_BASE_ADDR         ( UART1_BASE )
#define UART1_CLOCK             ( UART_CLOCK_PIOSC )
#define UART1_INT               ( INT_UART1 )
#define UART1_BAUDRATE          ( 115200 )
#define UART1_MODE              ( UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE )

#define UART1_RX_PORT           ( GPIO_B_BASE )
#define UART1_RX_PIN            ( GPIO_PIN_2 )
#define UART1_RX_IOC            ( IOC_UARTRXD_UART1 )
#define UART1_TX_PORT           ( GPIO_B_BASE )
#define UART1_TX_PIN            ( GPIO_PIN_3 )
#define UART1_TX_IOC            ( IOC_MUX_OUT_SEL_UART1_TXD )

#define SPI0_PERIPHERAL         ( SYS_CTRL_PERIPH_SSI0 )
#define SPI0_BASE_ADDR          ( SSI0_BASE )
#define SPI0_CLOCK              ( SSI_CLOCK_PIOSC )
#define SPI0_INT                ( INT_SSI0 )
#define SPI0_MODE               ( SSI_MODE_MASTER )
#define SPI0_PROTOCOL           ( SSI_FRF_MOTO_MODE_0 )
#define SPI0_DATAWIDTH          ( 8 )
#define SPI0_BAUDRATE           ( 16000000 )

#define SPI0_MISO_BASE          ( GPIO_A_BASE )
#define SPI0_MISO_PIN           ( GPIO_PIN_4 )
#define SPI0_MISO_IOC           ( IOC_SSIRXD_SSI0 )
#define SPI0_MOSI_BASE          ( GPIO_A_BASE )
#define SPI0_MOSI_PIN           ( GPIO_PIN_5 )
#define SPI0_MOSI_IOC           ( IOC_MUX_OUT_SEL_SSI0_TXD )
#define SPI0_CLK_BASE           ( GPIO_A_BASE )
#define SPI0_CLK_PIN            ( GPIO_PIN_2 )
#define SPI0_CLK_IOC            ( IOC_MUX_OUT_SEL_SSI0_CLKOUT )

#define SPI1_PERIPHERAL         ( SYS_CTRL_PERIPH_SSI1 )
#define SPI1_BASE_ADDR          ( SSI1_BASE )
#define SPI1_CLOCK              ( SSI_CLOCK_PIOSC )
#define SPI1_INT                ( INT_SSI1 )
#define SPI1_MODE               ( SSI_MODE_MASTER )
#define SPI1_PROTOCOL           ( SSI_FRF_MOTO_MODE_3 )
#define SPI1_DATAWIDTH          ( 8 )
#define SPI1_BAUDRATE           ( 16000000 )

#define SPI1_MISO_BASE          ( GPIO_B_BASE )
#define SPI1_MISO_PIN           ( GPIO_PIN_1 )
#define SPI1_MISO_IOC           ( IOC_SSIRXD_SSI1 )
#define SPI1_MOSI_BASE          ( GPIO_B_BASE )
#define SPI1_MOSI_PIN           ( GPIO_PIN_0 )
#define SPI1_MOSI_IOC           ( IOC_MUX_OUT_SEL_SSI1_TXD )
#define SPI1_CLK_BASE           ( GPIO_B_BASE )
#define SPI1_CLK_PIN            ( GPIO_PIN_2 )
#define SPI1_CLK_IOC            ( IOC_MUX_OUT_SEL_SSI1_CLKOUT )

#define I2C_PERIPHERAL          ( SYS_CTRL_PERIPH_I2C )
#define I2C_BAUDRATE            ( 400000 )
#define I2C_SCL_BASE            ( GPIO_B_BASE )
#define I2C_SCL_PIN             ( GPIO_PIN_1 )
#define I2C_SDA_BASE            ( GPIO_B_BASE )
#define I2C_SDA_PIN             ( GPIO_PIN_0 )

#define WATCHDOG_INTERVAL       ( WATCHDOG_INTERVAL_32768 )

#define AT86RF215_PWR_BASE      ( GPIO_C_BASE )
#define AT86RF215_PWR_PIN       ( GPIO_PIN_0 )

#define AT86RF215_RST_BASE      ( GPIO_D_BASE )
#define AT86RF215_RST_PIN       ( GPIO_PIN_1 )

#define AT86RF215_IRQ_BASE      ( GPIO_D_BASE )
#define AT86RF215_IRQ_PIN       ( GPIO_PIN_0 )
#define AT86RF215_IRQ_EDGE      ( GPIO_RISING_EDGE )

#define AT86RF215_CSn_BASE      ( GPIO_A_BASE )
#define AT86RF215_CSn_PIN       ( GPIO_PIN_3 )

/*================================ typedef ==================================*/

/*================================ private ==================================*/

/* Board management */
BoardParams board_params {BOARD_USE_32KHZ_XTAL, BOARD_USE_32MHZ_OSC, SYSTEM_CLOCK_DIVIDER, PERIPH_CLOCK_DIVIDER};

/* LEDs */
static const GpioConfig led_green_cfg  {LED_GREEN_PORT,  LED_GREEN_PIN,  0, 0, 1};
static const GpioConfig led_orange_cfg {LED_ORANGE_PORT, LED_ORANGE_PIN, 0, 0, 1};
static const GpioConfig led_red_cfg    {LED_RED_PORT,    LED_RED_PIN,    0, 0, 1};
static const GpioConfig led_yellow_cfg {LED_YELLOW_PORT, LED_YELLOW_PIN, 0, 0, 1};

/* Debug pins */ 
static const GpioConfig debug0_cfg {DEBUG0_PORT, DEBUG0_PIN, 0, 0, 0};
static const GpioConfig debug1_cfg {DEBUG1_PORT, DEBUG1_PIN, 0, 0, 0};
static const GpioConfig debug2_cfg {DEBUG2_PORT, DEBUG2_PIN, 0, 0, 0};
static const GpioConfig debug3_cfg {DEBUG3_PORT, DEBUG3_PIN, 0, 0, 0};
static const GpioConfig debug4_cfg {DEBUG4_PORT, DEBUG4_PIN, 0, 0, 0};
static const GpioConfig debug5_cfg {DEBUG4_PORT, DEBUG5_PIN, 0, 0, 0};

/* Buttons */
static const GpioConfig button_user_cfg {BUTTON_USER_PORT, BUTTON_USER_PIN, 0, BUTTON_USER_EDGE, 0};

/* Adc */ 
// GpioConfig gpio_adc_cfg = {DEBUG5_PORT, DEBUG5_PIN, 0, 0, 0};
// AdcConfig adc_cfg = {DEBUG5_ADC_RES, DEBUG5_ADC_REF, DEBUG5_ADC_CHAN};
// GpioAdc gpio_adc(gpio_adc_cfg, adc_cfg);

/* Antenna pins */
static const GpioConfig antenna_at86rf215_cfg {ANTENNA_AT86RF215_PORT, ANTENNA_AT86RF215_PIN, 0, 0, 0};
static const GpioConfig antenna_cc2538_cfg    {ANTENNA_CC2538_PORT,    ANTENNA_CC2538_PIN,    0, 0, 0};

/* Timers */
static const TimerConfig timer0a_cfg {TIMER0A_PERIPHERAL, TIMER0A_BASE, TIMER0A_SOURCE, TIMER0A_CONFIG, TIMER0A_INTERRUPT, TIMER0A_INTERRUPT_MODE};
static const TimerConfig timer0b_cfg {TIMER0B_PERIPHERAL, TIMER0B_BASE, TIMER0B_SOURCE, TIMER0B_CONFIG, TIMER0B_INTERRUPT, TIMER0B_INTERRUPT_MODE};
static const TimerConfig timer1a_cfg {TIMER1A_PERIPHERAL, TIMER1A_BASE, TIMER1A_SOURCE, TIMER1A_CONFIG, TIMER1A_INTERRUPT, TIMER1A_INTERRUPT_MODE};
static const TimerConfig timer1b_cfg {TIMER1B_PERIPHERAL, TIMER1B_BASE, TIMER1B_SOURCE, TIMER1B_CONFIG, TIMER1B_INTERRUPT, TIMER1B_INTERRUPT_MODE};
static const TimerConfig timer2a_cfg {TIMER2A_PERIPHERAL, TIMER2A_BASE, TIMER2A_SOURCE, TIMER2A_CONFIG, TIMER2A_INTERRUPT, TIMER2A_INTERRUPT_MODE};
static const TimerConfig timer2b_cfg {TIMER2B_PERIPHERAL, TIMER2B_BASE, TIMER2B_SOURCE, TIMER2B_CONFIG, TIMER2B_INTERRUPT, TIMER2B_INTERRUPT_MODE};
static const TimerConfig timer3a_cfg {TIMER3A_PERIPHERAL, TIMER3A_BASE, TIMER3A_SOURCE, TIMER3A_CONFIG, TIMER3A_INTERRUPT, TIMER3A_INTERRUPT_MODE};
static const TimerConfig timer3b_cfg {TIMER3B_PERIPHERAL, TIMER3B_BASE, TIMER3B_SOURCE, TIMER3B_CONFIG, TIMER3B_INTERRUPT, TIMER3B_INTERRUPT_MODE};

/* I2C peripheral */
static const GpioConfig i2c_scl_cfg {I2C_SCL_BASE, I2C_SCL_PIN, 0, 0, 0};
static const GpioConfig i2c_sda_cfg {I2C_SDA_BASE, I2C_SDA_PIN, 0, 0, 0};
static I2cConfig i2c_cfg            {I2C_PERIPHERAL, I2C_BAUDRATE};

/* SPI peripheral */
static const GpioConfig spi0_miso_cfg {SPI0_MISO_BASE, SPI0_MISO_PIN, SPI0_MISO_IOC, 0, 0};
static const GpioConfig spi0_mosi_cfg {SPI0_MOSI_BASE, SPI0_MOSI_PIN, SPI0_MOSI_IOC, 0, 0};
static const GpioConfig spi0_clk_cfg  {SPI0_CLK_BASE,  SPI0_CLK_PIN,  SPI0_CLK_IOC,  0, 0};
static SpiConfig spi0_cfg             {SPI0_PERIPHERAL, SPI0_BASE_ADDR, SPI0_CLOCK, SPI0_INT, SPI0_MODE, SPI0_PROTOCOL, SPI0_DATAWIDTH, SPI0_BAUDRATE};

static const GpioConfig spi1_miso_cfg {SPI1_MISO_BASE, SPI1_MISO_PIN, SPI1_MISO_IOC, 0, 0};
static const GpioConfig spi1_mosi_cfg {SPI1_MOSI_BASE, SPI1_MOSI_PIN, SPI1_MOSI_IOC, 0, 0};
static const GpioConfig spi1_clk_cfg  {SPI1_CLK_BASE,  SPI1_CLK_PIN,  SPI1_CLK_IOC,  0, 0};
static SpiConfig spi1_cfg             {SPI1_PERIPHERAL, SPI1_BASE_ADDR, SPI1_CLOCK, SPI1_INT, SPI1_MODE, SPI1_PROTOCOL, SPI1_DATAWIDTH, SPI1_BAUDRATE};

/* UART peripheral */
static const GpioConfig uart0_rx_cfg {UART0_RX_PORT, UART0_RX_PIN, UART0_RX_IOC, 0, 0};
static const GpioConfig uart0_tx_cfg {UART0_TX_PORT, UART0_TX_PIN, UART0_TX_IOC, 0, 0};
static UartConfig uart0_cfg          {UART0_PERIPHERAL, UART0_BASE_ADDR, UART0_CLOCK, UART0_INT, UART0_BAUDRATE, UART0_MODE};

static const GpioConfig uart1_rx_cfg {UART1_RX_PORT, UART1_RX_PIN, UART1_RX_IOC, 0, 0};
static const GpioConfig uart1_tx_cfg {UART1_TX_PORT, UART1_TX_PIN, UART1_TX_IOC, 0, 0};
static UartConfig uart1_cfg          {UART1_PERIPHERAL, UART1_BASE_ADDR, UART1_CLOCK, UART1_INT, UART1_BAUDRATE, UART1_MODE};

static Gpio i2c_scl {i2c_scl_cfg};
static Gpio i2c_sda {i2c_sda_cfg};

static Gpio spi0_miso {spi0_miso_cfg};
static Gpio spi0_mosi {spi0_mosi_cfg};
static Gpio spi0_clk  {spi0_clk_cfg};

static Gpio spi1_miso {spi1_miso_cfg};
static Gpio spi1_mosi {spi1_mosi_cfg};
static Gpio spi1_clk  {spi1_clk_cfg};

static Gpio uart0_rx {uart0_rx_cfg};
static Gpio uart0_tx {uart0_tx_cfg};

static Gpio uart1_rx {uart1_rx_cfg};
static Gpio uart1_tx {uart1_tx_cfg};

static const GpioConfig at86rf215_pwr_cfg {AT86RF215_PWR_BASE, AT86RF215_PWR_PIN, 0, 0, 0};
static const GpioConfig at86rf215_rst_cfg {AT86RF215_RST_BASE, AT86RF215_RST_PIN, 0, 0, 1};
static const GpioConfig at86rf215_csn_cfg {AT86RF215_CSn_BASE, AT86RF215_CSn_PIN, 0, 0, 1};
static const GpioConfig at86rf215_irq_cfg {AT86RF215_IRQ_BASE, AT86RF215_IRQ_PIN, 0, AT86RF215_IRQ_EDGE, 0};

static GpioOut at86rf215_pwr {at86rf215_pwr_cfg};
static GpioOut at86rf215_rst {at86rf215_rst_cfg};
static GpioOut at86rf215_csn {at86rf215_csn_cfg};
static GpioIn  at86rf215_irq {at86rf215_irq_cfg};

/*=============================== variables =================================*/

/* Board */
BoardImplementation board {board_params};

/* LEDs */
GpioOut led_green {led_green_cfg};
GpioOut led_orange {led_orange_cfg};
GpioOut led_red {led_red_cfg};
GpioOut led_yellow {led_yellow_cfg};

/* User button */
GpioInPow button_user {button_user_cfg};

/* Debug pins */
// GpioOut debug0 {debug0_cfg};
// GpioOut debug1 {debug1_cfg};
// GpioOut debug2 {debug2_cfg};
// GpioOut debug3 {debug3_cfg};
// GpioOut debug4 {debug4_cfg};
// GpioOut debug5 {debug5_cfg};

// Timer timer0a{timer0a_cfg};
// Timer timer0b(timer0b_cfg);
// Timer timer1a(timer1a_cfg);
// Timer timer1b(timer1b_cfg);
// Timer timer2a(timer2a_cfg);
// Timer timer2b(timer2b_cfg);
// Timer timer3a(timer3a_cfg);
// Timer timer3b(timer3b_cfg);

/* SleepTimer */
SleepTimer sleepTimer {SLEEP_TIMER_INTERRUPT};

/* RadioTimer */
RadioTimer radioTimer {RADIO_TIMER_INTERRUPT};

/* Watchdog */
Watchdog watchdog {WATCHDOG_INTERVAL};

/* I2C */
I2c i2c {i2c_scl, i2c_sda, i2c_cfg};

/* UART */
Uart uart0 {uart0_rx, uart0_tx, uart0_cfg};
Uart uart1 {uart1_rx, uart1_tx, uart1_cfg};

/* SPI */
Spi spi0 {spi0_miso, spi0_mosi, spi0_clk, spi0_cfg};
// Spi spi1 {spi1_miso, spi1_mosi, spi1_clk, spi1_cfg};

/* Pseudo-Random Number Generator */
DeterministicNumberGenerator prng;

/* IEEE 802.15.4 radio */
Radio radio;

/* AES module */
Aes aes;

/* CC2538 temperature sensor */
TemperatureSensor temp;

/* DMA controller */
Dma dma;

/* AT86RF215 radio transceiver */
At86rf215 at86rf215(spi0, at86rf215_pwr, at86rf215_rst, at86rf215_csn, at86rf215_irq);

/* Antenna switch */
GpioOut antenna_at86rf215 {antenna_at86rf215_cfg};
GpioOut antenna_cc2538    {antenna_cc2538_cfg};

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

/* Each BoardImplementation needs to provide its own constructor */
BoardImplementation::BoardImplementation(const BoardParams& boardParams):
  boardParams_(boardParams)
{
}

/* Each BoardImplementation needs to provide its own init function */
void BoardImplementation::init(void)
{
  /* Initialize basic board parameters */
  Board::init(boardParams_); 

  /* Make sure LEDs are off */
  led_green.off();
  led_yellow.off();
  led_orange.off();
  led_red.off();
}

/*================================ private ==================================*/
