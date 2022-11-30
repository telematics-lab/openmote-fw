/**
 * @file       main.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       February, 2019
 * @brief
 *
 * @copyright  Copyright 2019, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include "BoardImplementation.hpp"

#include "Buffer.hpp"
#include "Callback.hpp"
#include "Scheduler.hpp"
#include "Semaphore.hpp"
#include "Task.hpp"

/*================================ define ===================================*/

#define GREEN_LED_TASK_PRIORITY             ( tskIDLE_PRIORITY + 0 )
#define UART_TASK_PRIORITY                  ( tskIDLE_PRIORITY + 1 )

#define UART_BAUDRATE                       ( 2304000 )
#define UART_BUFFER_LENGTH                  ( 2048 )

/*================================ typedef ==================================*/

/*=============================== prototypes ================================*/

static void prvGreenLedTask(void *pvParameters);
static void prvUartTask(void *pvParameters);

static void uart_rx(void);
static void uart_tx(void);

/*=============================== variables =================================*/

static Task heartbeatTask{(const char *) "Green", 128, GREEN_LED_TASK_PRIORITY, prvGreenLedTask, nullptr};
static Task uartTask{(const char *) "Uart", 128, UART_TASK_PRIORITY, prvUartTask, nullptr};

static PlainCallback uart_rx_cb(&uart_rx);
static PlainCallback uart_tx_cb(&uart_tx);

static uint8_t uart_tx_buffer[UART_BUFFER_LENGTH];
static uint8_t uart_rx_buffer[UART_BUFFER_LENGTH];

static Buffer rx_buffer {uart_rx_buffer, sizeof(uart_tx_buffer)};
static Buffer tx_buffer {uart_tx_buffer, sizeof(uart_rx_buffer)};

static bool rx_error = false;
static bool copy_buffer = true;

/*================================= public ==================================*/

int main (void)
{
  /* Initialize the board */
  board.init();

  /* Enable the UART peripheral */
  uart0.enable(UART_BAUDRATE);
  uart0.setRxCallback(&uart_rx_cb);
  uart0.setTxCallback(&uart_tx_cb);
  uart0.enableDMA();
  uart0.enableInterrupts();

  /* Start the scheduler */
  Scheduler::run();
}

/*=============================== protected =================================*/

/*================================ private ==================================*/

static void prvUartTask(void *pvParameters)
{
    
  /* Forever */
  while (true)
  {
    /* Restore transmit and receive buffers */
    rx_buffer.reset();
    tx_buffer.reset();

    /* Wait until a packet has been received from UART */
    uart0.rxLock();

    /* Copy UART receive buffer */
    if (copy_buffer)
    {
      uint32_t length;

      /* Copy UART receive buffer to UART transmit buffer using DMA */
      length = dma.memcpy(tx_buffer.getHead(), rx_buffer.getHead(), rx_buffer.getSize());

      /* Send buffer to UART using DMA */
      uart0.writeByte(tx_buffer.getHead(), length);
    }
    else
    {
      /* Send buffer to UART using DMA */
      uart0.writeByte(rx_buffer.getHead(), rx_buffer.getSize());
    }
    
    /* Wait until packet has been transmitted from UART */
    uart0.txLock();
  }
}

static void prvGreenLedTask(void *pvParameters)
{
  /* Forever */
  while (true)
  {
    /* Turn off green LED for 990 ms */
    led_green.off();
    Scheduler::delay_ms(990);

    /* Turn on green LED for 10 ms */
    led_green.on();
    Scheduler::delay_ms(10);
  }
}

static void uart_rx(void)
{
  bool finished, status;

  /* Read bytes from UART */
  status = uart0.readBytes(rx_buffer, finished);
  
  /* Check for errors */
  if (!status) goto error;
    
  /* If the UART has finished */
  if (finished)
  {
    /* Give the RX semaphore to signal end of reception */
    uart0.rxUnlockFromInterrupt();
  }

  return;
  
error:
  rx_error = true;
  
  /* Give the RX semaphore to signal end of reception */
  uart0.rxUnlockFromInterrupt();
}

static void uart_tx(void)
{
  /* Give the TX semaphore to signal end of transmission */
  uart0.txUnlockFromInterrupt();
}