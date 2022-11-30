/**
 * @file       main.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include "BoardImplementation.hpp"

#include "Callback.hpp"
#include "Scheduler.hpp"
#include "Semaphore.hpp"
#include "Task.hpp"

#include "Serial.hpp"

/*================================ define ===================================*/

#define GREEN_LED_TASK_PRIORITY             ( tskIDLE_PRIORITY + 0 )
#define SERIAL_TASK_PRIORITY                ( tskIDLE_PRIORITY + 1 )

#define UART_BAUDRATE                       ( 1152000 )

#define BUFFER_LENGTH                       ( 2048 )

/*================================ typedef ==================================*/

/*=============================== prototypes ================================*/

static void prvGreenLedTask(void *pvParameters);
static void prvSerialTask(void *pvParameters);

/*=============================== variables =================================*/

static Task heartbeatTask{(const char *) "Green", 128, GREEN_LED_TASK_PRIORITY, prvGreenLedTask, nullptr};
static Task serialTask{(const char *) "Serial", 128, SERIAL_TASK_PRIORITY, prvSerialTask, nullptr};

static Serial serial(uart0);

static uint8_t serial_buffer[BUFFER_LENGTH];
static uint8_t* serial_ptr = serial_buffer;
static uint16_t serial_len  = sizeof(serial_buffer);

/*================================= public ==================================*/

int main (void)
{
  /* Initialize the board */
  board.init();

  /* Enable the UART interface */
  uart0.enable(UART_BAUDRATE);
  
  /* Initialize Serial interface */
  serial.init();

  /* Start the scheduler */
  Scheduler::run();
}

/*=============================== protected =================================*/

/*================================ private ==================================*/

static void prvSerialTask(void *pvParameters)
{
  /* Forever */
  while (true)
  {
    /* Restore serial_rx pointer and length */
    serial_ptr = serial_buffer;
    serial_len = sizeof(serial_buffer);

    /* Read buffer using Serial */
    serial_len = serial.read(serial_ptr, serial_len);
    
    /* If we have received a message */
    if (serial_len > 0)
    {
      /* Write buffer via Serial */
      serial.write(serial_ptr, serial_len, true);
    }
  }
}

static void prvGreenLedTask(void *pvParameters)
{
  /* Forever */
  while (true)
  {
    /* Turn off green LED for 950 ms */
    led_green.off();
    Scheduler::delay_ms(950);

    /* Turn on green LED for 50 ms */
    led_green.on();
    Scheduler::delay_ms(50);
  }
}
