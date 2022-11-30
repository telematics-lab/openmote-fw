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

#include "Gpio.hpp"
#include "Crc16.hpp"

#include "Callback.hpp"
#include "Scheduler.hpp"
#include "Semaphore.hpp"
#include "Task.hpp"

/*================================ define ===================================*/

#define GREEN_LED_TASK_PRIORITY             ( tskIDLE_PRIORITY + 1 )
#define CRC_TASK_PRIORITY                   ( tskIDLE_PRIORITY + 0 )

/*================================ typedef ==================================*/

/*=============================== prototypes ================================*/

static void prvGreenLedTask(void *pvParameters);
static void prvCrcTask(void *pvParameters);

/*=============================== variables =================================*/

static Task heartbeatTask{(const char *) "Green", 128, GREEN_LED_TASK_PRIORITY, prvGreenLedTask, nullptr};
static Task crcTask{(const char *) "Crc", 128, CRC_TASK_PRIORITY, prvCrcTask, nullptr};

static Crc16 crc16;

/*================================= public ==================================*/

int main (void)
{
  /* Initialize board */
  board.init();

  /* Start the scheduler */
  Scheduler::run();
}

/*=============================== protected =================================*/

/*================================ private ==================================*/

static void prvCrcTask(void *pvParameters)
{
    uint8_t crc_test[9] = {0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39};
    uint16_t crc_output, crc_result = 0x29B1;
    uint8_t * ptr = crc_test;
    uint8_t size = sizeof(crc_test);

    /* Forever */
    while(true)
    {
        ptr = crc_test;
        size = sizeof(crc_test);

        crc16.init();
        while(size--) {
            crc16.set(*ptr++);
        }
        crc_output = crc16.get();

        if (crc_output == crc_result)
        {
            led_yellow.on();
        }
        else {
            led_red.on();
        }

        Scheduler::delay_ms(250);

        led_red.off();
        led_yellow.off();

        Scheduler::delay_ms(250);
    }
}

static void prvGreenLedTask(void *pvParameters)
{
  /* Forever */
  while (true)
  {
    /* Turn off green LED for 950 ms */
    led_green.off();
    Scheduler::delay_ms(900);

    /* Turn on green LED for 50 ms */
    led_green.on();
    Scheduler::delay_ms(100);
  }
}
