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

#include "Spi.hpp"

#include "Callback.hpp"
#include "Scheduler.hpp"
#include "Semaphore.hpp"
#include "Task.hpp"

#include "Flir.hpp"
#include "Flir_regs.h"

/*================================ define ===================================*/

#define GREEN_LED_TASK_PRIORITY         ( tskIDLE_PRIORITY + 0 )
#define FLIR_TASK_PRIORITY              ( tskIDLE_PRIORITY + 1 )

#define FLIR_CSn_BASE                   ( GPIO_B_BASE )
#define FLIR_CSn_PIN                    ( GPIO_PIN_3 )

/*================================ typedef ==================================*/

/*=============================== prototypes ================================*/

static void prvGreenLedTask(void *pvParameters);
static void prvFlirTask(void *pvParameters);

/*=============================== variables =================================*/

static Task heartbeatTask{(const char *) "Heartbeat", 128, GREEN_LED_TASK_PRIORITY, prvGreenLedTask, nullptr};
static Task ina226Task{(const char *) "Flir", 128, FLIR_TASK_PRIORITY, prvFlirTask, nullptr};

static void timer0_callback(void);
static PlainCallback timerCallback0(timer0_callback);

static const GpioConfig flir_csn_cfg {FLIR_CSn_BASE, FLIR_CSn_PIN, 0, 0, 1};
static GpioOut flir_cs {flir_csn_cfg};
static Flir flir {spi1, flir_cs};

static SemaphoreBinary semaphore(false);

#pragma default_variable_attributes = @ "MY_DATA"
static uint8_t flir_buffer[FLIR_ROW_SIZE][FLIR_DATA_SIZE];
#pragma default_variable_attributes =

static uint16_t flir_buffer_len = sizeof(flir_buffer);

/*================================= public ==================================*/

void main(void)
{
  /* Initialize the board */
  board.init();
  
  /* Enable SPI */
  spi1.enable();

  /* Start the scheduler */
  Scheduler::run();
}

/*=============================== protected =================================*/

/*================================ private ==================================*/

static void prvFlirTask(void *pvParameters)
{
  /* Initialize Timer0 */
  timer0a.init();
  timer0a.setFrequency(37037);
  timer0a.setCallback(&timerCallback0);
  timer0a.enableInterrupts();
  
  while(true)
  {
    bool is_synchronized;
    
    /* Wake-up the camera */
    flir.wakeup();
    
    /* Syncronize with the camera, this will return once it is in sync */
    is_synchronized = flir.readFirst(&flir_buffer[0][0], flir_buffer_len);
    
    /* Start a one-shot timer to read the next frame */
    timer0a.start();
    
    /* Read the remaining of the frame */
    flir.readRemaining(&flir_buffer[1][0], flir_buffer_len);
    
    /* Forever */
    while (is_synchronized)
    {
      /* Wait for the one-shot timer to expire */
      semaphore.take();
      
      /* Read the first line of the frame to ensure we are still in sync */
      is_synchronized = flir.readFirst(&flir_buffer[0][0], flir_buffer_len);
      
      /* Start the one-shot timer to read the next frame */
      timer0a.start();
      
      /* Read the remaining of the frame */
      flir.readRemaining(&flir_buffer[1][0], flir_buffer_len);
    }
    
    /* Stop timer */
    timer0a.stop();
    
    /* Small delay before re-sync */
    Scheduler::delay_ms(10);
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

static void timer0_callback(void)
{
  /* Notify that the timer has expired */
  semaphore.giveFromInterrupt();
}
