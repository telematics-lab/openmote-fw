/**
 * @file       main.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2016
 * @brief
 *
 * @copyright  Copyright 2016, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "BoardImplementation.hpp"

#include "Scheduler.hpp"
#include "Semaphore.hpp"
#include "Task.hpp"

/*================================ define ===================================*/

#define GREEN_LED_TASK_PRIORITY         	( tskIDLE_PRIORITY + 0 )
#define RADIO_TASK_PRIORITY              	( tskIDLE_PRIORITY + 3 )
#define TEMPERATURE_TASK_PRIORITY       	( tskIDLE_PRIORITY + 2 )
#define BUTTON_TASK_PRIORITY                ( tskIDLE_PRIORITY + 1 )

#define UART_BAUDRATE                       ( 115200 )
#define SPI_BAUDRATE                        ( 8000000 )

/*================================ typedef ==================================*/

/*=============================== prototypes ================================*/

static void prvGreenLedTask(void *pvParameters);
static void prvTemperatureTask(void *pvParameters);
static void prvRadioTask(void *pvParameters);

static void prvButtonTask(void *pvParameters);
static void buttonCallback(void);

/*=============================== variables =================================*/

static SemaphoreBinary buttonSemaphore;
static PlainCallback userCallback(buttonCallback);

static uint8_t uart_buffer[32];
static uint8_t uart_len;

/*================================= public ==================================*/

int main(void) {
    // Initialize the board
    board.init();

    // Enable the UART interface
    uart.enable(UART_BAUDRATE);

    // Enable the SPI interface
    spi.enable(SPI_BAUDRATE);

    // Enable general interrupts
    board.enableInterrupts();

    // Create the FreeRTOS tasks
    xTaskCreate(prvGreenLedTask, (const char *) "GreenLed", 128, NULL, GREEN_LED_TASK_PRIORITY, NULL);
    xTaskCreate(prvRadioTask, (const char *) "Radio", 128, NULL, RADIO_TASK_PRIORITY, NULL);
    xTaskCreate(prvButtonTask, (const char *) "ButtonTask", 128, NULL, BUTTON_TASK_PRIORITY, NULL);

    // Start the scheduler
    Scheduler::run();
}

/*================================ private ==================================*/

static void prvRadioTask(void *pvParameters)
{
	bool status; 

    // Forever
    while (true)
    {
        // Turn AT86RF215 radio on
        at86rf215.on();

        status = at86rf215.check();
        if (status) {
        	led_yellow.on();
        } else {
        	led_red.on();
        }

        // Turn AT86RF215 radio off
        at86rf215.off();

        Scheduler::delay_ms(100);

        led_yellow.off();
        led_red.off();

        Scheduler::delay_ms(900);
    }
}

static void prvGreenLedTask(void *pvParameters)
{
    // Forever
    while (true)
    {
        // Turn off green LED for 1000 ms
        led_green.off();
        Scheduler::delay_ms(1000);

        // Turn on green LED for 1000 ms
        led_green.on();
        Scheduler::delay_ms(1000);
    }
}

static void prvButtonTask(void *pvParameters)
{
    // Configure the user button
    button_user.setCallback(&userCallback);
    button_user.enableInterrupts();

    // Forever
    while(true)
    {
        // Take the buttonSemaphore, block until available
        if (buttonSemaphore.take())
        {
            led_red.toggle();
        }
    }
}

/*================================ private ==================================*/

static void buttonCallback(void)
{
    // Give the button semaphore as the button has been pressed
    buttonSemaphore.giveFromInterrupt();
}
