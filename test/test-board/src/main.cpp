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

#include "Board.hpp"

#include "LedBlinker.hpp"

#include "Scheduler.hpp"
#include "Task.hpp"

/*================================ define ===================================*/

#define RED_LED_TASK_PRIORITY             ( tskIDLE_PRIORITY + 0 )
#define YELLOW_LED_TASK_PRIORITY          ( tskIDLE_PRIORITY + 1 )
#define ORANGE_LED_TASK_PRIORITY          ( tskIDLE_PRIORITY + 2 )

#define BUTTON_TASK_PRIORITY              ( tskIDLE_PRIORITY + 3 )

/*================================ typedef ==================================*/

/*=============================== prototypes ================================*/

static void prvButtonTask(void *pvParameters);
static void buttonCallback(void);

/*=============================== variables =================================*/

static LedBlinker redLedBlinker("redLedBlinker", 128, RED_LED_TASK_PRIORITY, led_red);
static LedBlinker yellowLedBlinker("yellowLedBlinker", 128, YELLOW_LED_TASK_PRIORITY, led_yellow);
static LedBlinker orangeLedBlinker("orangeLedBlinker", 128, ORANGE_LED_TASK_PRIORITY, led_orange);

static SemaphoreBinary buttonSemaphore;
static PlainCallback userCallback(buttonCallback);

/*================================= public ==================================*/

int main(void) {
	// Initialize the board
    board.init();

    // Set blinking periods
    yellowLedBlinker.setTime(200, 1600);
    orangeLedBlinker.setTime(400, 1600);
    redLedBlinker.setTime(800, 1600);

    // Create button task
    xTaskCreate(prvButtonTask, (const char *) "ButtonTask", 128, NULL, BUTTON_TASK_PRIORITY, NULL);

    // Start the scheduler
    Scheduler::run();
}

/*================================ private ==================================*/

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
            led_green.toggle();
        }
    }
}

static void buttonCallback(void)
{
    // Give the button semaphore as the button has been pressed
    buttonSemaphore.giveFromInterrupt();
}