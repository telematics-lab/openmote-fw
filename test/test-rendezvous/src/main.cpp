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

#include "FreeRTOS.h"
#include "task.h"

#include "board.h"

#include "Rendezvous.h"
#include "Scheduler.h"
#include "Semaphore.h"
#include "Task.h"

/*================================ define ===================================*/

#define GREEN_LED_TASK_PRIORITY             ( tskIDLE_PRIORITY + 0 )
#define YELLOW_LED_TASK_PRIORITY            ( tskIDLE_PRIORITY + 1 )
#define ORANGE_LED_TASK_PRIORITY            ( tskIDLE_PRIORITY + 2 )

/*================================ typedef ==================================*/

/*=============================== prototypes ================================*/

static void prvGreenLedTask(void *pvParameters);
static void prvYellowLedTask(void *pvParameters);
static void prvOrangeLedTask(void *pvParameters);

/*=============================== variables =================================*/

static Rendezvous rendezvous;

/*================================= public ==================================*/

int main (void)
{
    // Initialize board
    board.init();

    // Create two FreeRTOS tasks
    xTaskCreate(prvGreenLedTask, (const char *) "Green", 128, NULL, GREEN_LED_TASK_PRIORITY, NULL);
    xTaskCreate(prvYellowLedTask, (const char *) "Yellow", 128, NULL, YELLOW_LED_TASK_PRIORITY, NULL);
    xTaskCreate(prvOrangeLedTask, (const char *) "Orange", 128, NULL, ORANGE_LED_TASK_PRIORITY, NULL);

    // Start the scheduler
    Scheduler::run();
}

/*=============================== protected =================================*/

/*================================ private ==================================*/

static void prvGreenLedTask(void *pvParameters)
{
    static RendezvousId rendezvousId;

    if (!rendezvous.getId(&rendezvousId)) {
        led_red.on();
        while(true);
    }

    // Forever
    while (true) {
        // Turn off green LED for 1950 ms
        led_green.off();
        vTaskDelay(50 / portTICK_RATE_MS);

        // Synchronization point with maxium delay of 100 ms
        rendezvous.sync(rendezvousId, 100);

        // Turn on green LED for 50 ms
        led_green.on();
        vTaskDelay(50 / portTICK_RATE_MS);
    }
}

static void prvYellowLedTask(void *pvParameters)
{
    static RendezvousId rendezvousId;

    if (!rendezvous.getId(&rendezvousId)) {
        led_red.on();
        while(true);
    }

    // Forever
    while (true) {
        // Turn off yellow LED for 1950 ms
        led_yellow.off();
        vTaskDelay(50 / portTICK_RATE_MS);

        // Synchronization point with 1000 ms delay
        rendezvous.sync(rendezvousId, 1000);

        // Turn on yellow LED for 50 ms
        led_yellow.on();
        vTaskDelay(50 / portTICK_RATE_MS);
    }
}

static void prvOrangeLedTask(void *pvParameters)
{
    static RendezvousId rendezvousId;

    if (!rendezvous.getId(&rendezvousId)) {
        led_red.on();
        while(true);
    }

    // Forever
    while (true) {
        // Turn off green LED for 1950 ms
        led_orange.off();
        vTaskDelay(1950 / portTICK_RATE_MS);

        // Synchronization point with maximum delay
        rendezvous.sync(rendezvousId);

        // Turn on green LED for 50 ms
        led_orange.on();
        vTaskDelay(50 / portTICK_RATE_MS);
    }
}