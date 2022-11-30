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

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "BoardImplementation.hpp"

#include "Callback.hpp"
#include "Scheduler.hpp"
#include "Semaphore.hpp"
#include "Serial.hpp"
#include "Task.hpp"

/*================================ define ===================================*/

#define RADIO_MODE_RX                       ( 0 )
#define RADIO_MODE_TX                       ( 1 )

#define RADIO_CORE_24G                      ( At86rf215::CORE_RF24 )
#define RADIO_CORE_SUB                      ( At86rf215::CORE_RF09 )

#define RADIO_MODE                          ( RADIO_MODE_TX )
#define RADIO_CORE                          ( RADIO_CORE_24G )

#define PAYLOAD_LENGTH                      ( 125 )
#define EUI48_LENGTH                        ( 6 )

#define UART_BAUDRATE                       ( 115200 )
#define SPI_BAUDRATE                        ( 8000000 )

#define GREEN_LED_TASK_PRIORITY             ( tskIDLE_PRIORITY + 2 )
#define RADIO_RX_TASK_PRIORITY              ( tskIDLE_PRIORITY + 0 )
#define RADIO_TX_TASK_PRIORITY              ( tskIDLE_PRIORITY + 0 )

/*================================ typedef ==================================*/

/*=============================== prototypes ================================*/

static void prvGreenLedTask(void *pvParameters);
static void prvRadioRxTask(void *pvParameters);
static void prvRadioTxTask(void *pvParameters);

static void rxInit(void);
static void rxDone(void);
static void txInit(void);
static void txDone(void);

/*=============================== variables =================================*/

static SemaphoreBinary rxSemaphore, txSemaphore;

static PlainCallback rxInitCallback(&rxInit);
static PlainCallback rxDoneCallback(&rxDone);
static PlainCallback txInitCallback(&txInit);
static PlainCallback txDoneCallback(&txDone);

static uint8_t radio_buffer[PAYLOAD_LENGTH];
static uint8_t* radio_ptr = radio_buffer;
static uint8_t  radio_len = sizeof(radio_buffer);
static int8_t rssi;
static uint8_t lqi;
static uint8_t crc;

static Serial serial(uart);

/*================================= public ==================================*/

int main (void)
{
    // Initialize board
    board.init();

    // Enable the SPI interface
    spi.enable(SPI_BAUDRATE);

    // Set radio callbacks
    at86rf215.setTxCallbacks(RADIO_CORE, &txInitCallback, &txDoneCallback);
    at86rf215.setRxCallbacks(RADIO_CORE, &rxInitCallback, &rxDoneCallback);
    at86rf215.enableInterrupts();

    // Turn AT86RF215 radio on
    at86rf215.on();

    // Create the blink task
    xTaskCreate(prvGreenLedTask, (const char *) "Green", 128, NULL, GREEN_LED_TASK_PRIORITY, NULL);

#if (RADIO_MODE == RADIO_MODE_RX)
    // Enable the UART driver
    uart.enable(UART_BAUDRATE);

    // Create the radio receive task
    xTaskCreate(prvRadioRxTask, (const char *) "RadioRx", 128, NULL, RADIO_RX_TASK_PRIORITY, NULL);
#elif (RADIO_MODE == RADIO_MODE_TX)
    // Create the radio transmit task
    xTaskCreate(prvRadioTxTask, (const char *) "RadioTx", 128, NULL, RADIO_TX_TASK_PRIORITY, NULL);
#endif

    // Start the scheduler
    Scheduler::run();
}

/*================================ private ==================================*/

static void prvGreenLedTask(void *pvParameters)
{
    // Forever
    while (true)
    {
        // Turn off the green LED and keep it for 950 ms
        led_green.off();
        vTaskDelay(950 / portTICK_RATE_MS);

        // Turn on the green LED and keep it for 50 ms
        led_green.on();
        vTaskDelay(50 / portTICK_RATE_MS);
    }
}

static void prvRadioRxTask(void *pvParameters)
{
    static At86rf215::RadioResult result;

    // Forever
    while (true)
    {
        // Put the radio transceiver in receive mode
        at86rf215.ready(RADIO_CORE);
        at86rf215.receive(RADIO_CORE);

        // Turn the yellow LED on when a the radio is receiving
        led_yellow.on();

        // Take the rxSemaphre, block until available
        if (rxSemaphore.take())
        {
            // Turn the yellow LED off when a packet is received
            led_yellow.off();

            // Get a packet from the radio buffer
            radio_ptr = radio_buffer;
            radio_len = sizeof(radio_buffer);
            result = at86rf215.getPacket(RADIO_CORE, radio_ptr, &radio_len, &rssi, &lqi, &crc);

            if (result == At86rf215::Success && crc)
            {
                uart.writeByte(rssi);
                uart.writeByte(crc >> 7);
            }
        }
    }
}

static void prvRadioTxTask(void *pvParameters)
{
    static At86rf215::RadioResult result;

    // Get the EUI64 address of the board
    board.getEUI48(radio_buffer);

    // Forever
    while (true)
    {
        // Take the txSemaphre, block until available
        if (txSemaphore.take())
        {
            // Turn on the radio transceiver
            at86rf215.ready(RADIO_CORE);

            // Turn the yellow LED on when the packet is being loaded
            led_yellow.on();

            // Load the EUI64 address to the transmit buffer
            radio_ptr = radio_buffer;
            radio_len = EUI48_LENGTH;
            result = at86rf215.loadPacket(RADIO_CORE, radio_ptr, radio_len);

            if (result == At86rf215::Success)
            {
                // Put the radio transceiver in transmit mode
                at86rf215.transmit(RADIO_CORE);

                // Turn the yellow LED off when the packet has beed loaded
                led_yellow.off();
            }

            // Delay the transmission of the next packet 250 ms
            vTaskDelay(250 / portTICK_RATE_MS);
        }
    }
}

static void rxInit(void)
{
    // Turn on the radio LED as the radio is now receiving a packet
    led_red.on();
}

static void rxDone(void)
{
    // Turn off the radio LED as the packet is received
    led_red.off();

    // Let the task run once a packet is received
    rxSemaphore.giveFromInterrupt();
}

static void txInit(void)
{
    // Turn on the radio LED as the packet is now transmitting
    led_red.on();
}

static void txDone(void)
{
    // Turn off the radio LED as the packet is transmitted
    led_red.off();

    // Let the task run once a packet is transmitted
    txSemaphore.giveFromInterrupt();
}
