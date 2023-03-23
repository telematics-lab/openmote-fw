/**
 * @file       main.cpp
 * @author     A. Petrosino - G. Sciddurlo - F. Greco
 * @version    v0.02
 * @date       Mar, 2023
 * @brief
 *
 */

/*================================ include ==================================*/

#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include "Gpio.hpp"
#include "I2c.hpp"
#include "Spi.hpp"
#include <BoardImplementation.hpp>
#include <Callback.hpp>
#include <Scheduler.hpp>
#include <Task.hpp>
#include <platform_types.hpp>
#include "bme280/Bme280.hpp"
#include <string.h>
#include "Semaphore.hpp"
#include "opt3001/Opt3001.hpp"

/*================================ define ===================================*/

#define HEARTBEAT_TASK_PRIORITY		(tskIDLE_PRIORITY + 0)
#define SENSOR_TASK_PRIORITY		(tskIDLE_PRIORITY + 2)

#define PAYLOAD_LENGTH                      ( 200 )
#define EUI48_LENGTH                        ( 6 )

#define HEARTBEAT_TASK_STACK_SIZE	(64)
#define SENSOR_TASK_STACK_SIZE		(128)

#define SENSORS_CTRL_PORT           ( GPIO_A_BASE )
#define SENSORS_CTRL_PIN            ( GPIO_PIN_7 )
#define BME280_I2C_ADDRESS          ( BME280_I2C_ADDR_PRIM )

#define TX_BUFFER_LENGTH (127)
// #define EUI48_ADDDRESS_LENGTH (6)
#define EUI64_ADDDRESS_LENGTH (8)

#define UART_BAUDRATE                       ( 115200 )
#define RADIO_CHANNEL                       ( 26 )

#define OPT3001_I2C_ADDRESS                 ( OPT3001_I2C_ADDR_GND )

#define CURRENT_ROOM ( "Kitchen" )

/*================================ typedef ==================================*/

typedef struct {
  uint16_t temperature;
  uint16_t humidity;
  uint16_t pressure;
  uint16_t light;
} SensorData;

/*=============================== prototypes ================================*/

extern "C" void vApplicationTickHook(void);
extern "C" void vApplicationIdleHook(void);

static void txInit(void);
static void txDone(void);

static SemaphoreBinary rxSemaphore, txSemaphore;

static PlainCallback txInitCallback(&txInit);
static PlainCallback txDoneCallback(&txDone);

static uint8_t radio_buffer[PAYLOAD_LENGTH];
static uint8_t* radio_ptr = radio_buffer;
static uint8_t  radio_len = sizeof(radio_buffer);
static int8_t rssi;
static uint8_t lqi;

static void heartbeat(void *pvParameters);
static void printSensor(void *pvParameters);

// static uint8_t eui48_address[EUI48_ADDDRESS_LENGTH];
static uint8_t eui64_address[EUI64_ADDDRESS_LENGTH];

static GpioConfig sensors_pwr_cfg = {SENSORS_CTRL_PORT, SENSORS_CTRL_PIN, 0, 0, 0};
static GpioOut sensors_pwr_ctrl(sensors_pwr_cfg);

static Opt3001 opt3001(i2c, OPT3001_I2C_ADDRESS);

static Task heartbeatTask{"Heartbeat", HEARTBEAT_TASK_STACK_SIZE, HEARTBEAT_TASK_PRIORITY, heartbeat, nullptr};
static Task sensorTask{"Sensor", SENSOR_TASK_STACK_SIZE, SENSOR_TASK_PRIORITY, printSensor, nullptr};


static uint8_t uartBuffer[500];

static Bme280 bme280(i2c, BME280_I2C_ADDRESS);

// convert EUI bytes to human readable string
void byteArrayToString(uint8_t *byteArr, char *str, uint8_t arrLen)
{
    sprintf(str, "%02X", byteArr[0]);
    for (int i = 1; i < arrLen; i++)
        sprintf(str + strlen(str), "-%02X", byteArr[i]);
}

// unified function to fill both UART and transmit buffer
size_t fillBuffer(uint8_t* buffer, uint8_t* eui, SensorData &sensorData) {

  // EUI-48: "XX-XX-XX-XX-XX-XX"+'\0'
  // char euiStr[EUI48_ADDDRESS_LENGTH*3];
  // byteArrayToString(eui, euiStr, EUI48_ADDDRESS_LENGTH);

  // EUI-64: "XX-XX-XX-XX-XX-XX-XX-XX"+'\0'
  char euiStr[EUI64_ADDDRESS_LENGTH*3];
  byteArrayToString(eui, euiStr, EUI64_ADDDRESS_LENGTH);

  // return formatted buffer  
  // return sprintf((char*) buffer, "EUI-48\t%s\tRoom\t%s\tTemperature\t%u\tHumidity\t%u\tPressure\t%u\tLight\t%u\t\r\n", euiStr,CURRENT_ROOM , sensorData.temperature, sensorData.humidity, sensorData.pressure, sensorData.light);
  return sprintf((char*) buffer, "EUI-64\t%s\tRoom\t%s\tTemperature\t%u\tHumidity\t%u\tPressure\t%u\tLight\t%u\t\r\n", euiStr,CURRENT_ROOM , sensorData.temperature, sensorData.humidity, sensorData.pressure, sensorData.light);
}

/*///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
int main(void) {

  /* Initialize the board */
  board.init();

  // retrieve EUI48
  // board.getEUI48(eui48_address);
  board.getEUI64(eui64_address);

  // Enable the IEEE 802.15.4 radio
  radio.setTxCallbacks(&txInitCallback, &txDoneCallback);
  radio.enable();
  radio.enableInterrupts();
  radio.setChannel(RADIO_CHANNEL);

  /* Turn on the sensors board. Set PA7 to high. */
  sensors_pwr_ctrl.high();

  /* Enable i2c */
  i2c.enable();

  /* Enable UART */
  uart0.enable(UART_BAUDRATE);

  /* Start the scheduler */
  Scheduler::run();

  return 0;
}

/*////////////////////////////////////////////////////////////////////////////////////////////*/
static void heartbeat(void *pvParameters) {

 /* Blinking red led */
  while (true) {
    led_red.on();
    Scheduler::delay_ms(300);

    led_red.off();
    Scheduler::delay_ms(600);
  }
}


/*////////////////////////////////////////////////////////////////////////////////////////////*/
static void printSensor(void *pvParameters) {

  Bme280Data bme280_data;
  SensorData sensorData;
  Opt3001Data opt3001_data;

  static RadioResult result;

  bool status;
  bool sent;
  size_t len = 0;

  /* Initialize the bme280 */
  bme280.init();

  while (true) {

    /* Initialize and enable OPT3001 */
	  opt3001.init();
	  opt3001.enable();

    /* Read light */
    status = opt3001.read(&opt3001_data.raw);

    if (status)
    {
      /* Turn on yellow LED */
      led_yellow.on();
      opt3001.convert(opt3001_data.raw, &opt3001_data.lux);
      /* Fill-in sensor data */
      sensorData.light       = (uint16_t) (opt3001_data.lux * 10.0f);

    }

    //NEW
    uint16_t tx_buffer_len;

	/* Check whether we can read data from bme280 */
    status = bme280.read(&bme280_data);

    if (status)
    {
	  /* Fill-in sensor data */
      sensorData.temperature = (uint16_t) (bme280_data.temperature * 1.0f);
      sensorData.humidity = (uint16_t) (bme280_data.humidity * 1.0f);
      sensorData.pressure = (uint16_t) (bme280_data.pressure * 1.0f);

      led_green.on();
      // len = fillBuffer(uartBuffer, eui48_address, sensorData);
      len = fillBuffer(uartBuffer, eui64_address, sensorData);
    }

    if (len > 0)
    {
	  /* Write bytes to Serial */
      uart0.writeBytes(uartBuffer, len);
      len = 0;
    }
    
    led_green.on();    
    Scheduler::delay_ms(50);

    // Take the txSemaphre, block until available
    if (txSemaphore.take())
    {
        // Turn on the radio transceiver
        radio.on();

        // Turn the yellow LED on when the packet is being loaded
        led_yellow.on();

        // Load the packet to the transmit buffer
        // len = fillBuffer(radio_buffer, eui48_address, sensorData);
        len = fillBuffer(radio_buffer, eui64_address, sensorData);
        radio_ptr = radio_buffer;
        radio_len = sizeof(radio_buffer);
        radio_len = len;
        
        len = 0;

        result = radio.loadPacket(radio_ptr, radio_len);

        if (result == RadioResult_Success)
        {
            // Put the radio transceiver in transmit mode
            radio.transmit();
            Scheduler::delay_ms(500);
            // Turn the yellow LED off when the packet has beed loaded
            led_yellow.off();
        }

        // Delay the transmission of the next packet 50 ms        
        Scheduler::delay_ms(50);
    }

    
    led_green.off();

    Scheduler::delay_ms(500);
  }
}


static void txInit(void)
{
    // Turn on the radio LED as the packet is now transmitting
    // using orange LED so the red LED can work as heartbeat indicator without interruption
    led_orange.on();
}

static void txDone(void)
{
    // Turn off the radio LED as the packet is transmitted
    led_orange.off();

    // Let the task run once a packet is transmitted
    txSemaphore.giveFromInterrupt();
}