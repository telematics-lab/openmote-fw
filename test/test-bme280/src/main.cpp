#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>

#include "Gpio.hpp"
#include "I2c.hpp"
#include "Spi.hpp"

#include <BoardImplementation.hpp>
#include <Callback.hpp>
#include <OneWire.hpp>
#include <Scheduler.hpp>
#include <Task.hpp>
#include <platform_types.hpp>

#include "bme280/Bme280.hpp"

#define HEARTBEAT_TASK_PRIORITY		(tskIDLE_PRIORITY + 0)
#define SENSOR_TASK_PRIORITY		(tskIDLE_PRIORITY + 1)

#define HEARTBEAT_TASK_STACK_SIZE	(64)
#define SENSOR_TASK_STACK_SIZE		(128)

#define SENSORS_CTRL_PORT           ( GPIO_A_BASE )
#define SENSORS_CTRL_PIN            ( GPIO_PIN_7 )
#define BME280_I2C_ADDRESS          ( BME280_I2C_ADDR_PRIM )

static void heartbeat(void *pvParameters);
static void printSensor(void *pvParameters);

static GpioConfig sensors_pwr_cfg = {SENSORS_CTRL_PORT, SENSORS_CTRL_PIN, 0, 0, 0};
static GpioOut sensors_pwr_ctrl(sensors_pwr_cfg);

static Task heartbeatTask{"Heartbeat", HEARTBEAT_TASK_STACK_SIZE, HEARTBEAT_TASK_PRIORITY, heartbeat, nullptr};
static Task sensorTask{"Sensor", SENSOR_TASK_STACK_SIZE, SENSOR_TASK_PRIORITY, printSensor, nullptr};

static uint8_t uartBuffer[500];

static Bme280 bme280(i2c, BME280_I2C_ADDRESS);

typedef struct {
  uint16_t temperature;
  uint16_t humidity;
  uint16_t pressure;
} SensorData;

/*///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
int main(void) {

  /* Initialize the board */
  board.init();

  /* Turn on the sensors board. Set PA7 to high. */
  sensors_pwr_ctrl.high();

  /* Enable i2c */
  i2c.enable();

  /* Enable UART */
  uart0.enable(115200);

  /* Start the scheduler */
  Scheduler::run();

  return 0;
}

/*////////////////////////////////////////////////////////////////////////////////////////////*/
static void heartbeat(void *pvParameters) {

 /* Blinking red led */
  while (true) {
    led_red.on();
    Scheduler::delay_ms(50);

    led_red.off();
    Scheduler::delay_ms(950);
  }
}

/*////////////////////////////////////////////////////////////////////////////////////////////*/
static void printSensor(void *pvParameters) {

  Bme280Data bme280_data;
  SensorData sensorData;

  bool status;
  size_t len = 0;

  /* Initialize the bme280 */
  bme280.init();

  while (true) {

	/* Check whether we can read data from bme280 */
    status = bme280.read(&bme280_data);

    if (status)
    {
	  /* Fill-in sensor data */
      sensorData.temperature = (uint16_t) (bme280_data.temperature * 10.0f);
      sensorData.humidity = (uint16_t) (bme280_data.humidity * 10.0f);
      sensorData.pressure = (uint16_t) (bme280_data.pressure * 10.0f);

      led_green.on();
      len = sprintf((char*) uartBuffer, "Temperature: %u, Humidity: %u, Pressure: %u\r\n", sensorData.temperature, sensorData.humidity, sensorData.pressure);
    }

    if (len > 0)
    {
	  /* Write bytes to Serial */
      uart0.writeBytes(uartBuffer, len);
      len = 0;
    }

    Scheduler::delay_ms(100);

    led_green.off();

    Scheduler::delay_ms(900);
  }
}
