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

#include <cstdio>

#include "FreeRTOS.h"
#include "task.h"

#include "BoardImplementation.hpp"
#include "platform_types.hpp"

#include "Gpio.hpp"
#include "I2c.hpp"
#include "Spi.hpp"

#include "Callback.hpp"
#include "Task.hpp"
#include "Scheduler.hpp"
#include "Semaphore.hpp"

#include "opt3001/Opt3001.hpp"

/*================================ define ===================================*/

#define GREEN_LED_TASK_PRIORITY             ( tskIDLE_PRIORITY + 0 )
#define SENSOR_TASK_PRIORITY                ( tskIDLE_PRIORITY + 1 )

#define GREEN_LED_TASK_STACK_SIZE           ( 128 )
#define SENSOR_TASK_STACK_SIZE              ( 128 )

#define SENSORS_CTRL_PORT                   ( GPIO_A_BASE )
#define SENSORS_CTRL_PIN                    ( GPIO_PIN_7 )

#define OPT3001_I2C_ADDRESS                 ( OPT3001_I2C_ADDR_GND )

/*================================ typedef ==================================*/

static void prvGreenLedTask(void *pvParameters);
static void prvSensorTask(void *pvParameters);

/*=============================== variables =================================*/

static Task heartbeatTask{(const char *) "Green", GREEN_LED_TASK_STACK_SIZE, GREEN_LED_TASK_PRIORITY, prvGreenLedTask, nullptr};
static Task sensorTask{(const char *) "Sensor", SENSOR_TASK_STACK_SIZE, SENSOR_TASK_PRIORITY, prvSensorTask, nullptr};

static GpioConfig sensors_pwr_cfg = {SENSORS_CTRL_PORT, SENSORS_CTRL_PIN, 0, 0, 0};
static GpioOut sensors_pwr_ctrl(sensors_pwr_cfg);

static Opt3001 opt3001(i2c, OPT3001_I2C_ADDRESS);

typedef struct {
  uint16_t light;
} SensorData;

static SemaphoreBinary semaphore(false);
static uint8_t uartBuffer[500];

/*================================= public ==================================*/

int main(void)
{
  /* Initialize the board */
  board.init();

  /* Turn on the sensors board */
  sensors_pwr_ctrl.high();

  /* Enable I2C */
  i2c.enable();

  /* Enable UART */
  uart0.enable(115200);

  /* Start the scheduler */
  Scheduler::run();
}

/*================================ private ==================================*/

static void prvSensorTask(void *pvParameters)
{

  /* Delay for 100 milliseconds */
  Scheduler::delay_ms(100);

  /* Forever */
  while (true) {
    SensorData sensor_data;

    Opt3001Data opt3001_data;

    bool status;
	size_t len = 0;

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
      sensor_data.light       = (uint16_t) (opt3001_data.lux * 10.0f);

	  len = sprintf((char*) uartBuffer, "Light: %u\r\n", sensor_data.light);

	}

	if (len > 0)
	{
	  /* Write bytes to Serial */
      uart0.writeBytes(uartBuffer, len);
      len = 0;
	}

    Scheduler::delay_ms(100);

    /* Turn off LEDs */
	led_yellow.off();

    /* Delay for 10 seconds */
    Scheduler::delay_ms(10000);
  }
}

static void prvGreenLedTask(void *pvParameters)
{
  /* Forever */
  while (true) {
    /* Turn off green LED for 9999 ms */
    led_green.off();
    Scheduler::delay_ms(999);

    /* Turn on green LED for 1 ms */
    led_green.on();
    Scheduler::delay_ms(1);
  }
}
