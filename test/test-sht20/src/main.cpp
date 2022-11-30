#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>

#include <BoardImplementation.hpp>
#include <Callback.hpp>
#include <OneWire.hpp>
#include <Scheduler.hpp>
#include <Task.hpp>
#include <platform_types.hpp>

#include <sht20/Sht20.hpp>

#define HEARTBEAT_TASK_PRIORITY		(tskIDLE_PRIORITY + 0)
#define SENSOR_TASK_PRIORITY		(tskIDLE_PRIORITY + 1)

#define HEARTBEAT_TASK_STACK_SIZE	(128)
#define SENSOR_TASK_STACK_SIZE		(128)

static void heartbeat(void *pvParameters);
static void printSensor(void *pvParameters);

static Task heartbeatTask{"Heartbeat", HEARTBEAT_TASK_STACK_SIZE, HEARTBEAT_TASK_PRIORITY, heartbeat, nullptr};
static Task sensorTask{"Sensor", SENSOR_TASK_STACK_SIZE, SENSOR_TASK_PRIORITY, printSensor, nullptr};

static uint8_t uartBuffer[500];

static Sht20 sht20(i2c);

int main(void) {
	/* Initialize the board */
	board.init();

	i2c.enable();
	uart0.enable(115200);

	/* Start the scheduler */
	Scheduler::run();

	return 0;
}

static void heartbeat(void *pvParameters) {
	while (true) {
		led_red.on();
		Scheduler::delay_ms(50);

		led_red.off();
		Scheduler::delay_ms(950);
	}
}

static void printSensor(void *pvParameters) {
	int32_t temp;
	uint8_t rh;
	size_t len;

	Scheduler::delay_ms(1000);

	while (true) {
		if (!sht20.setResolution(SHT20_RES_RH12_T14)) {
			len = sprintf((char*) uartBuffer, "Set resolution error\r\n");
			led_yellow.on();
		} else if (!sht20.readTemp(&temp)) {
			len = sprintf((char*) uartBuffer, "Read temperature error\r\n");
			led_yellow.on();
		} else if (!sht20.readRH(&rh)) {
			len = sprintf((char*) uartBuffer, "Read relative humidity error\r\n");
			led_yellow.on();
		} else {
			len = sprintf((char*) uartBuffer, "Temp: %ld, RH: %u\r\n", temp, rh);
			led_green.on();
		}
		if (len > 0) {
			uart0.writeBytes(uartBuffer, len);
		}
		Scheduler::delay_ms(100);

		led_yellow.off();
		led_green.off();
		Scheduler::delay_ms(900);
	}
}
