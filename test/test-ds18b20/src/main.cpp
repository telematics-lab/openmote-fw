#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>

#include <BoardImplementation.hpp>
#include <Callback.hpp>
#include <OneWire.hpp>
#include <Scheduler.hpp>
#include <Task.hpp>
#include <platform_types.hpp>

#include <ds18b20/Ds18b20.hpp>

#define HEARTBEAT_TASK_PRIORITY		(tskIDLE_PRIORITY + 0)
#define SENSOR_TASK_PRIORITY		(tskIDLE_PRIORITY + 1)

#define HEARTBEAT_TASK_STACK_SIZE	(128)
#define SENSOR_TASK_STACK_SIZE		(128)

#define ONE_WIRE_PORT            ( GPIO_B_BASE )
#define ONE_WIRE_PIN             ( GPIO_PIN_0 )

static void heartbeat(void *pvParameters);
static void printSensor(void *pvParameters);

static Task heartbeatTask{"Heartbeat", HEARTBEAT_TASK_STACK_SIZE, HEARTBEAT_TASK_PRIORITY, heartbeat, nullptr};
static Task sensorTask{"Sensor", SENSOR_TASK_STACK_SIZE, SENSOR_TASK_PRIORITY, printSensor, nullptr};

static uint8_t uartBuffer[500];

static const GpioConfig oneWireGpioConfig{ONE_WIRE_PORT, ONE_WIRE_PIN, 0, 0, 0};
static OneWire oneWire(oneWireGpioConfig);
static Ds18b20 ds18b20[] = {
	Ds18b20(oneWire),
	Ds18b20(oneWire),
	Ds18b20(oneWire),
	Ds18b20(oneWire),
	Ds18b20(oneWire),
	Ds18b20(oneWire),
	Ds18b20(oneWire),
	Ds18b20(oneWire),
	Ds18b20(oneWire),
	Ds18b20(oneWire),
};
static const int maxSensors = sizeof(ds18b20) / sizeof(Ds18b20);

int main(void) {
	/* Initialize the board */
	board.init();

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
	int numSensors = 0;
	uint8_t addr[ONE_WIRE_ROM_SIZE];
	int32_t temp;
	uint16_t userData = 0x7766;
	int len;

	Scheduler::delay_ms(1000);

	len = sprintf((char *) uartBuffer, "Search one wire devices...\r\n");
	uart0.writeBytes(uartBuffer, len);
	oneWire.resetSearch();
	while (oneWire.search(addr) && (numSensors < maxSensors)) {
		if (!ds18b20[numSensors].enable(addr)) {
			continue;
		}

		len = sprintf((char *) uartBuffer, "%d - new DS18B20: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\r\nParasite power: %s\r\n",
				numSensors,
				addr[0], addr[1], addr[2], addr[3], addr[4], addr[5], addr[6], addr[7],
				ds18b20[numSensors].isParasitePower() ? "yes" : "no");
		uart0.writeBytes(uartBuffer, len);

		if (!ds18b20[numSensors].setResolution(DS18B20_RES_12)) {
			len = sprintf((char*) uartBuffer, "Set resolution error\r\n");
			uart0.writeBytes(uartBuffer, len);
		}

		++numSensors;
	}

	while (true) {
		for (int i = 0; i < numSensors; ++i) {
			if (!ds18b20[i].requestTemperature()) {
				len = sprintf((char*) uartBuffer, "Request error\r\n");
				led_yellow.on();

			} else if (!ds18b20[i].readTemp(&temp)) {
				len = sprintf((char*) uartBuffer, "Read temperature error\r\n");
				led_yellow.on();

			} else if (temp < -55000 || temp > 125000) {
				len = sprintf((char*) uartBuffer, "Invalid temperature: %ld\r\n", temp);
				led_yellow.on();

			} else if (!ds18b20[i].readUserData(&userData)) {
				len = sprintf((char*) uartBuffer, "Read user data error\r\n");
				led_yellow.on();

			} else {
				len = sprintf((char*) uartBuffer, "%d - temp: %ld, user data: 0x%04x\r\n", i, temp, userData);
				led_green.on();

			}
			if (len > 0) {
				uart0.writeBytes(uartBuffer, len);
			}
		}
		Scheduler::delay_ms(100);

		led_yellow.off();
		led_green.off();
		Scheduler::delay_ms(900);
	}
}
