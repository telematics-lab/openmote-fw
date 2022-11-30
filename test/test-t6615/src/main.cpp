#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>

#include <BoardImplementation.hpp>
#include <Callback.hpp>
#include <Scheduler.hpp>
#include <Task.hpp>

#include <t6615/T6615.hpp>

#define HEARTBEAT_TASK_PRIORITY		(tskIDLE_PRIORITY + 0)
#define SENSOR_TASK_PRIORITY		(tskIDLE_PRIORITY + 1)

#define HEARTBEAT_TASK_STACK_SIZE	(128)
#define SENSOR_TASK_STACK_SIZE		(128)

static void heartbeat(void *pvParameters);
static void printSensor(void *pvParameters);

static Task heartbeatTask{"Heartbeat", HEARTBEAT_TASK_STACK_SIZE, HEARTBEAT_TASK_PRIORITY, heartbeat, nullptr};
static Task sensorTask{"Sensor", SENSOR_TASK_STACK_SIZE, SENSOR_TASK_PRIORITY, printSensor, nullptr};

static uint8_t uartBuffer[500];

T6615 t6615(uart1);

int main(void) {
	/* Initialize the board */
	board.init();

	uart0.enable(115200);
	uart1.enable(19200);

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
	uint32_t len;
	uint16_t gasPpm;
	uint8_t status;
	char serialNumber[T6615_SERIAL_NUMBER_LEN];

	Scheduler::delay_ms(10000);

	while (true) {
		if (!t6615.getSerialNumber(serialNumber)) {
			led_orange.on();
			len = sprintf((char*) uartBuffer, "Serial number error\r\n");
		} else if (!t6615.getStatus(&status)) {
			led_yellow.on();
			len = sprintf((char*) uartBuffer, "Get status error\r\n");
		} else if (status != 0x00) {
			led_yellow.on();
			len = sprintf((char*) uartBuffer, "Not ready: 0x%02x\r\n", status);
		} else if (!t6615.readGasPpm(&gasPpm)) {
			led_yellow.on();
			len = sprintf((char*) uartBuffer, "Read gas ppm error\r\n");
		} else {
			led_green.on();
			len = sprintf((char*) uartBuffer, "Serial: %s, status: 0x%02x, gas ppm: %d\r\n", serialNumber, status, gasPpm);
		}
		if (len > 0) {
			uart0.writeBytes(uartBuffer, len);
			len = 0;
		}
		Scheduler::delay_ms(10);

		led_green.off();
		led_yellow.off();
		led_orange.off();
		Scheduler::delay_ms(5000);
	}
}
