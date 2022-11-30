#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>

#include <BoardImplementation.hpp>
#include <Callback.hpp>
#include <OneWire.hpp>
#include <Scheduler.hpp>
#include <Task.hpp>
#include <platform_types.hpp>

#include <Rs485.hpp>

#define HEARTBEAT_TASK_PRIORITY		(tskIDLE_PRIORITY + 0)
#define RX_TASK_PRIORITY			(tskIDLE_PRIORITY + 1)

#define HEARTBEAT_TASK_STACK_SIZE	(128)
#define RX_TASK_STACK_SIZE			(128)

#define RS485_GPIO_PORT				(GPIO_C_BASE)
#define RS485_GPIO_PIN				(GPIO_PIN_3)

static void heartbeat(void *pvParameters);
static void doRxTask(void *pvParameters);

static Task heartbeatTask{"Heartbeat", HEARTBEAT_TASK_STACK_SIZE, HEARTBEAT_TASK_PRIORITY, heartbeat, nullptr};
static Task rxTask{"Receive", RX_TASK_STACK_SIZE, RX_TASK_PRIORITY, doRxTask, nullptr};

static const GpioConfig rs485ModeConfig{RS485_GPIO_PORT, RS485_GPIO_PIN, 0, 0, 0};
static GpioOut rs485Mode(rs485ModeConfig);
static Rs485 rs485(uart1, rs485Mode);

int main(void) {
	/* Initialize the board */
	board.init();

	uart0.enable(115200);
	uart1.enable(9600);

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

static void doRxTask(void *pvParameters) {
	uint8_t rx;

	while (true) {
		if (rs485.readByte(&rx)) {
			led_green.on();

			uart0.writeByte((rx % 10) + '0');
			uart0.writeByte('\r');
			uart0.writeByte('\n');

			rs485.writeBytes(&rx, 1);
		}

		Scheduler::delay_ms(1);
		led_green.off();

	}
}
