#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>

#include <BoardImplementation.hpp>
#include <Callback.hpp>
#include <Scheduler.hpp>
#include <Task.hpp>
#include <Modbus.hpp>

#define HEARTBEAT_TASK_PRIORITY		(tskIDLE_PRIORITY + 0)
#define MODBUS_TASK_PRIORITY		(tskIDLE_PRIORITY + 1)

#define HEARTBEAT_TASK_STACK_SIZE	(128)
#define MODBUS_TASK_STACK_SIZE		(128)

#define SLAVE_ADDR					25
#define NUM_HOLDING_REGISTERS		1024

static void heartbeat(void *pvParameters);
static void doModbusTask(void *pvParameters);

static Task heartbeatTask{"Heartbeat", HEARTBEAT_TASK_STACK_SIZE, HEARTBEAT_TASK_PRIORITY, heartbeat, nullptr};
static Task modbusTask{"Modbus", MODBUS_TASK_STACK_SIZE, MODBUS_TASK_PRIORITY, doModbusTask, nullptr};

static ModbusRTUSlave modbus(uart1, SLAVE_ADDR);
static uint16_t holdingRegisters[NUM_HOLDING_REGISTERS];

int main(void) {
	/* Initialize the board */
	board.init();

	uart0.enable(115200);
	uart1.enable(19200, UART_CONFIG_WLEN_8 | UART_CONFIG_PAR_EVEN | UART_CONFIG_STOP_ONE);

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

static void doModbusTask(void *pvParameters) {
	for (int i = 0; i < NUM_HOLDING_REGISTERS; ++i) {
		holdingRegisters[i] = 0;
	}
	holdingRegisters[1] = 1;
	modbus.setHoldingRegisters(holdingRegisters, NUM_HOLDING_REGISTERS);

	while (true) {
		modbus.update();

		// Update outputs
		holdingRegisters[0] ? led_green.on() : led_green.off();
		holdingRegisters[1] ? led_yellow.on() : led_yellow.off();
		holdingRegisters[2] ? led_orange.on() : led_orange.off();

		Scheduler::delay_ms(10);
	}
}
