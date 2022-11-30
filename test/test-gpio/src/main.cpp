#include <FreeRTOS.h>

#include <BoardImplementation.hpp>
#include <Gpio.hpp>
#include <Scheduler.hpp>
#include <Task.hpp>

#include <platform_types.hpp>

#define TEST_HIGH	1
#define TEST_LOW	1
#define TEST_HZ		1

#define HIGH_TIME	5000
#define LOW_TIME	5000
#define HZ_TIME		5000

#define HEARTBEAT_TASK_PRIORITY		(tskIDLE_PRIORITY + 0)

#define HEARTBEAT_TASK_STACK_SIZE	(128)

class GpioBi : public Gpio {
	public:
		GpioBi(const GpioConfig& config) : Gpio(config) {
			GPIOPinTypeGPIOInput(config_.port, config_.pin);
		}

	public:
		void high() {
			GPIOPinTypeGPIOOutput(config_.port, config_.pin);
			GPIOPinWrite(config_.port, config_.pin, config_.pin);
		}
		void low() {
			GPIOPinTypeGPIOOutput(config_.port, config_.pin);
			GPIOPinWrite(config_.port, config_.pin, 0);
		}
		void hz() {
			GPIOPinTypeGPIOInput(config_.port, config_.pin);
		}
};

static void doHeartBeat(void *pvParameters);

static Task heartbeatTask{"Heartbeat", HEARTBEAT_TASK_STACK_SIZE, HEARTBEAT_TASK_PRIORITY, doHeartBeat, nullptr};

static const GpioConfig outputsConfig[] = {
	{GPIO_B_BASE, GPIO_PIN_0, 0, 0, 0},
	{GPIO_B_BASE, GPIO_PIN_1, 0, 0, 0},
	{GPIO_B_BASE, GPIO_PIN_2, 0, 0, 0},
	{GPIO_B_BASE, GPIO_PIN_3, 0, 0, 0},
	{GPIO_C_BASE, GPIO_PIN_3, 0, 0, 0},
	{GPIO_A_BASE, GPIO_PIN_7, 0, 0, 0}
};
static const uint8_t numOutputs = sizeof(outputsConfig) / sizeof(GpioConfig);
static GpioBi outputs[numOutputs] = {
	outputsConfig[0],
	outputsConfig[1],
	outputsConfig[2],
	outputsConfig[3],
	outputsConfig[4],
	outputsConfig[5],
};

int main(void) {
	/* Initialize the board */
	board.init();

	/* Start the scheduler */
	Scheduler::run();

	return 0;
}

static void doHeartBeat(void *pvParameters) {
	while (true) {
#if TEST_HIGH
		led_red.on();
		for (uint8_t i = 0; i < numOutputs; ++i) {
			outputs[i].high();
		}
		Scheduler::delay_ms(HIGH_TIME);
#endif

#if TEST_LOW
		led_red.off();
		for (uint8_t i = 0; i < numOutputs; ++i) {
			outputs[i].low();
		}
		Scheduler::delay_ms(LOW_TIME);
#endif

#if TEST_HZ
		led_red.off();
		for (uint8_t i = 0; i < numOutputs; ++i) {
			outputs[i].hz();
		}
		Scheduler::delay_ms(HZ_TIME);
#endif
	}
}

