#include <FreeRTOS.h>
#include <stdio.h>

#include <BoardImplementation.hpp>
#include <Callback.hpp>
#include <Scheduler.hpp>
#include <Task.hpp>

#include <at86rf215/At86rf215.hpp>
#include <at86rf215/At86rf215_conf.h>

#define HEARTBEAT_TASK_PRIORITY		(tskIDLE_PRIORITY + 0)
#define RADIO_TASK_PRIORITY			(tskIDLE_PRIORITY + 1)

#define HEARTBEAT_TASK_STACK_SIZE	(128)
#define RADIO_TASK_STACK_SIZE		(256)

#define SPI_BAUDRATE				(16000000)

#define RADIO_CHANNEL				(0)
#define RADIO_CORE					(At86rf215::CORE_RF09)
#define RADIO_FREQUENCY				(&frequency_settings_09[FREQUENCY_09_OFDM1])
#define RADIO_SETTINGS				(&radio_settings[CONFIG_OFDM1_MCS0])
#define RADIO_TX_POWER				(At86rf215::TransmitPower::TX_POWER_MAX)

static void doHeartBeat(void *pvParameters);
static void doRadioTx(void *pvParameters);

static void radioTxInited();
static void radioTxDone();

static Task heartbeatTask{"Heartbeat", HEARTBEAT_TASK_STACK_SIZE, HEARTBEAT_TASK_PRIORITY, doHeartBeat, nullptr};
static Task radioTask{"Radio", RADIO_TASK_STACK_SIZE, RADIO_TASK_PRIORITY, doRadioTx, nullptr};

static PlainCallback radioTxInitCallback{&radioTxInited};
static PlainCallback radioTxDoneCallback{&radioTxDone};
static SemaphoreBinary radioSemaphore{false};

int main(void) {
	/* Initialize the board */
	board.init();

	spi0.enable(SPI_BAUDRATE);

	antenna_at86rf215.high();
	antenna_cc2538.low();

	/* Start the scheduler */
	Scheduler::run();

	return 0;
}

static void doHeartBeat(void *pvParameters) {
	while (true) {
		led_red.on();
		Scheduler::delay_ms(50);

		led_red.off();
		Scheduler::delay_ms(950);
	}
}

static void doRadioTx(void *pvParameters) {
	uint32_t counter = 0;

	at86rf215.on();

	if (!at86rf215.check()) {
		led_orange.on();
		while (true) {
			Scheduler::delay_ms(1000);
		}
	}

	at86rf215.setTxCallbacks(RADIO_CORE, &radioTxInitCallback, &radioTxDoneCallback);
	at86rf215.enableInterrupts();

	at86rf215.configure(RADIO_CORE, RADIO_SETTINGS, RADIO_FREQUENCY, RADIO_CHANNEL);
	at86rf215.setTransmitPower(RADIO_CORE, RADIO_TX_POWER);

	while (true) {
		led_green.on();

		at86rf215.wakeup(RADIO_CORE);
		at86rf215.loadPacket(RADIO_CORE, (uint8_t*) &counter, sizeof(counter));
		at86rf215.transmit(RADIO_CORE);

		radioSemaphore.take();

		at86rf215.sleep(RADIO_CORE);
		++counter;
		led_green.off();

		Scheduler::delay_ms(1000);
	}
}

static void radioTxInited() {

}

static void radioTxDone() {
	radioSemaphore.giveFromInterrupt();
}
