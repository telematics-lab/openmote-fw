/*================================ include ==================================*/

#include "Mhz16.hpp"

#include <Scheduler.hpp>

/*================================ define ===================================*/

#define MHZ16_TIMEOUT		30000

#define START_BYTE			uint8_t(0xff)

#define CMD_READ_PPM		uint8_t(0x86)
#define CMD_SET_RANGE		uint8_t(0x99)
#define CMD_SET_AUTOCAL		uint8_t(0x79)

#define AUTOCAL_ON			uint8_t(0xa0)
#define AUTOCAL_OFF			uint8_t(0x00)

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

Mhz16::Mhz16(Uart& uart) : uart(uart) {

}

bool Mhz16::setSelfCalibration(bool active) {
	return sendCmd(CMD_SET_AUTOCAL, active ? AUTOCAL_ON : AUTOCAL_OFF);
}

bool Mhz16::setRange(uint32_t range) {
	return sendCmd(CMD_SET_RANGE, range);
}

bool Mhz16::readGasPpm(uint16_t* gasPpm) {
	if (!sendCmd(CMD_READ_PPM)) {
		return false;
	}

	if (!waitResponse()) {
		return false;
	}

	if (gasPpm) {
		*gasPpm = (uint16_t(buffer[2]) << 8) | uint16_t(buffer[3]);
	}

	return true;
}

/*=============================== protected =================================*/

/*================================ private ==================================*/

bool Mhz16::sendCmd(uint8_t cmd, uint8_t val8, uint32_t val32) {
	uint8_t* ptr = buffer;

	discardPendingBytes();

	*ptr++ = START_BYTE;
	*ptr++ = 0x01;
	*ptr++ = cmd;
	*ptr++ = val8;
	*ptr++ = val32 >> 24;
	*ptr++ = val32 >> 16;
	*ptr++ = val32 >> 8;
	*ptr++ = val32;
	*ptr++ = calcCrc();

	return uart.writeBytes(buffer, ptr - buffer) == 0;
}

bool Mhz16::waitResponse() {
	uint32_t startTime = Scheduler::get_ms();
	uint8_t b;
	size_t len = 0;
	bool ret = false;
	uint8_t* res = buffer;

	do {
		if (uart.readByte(&b)) {
			*res++ = b;
			++len;

			if (len == MHZ16_BUFFER_LEN) {

				uint8_t crc = calcCrc();
				if (buffer[MHZ16_BUFFER_LEN - 1] == crc) {
					ret = true;
				}
				break;
			}
		}
		Scheduler::delay_ms(1);
	} while (Scheduler::get_ms() - startTime < MHZ16_TIMEOUT);

	return ret;
}

void Mhz16::discardPendingBytes() {
	uint8_t b;
	while (uart.readByte(&b));
}

uint8_t Mhz16::calcCrc() {
	uint8_t len = MHZ16_BUFFER_LEN - 2;
	uint8_t ret = 0x00;
	uint8_t* buff = buffer + 1;

	while (len--) {
		ret += *buff++;
	}
	return (~ret) + 1;
}
