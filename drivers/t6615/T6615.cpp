/*================================ include ==================================*/

#include "T6615.hpp"
#include "Scheduler.hpp"

/*================================ define ===================================*/

#define BUFFER_LEN		20
#define T6615_FLAG		uint8_t(0xff)
#define T6615_SLAVE		uint8_t(0xfe)
#define T6615_MASTER	uint8_t(0xfa)
#define T6615_TIMEOUT	1000

#define CMD_READ		uint8_t(0x02)
#define CMD_UPDATE		uint8_t(0x03)
#define CMD_STATUS		uint8_t(0xb6)
#define CMD_IDLE		uint8_t(0xb9)

#define SERIAL_NUMBER	uint8_t(0x01)
#define GAS_PPM			uint8_t(0x03)
#define COMPILE_DATE	uint8_t(0x0c)
#define COMPILE_SUBVOL	uint8_t(0x0d)
#define ELEVATION		uint8_t(0x0f)

#define IDLE_ON			uint8_t(0x01)
#define IDLE_OFF		uint8_t(0x02)

/*================================ typedef ==================================*/

/*=============================== variables =================================*/
static uint8_t buffer[BUFFER_LEN];

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

T6615::T6615(Uart& uart) : uart(uart) {

}

bool T6615::enable() {
	return sendCmd(CMD_IDLE, IDLE_ON);
}

bool T6615::disable() {
	return sendCmd(CMD_IDLE, IDLE_OFF);
}

bool T6615::getSerialNumber(char* serialNumber) {
	return sendCmd(CMD_READ, SERIAL_NUMBER, (uint8_t*) serialNumber, T6615_SERIAL_NUMBER_LEN);
}

bool T6615::getStatus(uint8_t* status) {
	return sendCmd(CMD_STATUS, status, 1);
}

bool T6615::getElevation(uint16_t* elevation) {
	uint8_t v[2];
	bool ret = sendCmd(CMD_READ, ELEVATION, v, 2);
	if (elevation) {
		*elevation = (uint16_t(v[0]) << 8) | uint16_t(v[1]);
	}
	return ret;
}

bool T6615::setElevation(uint16_t elevation) {
	uint8_t b[3];
	uint8_t* ptr = b;
	*ptr++ = ELEVATION;
	*ptr++ = (elevation & 0xff00) >> 8;
	*ptr++ = (elevation & 0x00ff);
	return sendCmd(CMD_UPDATE, b, ptr - b);
}

bool T6615::readGasPpm(uint16_t* gasPpm) {
	uint8_t v[2];
	bool ret = sendCmd(CMD_READ, GAS_PPM, v, 2);
	if (gasPpm) {
		*gasPpm = (uint16_t(v[0]) << 8) | uint16_t(v[1]);
	}
	return ret;
}

/*=============================== protected =================================*/

/*================================ private ==================================*/

bool T6615::sendCmd(uint8_t cmd, uint8_t* req, size_t reqLen, uint8_t* res, size_t resLen) {
	uint8_t* ptr = buffer;

	discardPendingBytes();

	if (!req) {
		reqLen = 0;
	}

	*ptr++ = T6615_FLAG;
	*ptr++ = T6615_SLAVE;
	*ptr++ = reqLen + 1;
	*ptr++ = cmd;
	for (size_t i = 0; i < reqLen; ++i) {
		*ptr++ = *req++;
	}

	if (uart.writeBytes(buffer, ptr - buffer) != 0) {
		return false;
	}

	return waitResponse(res, resLen);
}

bool T6615::sendCmd(uint8_t cmd, uint8_t req, uint8_t* res, size_t resLen) {
	return sendCmd(cmd, &req, 1, res, resLen);
}

bool T6615::sendCmd(uint8_t cmd, uint8_t* res, size_t resLen) {
	return sendCmd(cmd, nullptr, 0, res, resLen);
}

bool T6615::waitResponse(uint8_t* res, size_t resLen) {
	uint32_t startTime = Scheduler::get_ms();
	uint8_t b;
	size_t len = 0;
	bool ret = false;

	if (!res) {
		resLen = 0;
	}

	do {
		if (uart.readByte(&b)) {
			switch (len) {
				case 0:
					if (b != T6615_FLAG) {
						discardPendingBytes();
						return false;
					}
					break;

				case 1:
					if (b != T6615_MASTER) {
						discardPendingBytes();
						return false;
					}
					break;

				case 2:
					if (b != resLen) {
						discardPendingBytes();
						return false;
					}
					break;

				default:
					*res++ = b;
					--resLen;
					break;
			}
			++len;

			if ((len >= 3) && (resLen == 0)) {
				ret = true;
				break;
			}
		}
		Scheduler::delay_ms(1);
	} while (Scheduler::get_ms() - startTime < T6615_TIMEOUT);

	return ret;
}

void T6615::discardPendingBytes() {
	uint8_t b;
	while (uart.readByte(&b));
}
