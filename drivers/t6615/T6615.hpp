#ifndef T6615_HPP_
#define T6615_HPP_

#include <stdint.h>
#include <Uart.hpp>

#define T6615_SERIAL_NUMBER_LEN		15

#define T6615_BAUDRATE				19200

#define T6615_READY_STATUS			0x00
#define T6615_ERROR_STATUS			0x01
#define T6615_WARMUP_STATUS			0x02
#define T6615_CALIBRATION_STATUS	0x04
#define T6615_IDLE_STATUS			0x08
#define T6615_SELF_TEST_STATUS		0x80

class T6615 {
	public:
		T6615(Uart& uart);

	public:
		bool enable(); // Idle ON
		bool disable(); // Idle OFF

		bool getSerialNumber(char* serialNumber);
		bool getStatus(uint8_t* status);

		bool getElevation(uint16_t* elevation);
		bool setElevation(uint16_t elevation);

		bool readGasPpm(uint16_t* gasPpm);

	private:
		bool sendCmd(uint8_t cmd, uint8_t* req, size_t reqLen, uint8_t* res, size_t resLen);
		bool sendCmd(uint8_t cmd, uint8_t req, uint8_t* res = nullptr, size_t resLen = 0);
		bool sendCmd(uint8_t cmd, uint8_t* res = nullptr, size_t resLen = 0);
		bool waitResponse(uint8_t* res, size_t resLen);
		void discardPendingBytes();

	private:
		Uart& uart;
};

#endif /* T6615_HPP_ */

