#ifndef MHZ16_HPP_
#define MHZ16_HPP_

#include <stdint.h>
#include <Uart.hpp>

#define MHZ16_BAUDRATE				9600
#define MHZ16_WARMUP_TIME			(3 * 60 * 1000)
#define MHZ16_BUFFER_LEN			9

class Mhz16 {
	public:
		Mhz16(Uart& uart);

	public:
		bool setSelfCalibration(bool active);
		bool setRange(uint32_t range);
		bool readGasPpm(uint16_t* gasPpm);

	private:
		bool sendCmd(uint8_t cmd, uint8_t val8 = 0, uint32_t val32 = 0UL);
		inline bool sendCmd(uint8_t cmd, uint32_t val32) { return sendCmd(cmd, 0, val32); }
		bool waitResponse();
		void discardPendingBytes();
		uint8_t calcCrc();

	private:
		Uart& uart;

		uint8_t buffer[MHZ16_BUFFER_LEN];
};

#endif /* T6615_HPP_ */

