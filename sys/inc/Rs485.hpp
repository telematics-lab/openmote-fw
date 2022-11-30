#ifndef __RS485_HPP__
#define __RS485_HPP__

#include <Uart.hpp>
#include <Gpio.hpp>

class Rs485 {
	public:
		Rs485(Uart& uart, GpioOut& txPin);

	public:
		bool readByte(uint8_t* byte);
		uint32_t writeBytes(const uint8_t* buffer, uint32_t length);

	private:
		Uart& uart;
		GpioOut& txPin;
};

#endif
