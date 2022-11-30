#include <Rs485.hpp>

Rs485::Rs485(Uart& uart, GpioOut& txPin) : uart(uart), txPin(txPin) {
	txPin.off();
}

bool Rs485::readByte(uint8_t* byte) {
	return uart.readByte(byte);
}

uint32_t Rs485::writeBytes(const uint8_t* buffer, uint32_t length) {
	uint32_t ret;
	txPin.high();
	ret = uart.writeBytes(buffer, length);
	uart.flush();
	txPin.low();
	return ret;
}
