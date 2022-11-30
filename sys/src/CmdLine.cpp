#include "CmdLine.hpp"

////////////////////////////////////////////////////////////////////////////////////////////////////
CmdLine::CmdLine(Uart& uart) : uart(uart) {

}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool CmdLine::read(char* cmd, size_t len) const {
	char* ptr = cmd;
	char in;

	// Prompt
	uart.writeBytes((uint8_t*) "> ", 2);

	do {
		uart.readBytes((uint8_t*) &in, 1);
		uart.writeBytes((uint8_t*) &in, 1);

		if ((ptr == cmd) && (in == ' ')) {
			// Trim input
		} else if ((in == '\r') || (in == '\n')) {
			if (in == '\r') {
				uart.writeBytes((uint8_t*) "\n", 1);
			} else if (ptr == cmd) {
				// Receive only '\n', ignore it
				continue;
			}

			// Finish input
			*ptr++ = '\0';
			return true;

		} else if ((in == 0x7f) || (in == '\b')) {
			if (ptr > cmd) {
				--ptr;
				uart.writeBytes((uint8_t*) "\b \b", 3);
			}
		} else {
			*ptr++ = in;
		}
	} while (uint32_t(ptr - cmd) < len - 1);

	return false;
}
