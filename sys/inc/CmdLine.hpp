#ifndef __CMD_LINE_HPP__
#define __CMD_LINE_HPP__

#include <Uart.hpp>

class CmdLine {
	public:
		explicit CmdLine(Uart& uart);

	public:
		bool read(char* cmd, size_t len) const;

	private:
		Uart& uart;
};

#endif
