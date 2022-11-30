/**
 * @file       Logging.hpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2016
 * @brief
 *
 * @copyright  Copyright 2016, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef LOGGING_HPP_
#define LOGGING_HPP_

#include <stdint.h>
#include <stdbool.h>

#include "Callback.hpp"
#include "Buffer.hpp"
#include "Mutex.hpp"

class Logging;
class Uart;

enum LogLevel {
	None,
	Call,
	Info,
	Warn,
	Error,
	All
};

typedef GenericCallback<Logging> LoggingCallback;

class Logging
{

friend class LoggingManager;

public:
	Logging(Uart& uart);
	void init(void);
	void setLevel(LogLevel level);
	void log(char const * message);
	void call(char const * message);
	void info(char const * message);
	void warn(char const * message);
	void error(char const * message);
private:
	void txCallback(void);
private:
	Uart& uart_;

	Mutex mutex_;
	LoggingCallback txCallback_;

	uint8_t transmit_buffer_[256];
  Buffer txBuffer_;
	
	LogLevel level_;

	bool initialized_;
};

#endif /* LOGGING_HPP_ */
