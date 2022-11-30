/*================================ include ==================================*/

#include "OneWire.hpp"

#include <BoardImplementation.hpp>

#include <platform_includes.h>
#include <platform_types.hpp>

#include <string.h>

extern BoardImplementation board;

/*================================ define ===================================*/

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

static const uint8_t crc8_table[] = {
	0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83,
	0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
	0x00, 0x9D, 0x23, 0xBE, 0x46, 0xDB, 0x65, 0xF8,
	0x8C, 0x11, 0xAF, 0x32, 0xCA, 0x57, 0xE9, 0x74
};

/*=============================== prototypes ================================*/
/*================================= public ==================================*/

OneWire::OneWire(const GpioConfig& config):
	Gpio(config), semaphore(true)
{
	/* Set the pin as input */
	GPIOPinTypeGPIOInput(config_.port, config_.pin);

	resetSearch();
}

bool OneWire::reset() const
{
	uint8_t retries = 125;
	bool r;

	GPIOPinTypeGPIOInput(config_.port, config_.pin);

	do {
		if (--retries == 0) {
			return false;
		}
		board.delayMicroseconds2(2);
	} while (!GPIOPinRead(config_.port, config_.pin));

	taskENTER_CRITICAL();
	GPIOPinTypeGPIOOutput(config_.port, config_.pin);
	GPIOPinWrite(config_.port, config_.pin, 0);
	taskEXIT_CRITICAL();

	board.delayMicroseconds2(480);

	taskENTER_CRITICAL();
	GPIOPinTypeGPIOInput(config_.port, config_.pin);
	board.delayMicroseconds2(70);
	r = !GPIOPinRead(config_.port, config_.pin);
	taskEXIT_CRITICAL();

	board.delayMicroseconds2(410);

	return r;
}

void OneWire::select(const uint8_t* rom) const
{
	write(ONE_WIRE_SELECT_ROM);
	write(rom, ONE_WIRE_ROM_SIZE);
}

void OneWire::skip() const
{
	write(ONE_WIRE_SKIP_ROM);
}

void OneWire::depower() const
{
	taskENTER_CRITICAL();
	GPIOPinTypeGPIOInput(config_.port, config_.pin);
	GPIOPinWrite(config_.port, config_.pin, 0);
	taskEXIT_CRITICAL();
}

void OneWire::resetSearch()
{
	lastDiscrepancy = 0;
	lastDeviceFound = false;
	memset(lastRom, 0, sizeof(lastRom));
}

bool OneWire::search(uint8_t* rom)
{
	uint8_t lastZero = 0;
	bool idBit;
	bool idBitCmp;
	uint8_t idBitNumber = 1;
	uint8_t romByteNumber = 0;
	uint8_t romByteMask = 1;
	bool searchDirection;
	bool searchResult = false;

	if (!lastDeviceFound) {
		if (!reset()) {
			lastDiscrepancy = 0;
			lastDeviceFound = false;
			return false;
		}

		write(ONE_WIRE_SEARCH);

		do {
			idBit = readBit();
			idBitCmp = readBit();
			if (idBit && idBitCmp) {
				break;
			}

			if (idBit != idBitCmp) {
				searchDirection = idBit;
			} else {
				if (idBitNumber < lastDiscrepancy) {
					searchDirection = ((lastRom[romByteNumber] & romByteMask) > 0);
				} else {
					searchDirection = (idBitNumber == lastDiscrepancy);
				}

				if (!searchDirection) {
					lastZero = idBitNumber;
				}
			}

			if (searchDirection) {
				lastRom[romByteNumber] |= romByteMask;
			} else {
				lastRom[romByteNumber] &= ~romByteMask;
			}

			writeBit(searchDirection);

			++idBitNumber;
			romByteMask <<= 1;

			if (romByteMask == 0) {
				++romByteNumber;
				romByteMask = 1;
			}
		} while (romByteNumber < 8);

		if (!(idBitNumber < 65)) {
			lastDiscrepancy = lastZero;

			if (lastDiscrepancy == 0) {
				lastDeviceFound = true;
			}

			searchResult = true;
		}
	}

	if (!searchResult || !lastRom[0]) {
		lastDiscrepancy = 0;
		lastDeviceFound = false;
		searchResult = false;
	} else if (rom) {
		memcpy(rom, lastRom, sizeof(lastRom));
	}

	return searchResult;
}

void OneWire::write(uint8_t v) const
{
	uint8_t bitMask;

	for (bitMask = 0x01; bitMask; bitMask <<= 1) {
		writeBit(v & bitMask);
	}
}

void OneWire::write(const uint8_t* buff, size_t count) const
{
	while (count--) {
		write(*buff++);
	}
}

uint8_t OneWire::read() const
{
	uint8_t bitMask;
	uint8_t r = 0;
	bool bit;

	for (bitMask = 0x01; bitMask; bitMask <<= 1) {
		bit = readBit();
		if (bit) {
			r |= bitMask;
		}
	}

	return r;
}

void OneWire::read(uint8_t* buff, size_t count) const
{
	while (count--) {
		*buff++ = read();
	}
}

uint8_t OneWire::crc8(const uint8_t* buff, uint8_t len)
{
	uint8_t crc = 0;

	while (len--) {
		crc = *buff++ ^ crc;
		crc = *(crc8_table + (crc & 0x0f)) ^ *(crc8_table + 16 + ((crc >> 4) & 0x0f));
	}

	return crc;
}

bool OneWire::isValidRom(const uint8_t* rom) {
	return rom[ONE_WIRE_ROM_SIZE - 1] == crc8(rom, ONE_WIRE_ROM_SIZE - 1);
}

void OneWire::writeBit(bool bit) const
{
	if (bit) {
		taskENTER_CRITICAL();
		GPIOPinTypeGPIOOutput(config_.port, config_.pin);
		GPIOPinWrite(config_.port, config_.pin, 0);
		board.delayMicroseconds2(10);
		GPIOPinWrite(config_.port, config_.pin, config_.pin);
		taskEXIT_CRITICAL();
		board.delayMicroseconds2(65);
	} else {
		taskENTER_CRITICAL();
		GPIOPinTypeGPIOOutput(config_.port, config_.pin);
		GPIOPinWrite(config_.port, config_.pin, 0);
		board.delayMicroseconds2(65);
		GPIOPinWrite(config_.port, config_.pin, config_.pin);
		taskEXIT_CRITICAL();
	}

	board.delayMicroseconds2(1);
}

bool OneWire::readBit() const
{
	uint32_t r;

	taskENTER_CRITICAL();
	GPIOPinTypeGPIOOutput(config_.port, config_.pin);
	GPIOPinWrite(config_.port, config_.pin, 0);
	board.delayMicroseconds2(3);
	GPIOPinTypeGPIOInput(config_.port, config_.pin);
	board.delayMicroseconds2(10);
	r = GPIOPinRead(config_.port, config_.pin);
	taskEXIT_CRITICAL();

	board.delayMicroseconds2(52);

	return (bool) r;
}

/*=============================== protected =================================*/
/*================================ private ==================================*/
