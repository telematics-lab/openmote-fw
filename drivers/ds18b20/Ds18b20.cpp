/*================================ include ==================================*/

#include "BoardImplementation.hpp"
#include "Ds18b20.hpp"
#include "Scheduler.hpp"

#include "platform_types.h"
#include <string.h>

/*================================ define ===================================*/

#define WAIT_FOR_CONVERSION		1
#define SCRATCHPAD_SIZE			9
#define WAIT_CONVERSION_TIMEOUT	1000

#define REQUEST_TEMPERATURE		0x44
#define WRITE_SCRATCHPAD		0x4e
#define COPY_SCRATCHPAD			0x48
#define READ_POWERSUPPLY		0xb4
#define READ_SCRATCHPAD			0xbe

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

Ds18b20::Ds18b20(OneWire& onewire):
	onewire(onewire)
{
	disable();
}

bool Ds18b20::enable(const uint8_t *rom)
{
	if (!isDS18B20(rom)) {
		return false;
	}

	memcpy(this->rom, rom, ONE_WIRE_ROM_SIZE);
	parasite = readPowerSupply();

	return true;
}

void Ds18b20::disable()
{
	memset(this->rom, 0, ONE_WIRE_ROM_SIZE);
}

bool Ds18b20::requestTemperature() const
{
	onewire.lock();

	if (!onewire.reset()) {
		onewire.unlock();
		return false;
	}
	onewire.select(rom);
	onewire.write(REQUEST_TEMPERATURE);

	if (!parasite) {
		onewire.depower();
	}

#if WAIT_FOR_CONVERSION
	waitForConversion(onewire, parasite);
#endif

	onewire.unlock();

	return true;
}

bool Ds18b20::readTemp(int32_t* temp) const
{
	uint16_t rawTemp;
	if (!readRawTemp(&rawTemp)) {
		return false;
	}

	if (temp) {
		// mC
		*temp = 1000 * int32_t(rawTemp);
		// Resolution 12: 0.0625 per step ---> temp * 0.0625 == temp / 16 == temp >> 4
		*temp >>= 4;
	}

	return true;
}

bool Ds18b20::readRawTemp(uint16_t* raw) const
{
	uint8_t scratchPad[SCRATCHPAD_SIZE];
	if (!readScratchPad(scratchPad)) {
		return false;
	}

	if (raw) {
		*raw = (uint16_t(scratchPad[1]) << 8) | uint16_t(scratchPad[0]);
	}

	return true;
}

bool Ds18b20::readUserData(uint16_t* data) const
{
	uint8_t scratchPad[SCRATCHPAD_SIZE];
	if (!readScratchPad(scratchPad)) {
		return false;
	}

	if (data) {
		*data = (uint16_t(scratchPad[3]) << 8) | uint16_t(scratchPad[2]);
	}

	return true;
}

bool Ds18b20:: writeUserData(uint16_t data) const
{
	uint8_t scratchPad[SCRATCHPAD_SIZE];
	if (!readScratchPad(scratchPad)) {
		return false;
	}

	scratchPad[2] = data;
	scratchPad[3] = data >> 8;

	if (!writeScratchPad(scratchPad)) {
		return false;
	}

	return copyScratchPad();
}

bool Ds18b20::isDS18B20(const uint8_t *rom)
{
	return OneWire::isValidRom(rom) && (rom[0] == 0x28);
}

bool Ds18b20::requestTemperatures(const OneWire& onewire, bool parasite)
{
	onewire.lock();

	if (!onewire.reset()) {
		onewire.unlock();
		return false;
	}
	onewire.skip();
	onewire.write(REQUEST_TEMPERATURE);

	if (!parasite) {
		onewire.depower();
	}

#if WAIT_FOR_CONVERSION
	waitForConversion(onewire, parasite);
#endif

	onewire.unlock();

	return true;
}

bool Ds18b20::setResolution(uint8_t resolution) const {
	uint8_t scratchPad[SCRATCHPAD_SIZE];
	if (!readScratchPad(scratchPad)) {
		return false;
	}

	scratchPad[4] = 0x9f;
	scratchPad[4] |= resolution;

	return writeScratchPad(scratchPad);
}

/*=============================== protected =================================*/
/*================================ private ==================================*/
bool Ds18b20::readScratchPad(uint8_t* scratchPad) const
{

	onewire.lock();

	if (!onewire.reset()) {
		onewire.unlock();
		return false;
	}
	onewire.select(rom);
	onewire.write(READ_SCRATCHPAD);
	onewire.read(scratchPad, SCRATCHPAD_SIZE);

	bool ret = onewire.reset();

	onewire.unlock();

	return ret;
}

bool Ds18b20::readPowerSupply() const
{
	bool parasiteMode = false;

	onewire.lock();

	if (!onewire.reset()) {
		onewire.unlock();
		return false;
	}
	onewire.select(rom);
	onewire.write(READ_POWERSUPPLY);
	parasiteMode = !onewire.readBit();
	onewire.reset();

	onewire.unlock();

	return parasiteMode;
}

bool Ds18b20::writeScratchPad(uint8_t* scratchPad) const
{

	onewire.lock();

	if (!onewire.reset()) {
		onewire.unlock();
		return false;
	}
	onewire.select(rom);
	onewire.write(WRITE_SCRATCHPAD);
	onewire.write(scratchPad + 2, 3);

	bool ret = onewire.reset();

	onewire.unlock();

	return ret;
}

bool Ds18b20::copyScratchPad() const
{
	onewire.lock();

	if (!onewire.reset()) {
		onewire.unlock();
		return false;
	}

	onewire.select(rom);
	onewire.write(COPY_SCRATCHPAD);

	if (!parasite) {
		onewire.depower();
	}

	Scheduler::delay_ms(10);

	onewire.unlock();

	return true;
}

void Ds18b20::waitForConversion(const OneWire& onewire, bool parasite) {
	if (!parasite) {
		uint32_t startTime = Scheduler::get_ms();
		while (Scheduler::get_ms() - startTime < WAIT_CONVERSION_TIMEOUT) {
			if (onewire.readBit() == 1) {
				break;
			}
		}
	} else {
		// Wait conversion time depending on resolution (12bits => 750ms)
		// Only 12 bits resolution
		Scheduler::delay_ms(750);
	}
}
