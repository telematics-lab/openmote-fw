#ifndef DS18B20_HPP_
#define DS18B20_HPP_

#include <stdint.h>
#include <OneWire.hpp>

#define DS18B20_RES_9	0x00
#define DS18B20_RES_10	0x20
#define DS18B20_RES_11	0x40
#define DS18B20_RES_12	0x60

class Ds18b20
{
	public:
		Ds18b20(OneWire& onewire);

	public:
		bool enable(const uint8_t *rom);
		inline const uint8_t* getRom() const { return rom; }
		void disable();

		inline bool isParasitePower() const { return parasite; }
		bool requestTemperature() const;
		bool readRawTemp(uint16_t* raw) const;
		bool readTemp(int32_t* temp) const;
		bool readUserData(uint16_t* data) const;

		bool writeUserData(uint16_t data) const;

		bool setResolution(uint8_t resolution) const;

	public:
		static bool isDS18B20(const uint8_t *rom);
		static bool requestTemperatures(const OneWire& onewire, bool parasite);

	private:
		bool readScratchPad(uint8_t* scratchPad) const;
		bool readPowerSupply() const;

		bool writeScratchPad(uint8_t* scratchPad) const;
		bool copyScratchPad() const;

	private:
		static void waitForConversion(const OneWire& onewire, bool parasite);

	private:
		OneWire& onewire;
		uint8_t rom[ONE_WIRE_ROM_SIZE];
		bool parasite;
};

#endif /* DS18B20_HPP_ */

