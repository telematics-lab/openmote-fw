#ifndef ONEWIRE_HPP_
#define ONEWIRE_HPP_

#include <stddef.h>

#include <Gpio.hpp>
#include <Semaphore.hpp>

#define ONE_WIRE_ROM_SIZE			8

#define ONE_WIRE_SELECT_ROM			0x55
#define ONE_WIRE_SKIP_ROM			0xcc
#define ONE_WIRE_SEARCH				0xf0

class OneWire : public Gpio
{
	public:
		OneWire(const GpioConfig& config);

	public:
		inline void lock() const { semaphore.take(); }
		inline void unlock() const { semaphore.give(); }
		inline void unlockFromInterrupt() const { semaphore.giveFromInterrupt(); }

		bool reset() const;
		void select(const uint8_t* rom) const;
		void skip() const;
		void depower() const;

		void resetSearch();
		bool search(uint8_t* rom);

		void write(uint8_t v) const;
		void write(const uint8_t* buff, size_t count) const;
		uint8_t read() const;
		void read(uint8_t* buff, size_t count) const;

		void writeBit(bool bit) const;
		bool readBit() const;

	public:
		static bool isValidRom(const uint8_t* rom);
		static uint8_t crc8(const uint8_t* buff, uint8_t len);

	private:
		SemaphoreBinary semaphore;
		uint8_t lastDiscrepancy;
		bool lastDeviceFound;
		uint8_t lastRom[ONE_WIRE_ROM_SIZE];
};

#endif /* ONEWIRE_HPP_ */
