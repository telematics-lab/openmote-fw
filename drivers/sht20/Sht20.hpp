#ifndef __SHT20_HPP__
#define __SHT20_HPP__

#include <stdint.h>

#include <I2c.hpp>

#define SHT20_I2C_ADDR		(0x40)

#define SHT20_RES_RH12_T14	(0x00)
#define SHT20_RES_RH8_T12	(0x01)
#define SHT20_RES_RH10_T13	(0x80)
#define SHT20_RES_RH11_T11	(0x81)

class Sht20 {
	public:
		explicit Sht20(I2c& i2c, uint8_t address = SHT20_I2C_ADDR);

		bool readTemp(int32_t* temp) const;
		bool readRH(uint8_t* rh) const;
		bool setResolution(uint8_t resolution) const;

		bool readUserRegister(uint8_t* userRegister) const;
		bool writeUserRegister(uint8_t userRegister) const;

	private:
		bool readValue(uint8_t cmd, uint16_t* value) const;

		static uint8_t calcCrc(uint16_t value);

	private:
		I2c& i2c;
		uint8_t address;
};

#endif
