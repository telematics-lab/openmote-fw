/*================================ include ==================================*/

#include "Sht20.hpp"

#include <BoardImplementation.hpp>
#include <Scheduler.hpp>

/*================================ define ===================================*/

#define WAIT_CONVERSION_TIMEOUT	100		// ms
#define READ_VALUE_DATA_LEN		3
#define CRC_POLY				0x131UL
#define SHIFTED_CRC_POLY		((CRC_POLY) << 15)

#define READ_TEMP_HOLD			0xe3
#define READ_RH_HOLD			0xe5
#define READ_TEMP_NOHOLD		0xf3
#define READ_RH_NOHOLD			0xf5
#define WRITE_USER_REGISTER		0xe6
#define READ_USER_REGISTER		0xe7
#define SOFT_RESET				0xfe

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

Sht20::Sht20(I2c& i2c, uint8_t address) : i2c(i2c), address(address) {

}

bool Sht20::readTemp(int32_t* temp) const {
	uint16_t value;
	if (!readValue(READ_TEMP_NOHOLD, &value)) {
		return false;
	}

	if (temp) {
		*temp = (((int32_t(value) * 17572L) >> 16) - 4685L) * 10;
	}

	return true;
}

bool Sht20::readRH(uint8_t* rh) const {
	uint16_t value;
	if (!readValue(READ_RH_NOHOLD, &value)) {
		return false;
	}

	if (rh) {
		*rh = ((uint32_t(value) * 125UL) >> 16) - 6;
	}

	return true;
}

bool Sht20::setResolution(uint8_t resolution) const {
	uint8_t userRegister;
	if (!readUserRegister(&userRegister)) {
		return false;
	}

	userRegister &= 0x7e;
	userRegister |= resolution & 0x81;

	return writeUserRegister(userRegister);
}

bool Sht20::readUserRegister(uint8_t* userRegister) const {
	i2c.lock();

	if (!i2c.writeByte(address, READ_USER_REGISTER)) {
		i2c.unlock();
		return false;
	}

	if (!i2c.readByte(address, userRegister)) {
		i2c.unlock();
		return false;
	}

	i2c.unlock();
	return true;
}

bool Sht20::writeUserRegister(uint8_t userRegister) const {
	i2c.lock();

	uint8_t buff[2];
	buff[0] = WRITE_USER_REGISTER;
	buff[1] = userRegister;

	if (!i2c.writeByte(address, buff, 2)) {
		i2c.unlock();
		return false;
	}

	i2c.unlock();
	return true;
}

/*=============================== protected =================================*/

/*================================ private ==================================*/

bool Sht20::readValue(uint8_t cmd, uint16_t* value) const {
	bool done = false;

	i2c.lock();

	if (!i2c.writeByte(address, cmd)) {
		i2c.unlock();
		return false;
	}

	uint32_t startTime = Scheduler::get_ms();
	uint8_t data[READ_VALUE_DATA_LEN];
	do {
		if (!i2c.readByte(address, data, READ_VALUE_DATA_LEN)) {
			Scheduler::delay_ms(1);
		} else {
			done = true;
			break;
		}
	} while (Scheduler::get_ms() - startTime < WAIT_CONVERSION_TIMEOUT);

	i2c.unlock();

	if (!done) {
		return false;
	}

	uint16_t dataValue = (uint16_t(data[0]) << 8) + uint16_t(data[1]);
	if (data[2] != Sht20::calcCrc(dataValue)) {
		return false;
	}

	if (value) {
		*value = dataValue;
	}

	return true;
}

uint8_t Sht20::calcCrc(uint16_t value) {
    uint32_t remainder = ((uint32_t) value) << 8;
    uint32_t divisor = SHIFTED_CRC_POLY;
    for (int i = 0; i < 16; ++i) {
        if (remainder & (1UL << (23 - i))) {
            remainder ^= divisor;
        }
        divisor >>= 1;
    }
    return remainder;
}
