/*================================ include ==================================*/

#include <Modbus.hpp>
#include <Scheduler.hpp>

#include <BoardImplementation.hpp>

/*================================ define ===================================*/

#define READ_COILS					1
#define READ_DISCRETE_INPUTS		2
#define READ_HOLDING_REGISTERS		3
#define READ_INPUT_REGISTERS		4
#define WRITE_COIL					5
#define WRITE_HOLDING_REGISTER		6
#define WRITE_COILS					15
#define WRITE_HOLDING_REGISTERS		16

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

uint16_t Modbus::crc(const uint8_t* buff, size_t len) {
	int i;
	uint16_t crc = 0xffff;

	while (len--) {
		crc ^= *buff++;
		for (i = 0; i < 8; ++i) {
			crc = (crc & 1) ? ((crc >> 1) ^ 0xA001) : (crc >> 1);
		}
	}

	return ((crc >> 8) & 0xff) | ((crc << 8) & 0xff00);
}

ModbusRTUMaster::ModbusRTUMaster(Rs485& rs485, uint32_t responseTimeout) :
	rs485(rs485), responseTimeout(responseTimeout) {

}

bool ModbusRTUMaster::readHoldingRegister(uint8_t slave, uint16_t addr, uint16_t* value) {
	return readHoldingRegisters(slave, addr, value, 1);
}

bool ModbusRTUMaster::readHoldingRegisters(uint8_t slave, uint16_t addr, uint16_t* values, uint16_t num) {
	if (!isValidSlave(slave)) {
		return false;
	}
	if ((num == 0) || (num > 125)) {
		return false;
	}

	uint8_t *pdu = preparePdu(slave, READ_HOLDING_REGISTERS);

	*pdu++ = addr >> 8;
	*pdu++ = addr;
	*pdu++ = num >> 8;
	*pdu++ = num;

	uint16_t crc = Modbus::crc(adu, pdu - adu);
	*pdu++ = crc >> 8;
	*pdu++ = crc;

	discardPendingBytes();

	if (rs485.writeBytes(adu, pdu - adu) != 0) {
		return false;
	}

	pdu = receiveResponse();
	if (!pdu) {
		return false;
	}

	if (*pdu++ != slave) {
		// Invalid slave
		return false;
	}

	if (*pdu++ != READ_HOLDING_REGISTERS) {
		// Invalid FC
		return false;
	}

	uint8_t byteCount = num * 2;
	if (*pdu++ != byteCount) {
		// Invalid length
		return false;
	}

	while (num--) {
		*values = *pdu++ << 8;
		*values += *pdu++;
		++values;
	}

	return true;
}

bool ModbusRTUMaster::writeHoldingRegister(uint8_t slave, uint16_t addr, uint16_t value) {
	return writeHoldingRegisters(slave, addr, &value, 1);
}

bool ModbusRTUMaster::writeHoldingRegisters(uint8_t slave, uint16_t addr, uint16_t* values, uint16_t num) {
	if (!isValidSlave(slave)) {
		return false;
	}
	if ((num == 0) || (num > 123)) {
		return false;
	}

	uint8_t *pdu = preparePdu(slave, WRITE_HOLDING_REGISTERS);

	*pdu++ = addr >> 8;
	*pdu++ = addr;
	*pdu++ = num >> 8;
	*pdu++ = num;
	*pdu++ = num << 1;

	while (num--) {
		*pdu++ = *values >> 8;
		*pdu++ = *values++;
	}

	uint16_t crc = Modbus::crc(adu, pdu - adu);
	*pdu++ = crc >> 8;
	*pdu++ = crc;

	discardPendingBytes();

	if (rs485.writeBytes(adu, pdu - adu) != 0) {
		return false;
	}

	pdu = receiveResponse();
	if (!pdu) {
		return false;
	}

	if (*pdu++ != slave) {
		// Invalid slave
		return false;
	}

	if (*pdu++ != WRITE_HOLDING_REGISTERS) {
		// Invalid FC
		return false;
	}

	return true;
}

ModbusRTUSlave::ModbusRTUSlave(Rs485& rs485, uint8_t slaveAddr) :
	rs485(rs485), slaveAddr(slaveAddr),
	holdingRegisters(nullptr), numHoldingRegisters(0)
{

}

void ModbusRTUSlave::setHoldingRegisters(uint16_t* holdingRegisters, uint16_t numHoldingRegisters) {
	holdingRegistersMutex.take();
	this->holdingRegisters = holdingRegisters;
	this->numHoldingRegisters = numHoldingRegisters;
	holdingRegistersMutex.give();
}

void ModbusRTUSlave::update() {
	uint8_t b;
	uint8_t* pdu = adu;
	uint32_t lastByteTime = Scheduler::get_ms();

	do {
		if (rs485.readByte(&b)) {
			lastByteTime = Scheduler::get_ms();

			*pdu++ = b;

		} else if (pdu == adu) {
			// No request received
			break;

		} else if ((pdu != adu) && (Scheduler::get_ms() - lastByteTime > MODBUS_RTU_BYTE_TIMEOUT)) {
			// Frame finished

			if (pdu - adu < 3) {
				// Bad request
				break;
			}

			uint16_t crc = (uint16_t(*(pdu - 2)) << 8) | uint16_t(*(pdu - 1));
			if (crc != Modbus::crc(adu, pdu - adu - 2)) {
				// Bad CRC
				break;
			}

			if (adu[0] != slaveAddr) {
				// Not for me
				break;
			}

			// Process request
			uint8_t err = processRequest(adu + MODBUS_RTU_ADU_HEADER_SIZE, &pdu);
			if (err) {
				pdu = adu + MODBUS_RTU_ADU_HEADER_SIZE;
				*pdu++ |= 0x80;
				*pdu++ = err;
			}

			crc = Modbus::crc(adu, pdu - adu);
			*pdu++ = crc >> 8;
			*pdu++ = crc;

			Scheduler::delay_ms(5);

			// Send response
			rs485.writeBytes(adu, pdu - adu);
			break;

		} else {
			Scheduler::delay_ms(1);
		}
	} while (true);
}

void ModbusRTUSlave::setHoldingRegister(uint16_t addr, uint16_t value) {
	if (addr < numHoldingRegisters) {
		holdingRegistersMutex.take();
		holdingRegisters[addr] = value;
		holdingRegistersMutex.give();
	}
}

uint16_t ModbusRTUSlave::getHoldingRegister(uint16_t addr) {
	uint16_t value = 0;

	if (addr < numHoldingRegisters) {
		holdingRegistersMutex.take();
		value = holdingRegisters[addr];
		holdingRegistersMutex.give();
	}

	return value;
}

/*=============================== protected =================================*/

/*================================ private ==================================*/

bool ModbusRTUMaster::isValidSlave(uint8_t slave) {
	return (slave > 0) && (slave < 248);
}

uint8_t* ModbusRTUMaster::preparePdu(uint8_t slave, uint8_t fc) {
	uint8_t *pdu = adu;

	*pdu++ = slave;
	*pdu++ = fc;

	return pdu;
}

void ModbusRTUMaster::discardPendingBytes() {
	uint8_t b;
	while (rs485.readByte(&b));
}

uint8_t* ModbusRTUMaster::receiveResponse() {
	uint8_t b;
	uint8_t* pdu = adu;
	uint32_t startTime = Scheduler::get_ms();
	uint32_t lastByteTime = Scheduler::get_ms();
	do {
		if (rs485.readByte(&b)) {
			lastByteTime = Scheduler::get_ms();

			*pdu++ = b;

		} else if ((pdu == adu) && (Scheduler::get_ms() - startTime > responseTimeout)) {
			// Timeout
			break;

		} else if ((pdu != adu) && (Scheduler::get_ms() - lastByteTime > MODBUS_RTU_BYTE_TIMEOUT)) {
			// Frame finished

			if (pdu - adu < 3) {
				// Bad response
				break;
			}

			uint16_t crc = (uint16_t(*(pdu - 2)) << 8) | uint16_t(*(pdu - 1));
			if (crc != Modbus::crc(adu, pdu - adu - 2)) {
				// Bad CRC
				break;
			}

			return adu;

		} else {
			Scheduler::delay_ms(1);

		}
	} while (true);

	return nullptr;
}

uint8_t ModbusRTUSlave::processRequest(uint8_t* pdu, uint8_t** pduRet) {
	uint8_t ret;

	switch (*pdu) {
		case READ_HOLDING_REGISTERS:
			ret = readHoldingRegisters(pdu + 1, pduRet);
			break;

		case WRITE_HOLDING_REGISTER:
			ret = writeHoldingRegister(pdu + 1, pduRet);
			break;

		case WRITE_HOLDING_REGISTERS:
			ret = writeHoldingRegisters(pdu + 1, pduRet);
			break;

		default:
			ret = MODBUS_ERR_ILLEGAL_FUNCTION;
	}

	return ret;
}

uint8_t ModbusRTUSlave::readHoldingRegisters(uint8_t* pdu, uint8_t** pduRet) {
	if (!holdingRegisters) {
		return MODBUS_ERR_ILLEGAL_FUNCTION;
	}

	uint16_t addr = (uint16_t(pdu[0]) << 8) + pdu[1];
	uint16_t num = (uint16_t(pdu[2]) << 8) + pdu[3];

	if ((num > 125) || (addr + num > numHoldingRegisters)) {
		return MODBUS_ERR_ILLEGAL_ADDR;
	}

	uint8_t byteCount = num * 2;
	*pdu++ = byteCount;

	holdingRegistersMutex.take();
	uint16_t* registers = holdingRegisters + addr;
	while (num--) {
		*pdu++ = *registers >> 8;
		*pdu++ = *registers++;
	}
	holdingRegistersMutex.give();

	if (pduRet) {
		*pduRet = pdu;
	}

	return 0;
}

uint8_t ModbusRTUSlave::writeHoldingRegister(uint8_t* pdu, uint8_t** pduRet) {
	if (!holdingRegisters) {
		return MODBUS_ERR_ILLEGAL_FUNCTION;
	}

	uint16_t addr = (pdu[0] << 8) + pdu[1];
	if (addr > numHoldingRegisters) {
		return MODBUS_ERR_ILLEGAL_ADDR;
	}

	holdingRegistersMutex.take();
	*(holdingRegisters + addr) = (pdu[2] << 8) + pdu[3];
	holdingRegistersMutex.give();

	if (pduRet) {
		*pduRet = pdu;
	}

	return 0;
}

uint8_t ModbusRTUSlave::writeHoldingRegisters(uint8_t* pdu, uint8_t** pduRet) {
	if (!holdingRegisters) {
		return MODBUS_ERR_ILLEGAL_FUNCTION;
	}

	uint16_t addr = (pdu[0] << 8) + pdu[1];
	uint16_t num = (pdu[2] << 8) + pdu[3];

	if ((num > 125) || (addr + num > numHoldingRegisters)) {
		return MODBUS_ERR_ILLEGAL_ADDR;
	}

	uint8_t byteCount = pdu[4];
	if (num * 2 != byteCount) {
		return MODBUS_ERR_ILLEGAL_VALUE;
	}

	uint8_t* values = pdu + 5;
	holdingRegistersMutex.take();
	uint16_t* ptr = holdingRegisters + addr;
	while (num--) {
		*ptr++ = (values[0] << 8) + values[1];
		values += 2;
	}
	holdingRegistersMutex.give();

	// Keep the response equal to the first four request bytes
    // (send address and quantity only)
	pdu += 4;

	if (pduRet) {
		*pduRet = pdu;
	}

	return 0;
}
