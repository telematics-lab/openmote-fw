#ifndef __MODBUS_H__
#define __MODBUS_H__

#include <Mutex.hpp>
#include <Rs485.hpp>

#define MODBUS_PDU_HEADER_SIZE		1 // FC (1)
#define MODBUS_PDU_DATA_SIZE		252 // Data bytes
#define MODBUS_PDU_FOOTER_SIZE		0
#define MODBUS_PDU_SIZE				MODBUS_PDU_HEADER_SIZE + MODBUS_PDU_DATA_SIZE + MODBUS_PDU_FOOTER_SIZE

#define MODBUS_RTU_ADU_HEADER_SIZE	1 // SLAVE ADDR (1)
#define MODBUS_RTU_ADU_FOOTER_SIZE	2 // CRC (2)
#define MODBUS_RTU_ADU_SIZE			MODBUS_RTU_ADU_HEADER_SIZE + MODBUS_PDU_SIZE + MODBUS_RTU_ADU_FOOTER_SIZE

#define MODBUS_RTU_BYTE_TIMEOUT		5 // Byte reception timeout in ms

#define MODBUS_ERR_ILLEGAL_FUNCTION	1
#define MODBUS_ERR_ILLEGAL_ADDR		2
#define MODBUS_ERR_ILLEGAL_VALUE	3

class Modbus {
	public:
		static uint16_t crc(const uint8_t* buff, size_t len);
};

class ModbusRTUMaster {
	public:
		explicit ModbusRTUMaster(Rs485& rs485, uint32_t responseTimeout = 1000);

	public:
		bool readHoldingRegister(uint8_t slave, uint16_t addr, uint16_t* value);
		bool readHoldingRegisters(uint8_t slave, uint16_t addr, uint16_t* values, uint16_t num);

		bool writeHoldingRegister(uint8_t slave, uint16_t addr, uint16_t value);
		bool writeHoldingRegisters(uint8_t slave, uint16_t addr, uint16_t* values, uint16_t num);

	private:
		bool isValidSlave(uint8_t slave);
		uint8_t* preparePdu(uint8_t slave, uint8_t fc);
		void discardPendingBytes();
		uint8_t* receiveResponse();

	private:
		Rs485& rs485;
		uint8_t adu[MODBUS_RTU_ADU_SIZE];
		uint32_t responseTimeout;
};

class ModbusRTUSlave {
	public:
		explicit ModbusRTUSlave(Rs485& rs485, uint8_t slaveAddr);

	public:
		void setHoldingRegisters(uint16_t* holdingRegisters, uint16_t numHoldingRegisters);

		void update();

		void setHoldingRegister(uint16_t addr, uint16_t value);
		uint16_t getHoldingRegister(uint16_t addr);

	private:
		uint8_t processRequest(uint8_t* pdu, uint8_t** pduRet);

		uint8_t readHoldingRegisters(uint8_t* pdu, uint8_t** pduRet);
		uint8_t writeHoldingRegister(uint8_t* pdu, uint8_t** pduRet);
		uint8_t writeHoldingRegisters(uint8_t* pdu, uint8_t** pduRet);

	private:
		Rs485& rs485;
		uint8_t slaveAddr;

		uint8_t adu[MODBUS_RTU_ADU_SIZE];

		Mutex holdingRegistersMutex;
		uint16_t* holdingRegisters;
		uint16_t numHoldingRegisters;
};

#endif
