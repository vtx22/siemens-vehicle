#include "UART.hpp"


UART::UART(UART_HandleTypeDef* huart, INA226* ina, PRINTER* print, DS18B20* tempPCB)
{
	_huart = huart;
	_ina = ina;
	_print = print;
	_tempPCB = tempPCB;
}

void UART::startReceiveIT()
{
	HAL_UART_Receive_IT(_huart, RXBuffer, RX_BUF_SIZE);
}

void UART::parseMessage()
{
	if(calculateChecksum(RXBuffer) != RXBuffer[RX_BUF_SIZE - 1]) { return; }
	if(RXBuffer[0] != _id) { return; }

	switch(RXBuffer[1])		//Check Command ID
	{
		case 1:
			parseMSG1();
			break;
		case 8:
			break;
		default:
			break;
	}
}

void UART::parseMSG1()
{
	if(RXBuffer[2])
	{
		_globalEnable = true;
	}
	else
	{
		_globalEnable = false;
	}
}


uint8_t UART::calculateChecksum(uint8_t* msg)
{
	uint8_t cs = 0x00;
	for(uint8_t i = 0; i < RX_BUF_SIZE - 1; i++)
	{
		cs ^= msg[i];
	}

	return cs;
}

void UART::sendStatus(uint8_t code)
{
	std::array<uint8_t, TX_BUF_SIZE> data = statusMessage(code);
	sendBytes(data);
}
void UART::sendBATstatus()
{
	uint16_t voltage = (uint16_t)(_ina->getBusVol() * 1000);
	int16_t current = (int16_t)(_ina->getCurrent() * 1000);

	std::array<uint8_t, TX_BUF_SIZE> data = batteryStatus(voltage, current);
	sendBytes(data);
}

void UART::sendTEMPstatus()
{
	uint16_t pcbTemp = (uint16_t)(_tempPCB->readTemperature() * 100);
	uint16_t iotTemp = (uint16_t)(0 * 100);
	uint16_t outTemp = (uint16_t)(0 * 100);
	uint16_t humidity = (uint16_t)(0 * 10);


	std::array<uint8_t, TX_BUF_SIZE> data = tempStatus(pcbTemp, iotTemp, outTemp, humidity);
	sendBytes(data);
}


std::array<uint8_t, TX_BUF_SIZE> UART::statusMessage(uint8_t codes)
{
    const uint8_t CMD_ID = 0x01;
    std::array<uint8_t, TX_BUF_SIZE> bytearray = {0};

    bytearray.at(0) = _id;
    bytearray.at(1) = CMD_ID;
    bytearray.at(2) = codes;
    bytearray.at(TX_BUF_SIZE - 1) = calculateChecksum(bytearray);

    return bytearray;
};

std::array<uint8_t, TX_BUF_SIZE> UART::tempStatus(uint16_t tempPCB, uint16_t tempIOT, uint16_t tempOUT, uint16_t humidity)
{
    const uint8_t CMD_ID = 0x86;
    std::array<uint8_t, TX_BUF_SIZE> bytearray = {0};

    bytearray.at(0) = _id;
    bytearray.at(1) = CMD_ID;
    bytearray.at(2) = tempPCB >> 8;
    bytearray.at(3) = tempPCB;
    bytearray.at(4) = tempIOT >> 8;
    bytearray.at(5) = tempIOT;
    bytearray.at(6) = tempOUT >> 8;
    bytearray.at(7) = tempOUT;
    bytearray.at(8) = humidity >> 8;
    bytearray.at(9) = humidity;
    bytearray.at(TX_BUF_SIZE - 1) = calculateChecksum(bytearray);

    return bytearray;
};


std::array<uint8_t, TX_BUF_SIZE> UART::batteryStatus(uint16_t voltage, int16_t current)
{
    const uint8_t CMD_ID = 0x85;
    std::array<uint8_t, TX_BUF_SIZE> bytearray = {0};

    bytearray.at(0) = _id;
    bytearray.at(1) = CMD_ID;
    bytearray.at(2) = voltage >> 8;
    bytearray.at(3) = voltage;
    bytearray.at(4) = current >> 8;
    bytearray.at(5) = current;
    bytearray.at(TX_BUF_SIZE - 1) = calculateChecksum(bytearray);

    return bytearray;
};

