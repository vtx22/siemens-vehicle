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

void UART::sendGYROAngle(float angleX, float angleY)
{
	std::array<uint8_t, TX_BUF_SIZE> data;
	data[0] = _id;
	data[1] = 0x87;

	int32_t angleXInt = (int32_t)(angleX * 1000.f);
	int32_t angleYInt = (int32_t)(angleY * 1000.f);

	data[2] = (uint8_t)(angleXInt >> 24);
	data[3] = (uint8_t)(angleXInt >> 16);
	data[4] = (uint8_t)(angleXInt >> 8);
	data[5] = (uint8_t)(angleXInt);
	data[6] = (uint8_t)(angleYInt >> 24);
	data[7] = (uint8_t)(angleYInt >> 16);
	data[8] = (uint8_t)(angleYInt >> 8);
	data[9] = (uint8_t)(angleYInt);
	data[TX_BUF_SIZE - 1] = calculateChecksum(data);
	sendBytes(data);
}

void UART::sendGYROAccel(float accX, float accY, float accZ)
{
	std::array<uint8_t, TX_BUF_SIZE> data;
	data[0] = _id;
	data[1] = 0x88;

	uint16_t accelX = (uint16_t)(accX * 10);
	uint16_t accelY = (uint16_t)(accY * 10);
	uint16_t accelZ = (uint16_t)(accZ * 10);

	data[2] = (accelX >> 8);
	data[3] = accelX;
	data[4] = (accelY >> 8);
	data[5] = accelY;
	data[6] = (accelZ >> 8);
	data[7] = accelZ;
	data[TX_BUF_SIZE - 1] = calculateChecksum(data);
	sendBytes(data);
}

void UART::sendGYROVeloc(float velX, float velY, float velZ)
{
	std::array<uint8_t, TX_BUF_SIZE> data;
	data[0] = _id;
	data[1] = 0x89;

	uint16_t veloX = (uint16_t)(velX * 10);
	uint16_t veloY = (uint16_t)(velY * 10);
	uint16_t veloZ = (uint16_t)(velZ * 10);

	data[2] = (veloX >> 8);
	data[3] = veloX;
	data[4] = (veloY >> 8);
	data[5] = veloY;
	data[6] = (veloZ >> 8);
	data[7] = veloZ;
	data[TX_BUF_SIZE - 1] = calculateChecksum(data);
	sendBytes(data);
}

void UART::sendStatus(uint8_t code)
{
	std::array<uint8_t, TX_BUF_SIZE> data = statusMessage(code);
	sendBytes(data);
}
void UART::sendBATstatus()
{
	uint16_t voltage = (uint16_t)(_ina->getBusVol() * 1000.f);
	int16_t current = (int16_t)(_ina->getCurrent() * 1000.f);

	std::array<uint8_t, TX_BUF_SIZE> data = batteryStatus(voltage, current);
	sendBytes(data);
}

void UART::sendTEMPstatus(float outsideTemp, float hum)
{
	uint16_t pcbTemp = (uint16_t)(_tempPCB->readTemperature()* 100);
	uint16_t iotTemp = (uint16_t)(0 * 100);
	uint16_t outTemp = (uint16_t)(outsideTemp * 100);
	uint16_t humidity = (uint16_t)(hum * 10);


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
    bytearray.at(4) = (uint8_t)(current >> 8);
    bytearray.at(5) = (uint8_t)current;
    bytearray.at(TX_BUF_SIZE - 1) = calculateChecksum(bytearray);

    return bytearray;
};

