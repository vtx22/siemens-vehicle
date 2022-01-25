#include "DS18B20.hpp"

DS18B20::DS18B20(TIMER* tim, GPIO_TypeDef* port, uint16_t pin)
{
	_tim = tim;
	_PORT = port;
	_PIN = pin;
}

void DS18B20::setDataPin(bool on)
{
	if(on)
	{
		HAL_GPIO_WritePin(_PORT, _PIN, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(_PORT, _PIN, GPIO_PIN_RESET);
	}
}

void DS18B20::toggleDataPin()
{
	HAL_GPIO_TogglePin(_PORT, _PIN);
}

void DS18B20::setPinOUTPUT()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = _PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(_PORT, &GPIO_InitStruct);
}

void DS18B20::setPinINPUT()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = _PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(_PORT, &GPIO_InitStruct);
}

uint8_t DS18B20::startSensor()
{
	uint8_t response = 0;

	setPinOUTPUT();
	setDataPin(false);

	_tim->delayUS(480);
	setPinINPUT();
	_tim->delayUS(80);
	if(!HAL_GPIO_ReadPin(_PORT, _PIN))
	{
		response = 1;
	}
	else
	{
		response = -1;
	}
	_tim->delayUS(400);

	return response;
}

void DS18B20::writeData(uint8_t data)
{
	setPinOUTPUT();  // set as output

	for (uint8_t i=0; i<8; i++)
	{

		if ((data & (1<<i))!=0)  // if the bit is high
		{
			// write 1

			setPinOUTPUT();  // set as output
			setDataPin(false);  // pull the pin LOW
			_tim->delayUS(1);  // wait for 1 us

			setPinINPUT();  // set as input
			_tim->delayUS(60);  // wait for 60 us
		}

		else  // if the bit is low
		{
			// write 0

			setPinOUTPUT();
			setDataPin(false);  // pull the pin LOW
			_tim->delayUS(60);  // wait for 60 us

			setPinINPUT();
		}
	}
}


uint8_t DS18B20::readData()
{
	uint8_t value=0;
	setPinINPUT();

	for (uint8_t i=0;i<8;i++)
	{
		setPinOUTPUT();   // set as output

		setDataPin(false);  // pull the data pin LOW
		_tim->delayUS(2);  // wait for 2 us

		setPinINPUT();  // set as input
		if(HAL_GPIO_ReadPin (_PORT, _PIN))  // if the pin is HIGH
		{
			value |= 1<<i;  // read = 1
		}
		_tim->delayUS(60);  // wait for 60 us
	}
	return value;
}

float DS18B20::readTemperature()
{
	_firstREQ();

	HAL_Delay(800);

	return _secREQ();
}

void DS18B20::_firstREQ()
{
	startSensor();
	HAL_Delay(1);
	writeData(0xCC);
	writeData(0x44);
}

float DS18B20::_secREQ()
{
	startSensor();
	writeData(0xCC);
	writeData(0xBE);

	uint8_t temp1 = readData();
	uint8_t temp2 = readData();

	uint16_t tempCom = (temp2<<8)|temp1;

	return (float)(tempCom/16.0);
}
