#include "TSL2591.hpp"

TSL2591::TSL2591(I2C_HandleTypeDef* hi2c)
{
	_hi2c = hi2c;
	_i2c = new I2C(_hi2c);

	enable();
	config();
}

uint8_t TSL2591::readByte(uint8_t registerAddress)
{
	return _i2c->readOneByte(_address, registerAddress);
}

void TSL2591::enable()
{
	_i2c->writeByte(_address, ENABLE_REG, 0b11);
}

void TSL2591::config()
{
	_i2c->writeByte(_address, CONFIG_REG, 0b00010000);
}

uint32_t TSL2591::readALS()
{
	uint16_t c0 = (_i2c->readOneByte(_address, C0DATAH_REG) << 8) + _i2c->readOneByte(_address, C0DATAL_REG);
	uint16_t c1 = (_i2c->readOneByte(_address, C1DATAH_REG) << 8) + _i2c->readOneByte(_address, C1DATAL_REG);

	return ((c1 << 16) + c0);
}
