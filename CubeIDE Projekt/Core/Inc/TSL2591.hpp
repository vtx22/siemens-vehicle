#pragma once
#include "stm32f1xx_hal.h"
#include <stdint.h>
#include "I2C.hpp"

//REGISTER ADDRESSES
const uint8_t ENABLE_REG 	= 0x00;	//Enables states and interrupts
const uint8_t CONFIG_REG 	= 0x01;	//ALS gain and integration time configuration
const uint8_t AILTL_REG 	= 0x04;	//ALS interrupt low threshold low byte
const uint8_t AILTH_REG 	= 0x05;	//ALS interrupt low threshold high byte
const uint8_t AIHTL_REG 	= 0x06;	//ALS interrupt high threshold low byte
const uint8_t AIHTH_REG 	= 0x07;	//ALS interrupt high threshold high byte
const uint8_t NPAILTL_REG 	= 0x08;	//No Persist ALS interrupt low threshold low byte
const uint8_t NPAILTH_REG 	= 0x09;	//No Persist ALS interrupt low threshold high byte
const uint8_t NPAIHTL_REG 	= 0x0A;	//No Persist ALS interrupt high threshold low byte
const uint8_t NPAIHTH_REG 	= 0x0B;	//No Persist ALS interrupt high threshold high byte
const uint8_t PERSIST_REG 	= 0x0C;	//Interrupt persistence filter
const uint8_t PID_REG 		= 0x11;	//Package ID
const uint8_t ID_REG 		= 0x12;	//Device ID
const uint8_t STATUS_REG 	= 0x13;	//Device Status
const uint8_t C0DATAL_REG 	= 0x14;	//CH0 ADC low data byte
const uint8_t C0DATAH_REG 	= 0x15;	//CH0 ADC high data byte
const uint8_t C1DATAL_REG 	= 0x16;	//CH1 ADC low data byte
const uint8_t C1DATAH_REG 	= 0x17;	//CH1 ADC high data byte



class TSL2591
{
	public:
		TSL2591(I2C_HandleTypeDef* hi2c);

		void enable();
		void config();
		uint32_t readALS();

	private:
		uint8_t readByte(uint8_t registerAddress);
		uint8_t _address = 0x29;				//I2C Address of the Sensor
		I2C_HandleTypeDef* _hi2c;				//Used I2C Bus
		I2C* _i2c;
};
