#pragma once
#include "stm32f1xx_hal.h"
#include <stdint.h>
#include "Timing.hpp"

class DS18B20
{
	public:
		DS18B20(TIMER* tim, GPIO_TypeDef* port, uint16_t pin);
		float readTemperature();

		void _firstREQ();
		float _secREQ();
		float _lastTemp = 0.f;

	private:
		//PIN CONFIG AND SETTING
		void setDataPin(bool on);
		void toggleDataPin();

		void setPinOUTPUT();
		void setPinINPUT();

		//INTERFACING FUNCTIONS
		uint8_t startSensor();


		void writeData(uint8_t data);
		uint8_t readData();

		GPIO_TypeDef* _PORT;
		uint16_t _PIN;

		TIMER* _tim;

};
