#pragma once
#include "stm32f1xx_hal.h"
#include <stdint.h>

class ADC
{
	public:
		ADC(ADC_HandleTypeDef* hadc);
		uint16_t getRAWValue(uint8_t channel);
		float getVoltage(uint8_t channel);

	private:
		ADC_HandleTypeDef* _hadc;

		//Stores the ratios of the voltage dividers used for voltage sensing
		//Order: Charger, BAT, ADJ, 24V, 5V
		const float _dividers[5] = {1.0, 1.0, 1.0, 1.0, 1.0};

};
