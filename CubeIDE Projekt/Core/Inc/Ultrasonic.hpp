#pragma once
#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "Timing.hpp"

class ULTRA
{
	public:
		ULTRA(GPIO_TypeDef* portTRIG, uint16_t pinTRIG, GPIO_TypeDef* portECHO, uint16_t pinECHO, TIMER* tim);

		float getDistanceCM();
		float counts2cm(uint16_t counts);
		void sendPulse();

	private:

		GPIO_TypeDef* _portTRIG;
		uint16_t _pinTRIG;
		GPIO_TypeDef* _portECHO;
		uint16_t _pinECHO;

		TIMER* _tim;

		uint16_t _speedOfSound = 34.3;	//in cm/ms
};
