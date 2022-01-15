#pragma once
#include <stdint.h>
#include "stm32f1xx_hal.h"

class SERVO
{
	public:
		SERVO(TIM_HandleTypeDef* tim);
		void setAngle(float angle);
	private:
		uint32_t pwm2cycle(float pwm);
		void setDutyCycle(float duty);

		uint16_t _lowPeriod 	= 10;
		uint16_t _highPeriod 	= 10;

		TIM_HandleTypeDef* _tim;
		uint8_t _reload = 199;
		uint8_t _channel;
};

