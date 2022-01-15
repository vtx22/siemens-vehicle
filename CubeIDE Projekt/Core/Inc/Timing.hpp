#pragma once
#include <stdint.h>
#include "stm32f1xx_hal.h"

class TIMER
{
	public:
		TIMER(TIM_HandleTypeDef* tim);
		void delayUS(uint16_t us);
		TIM_HandleTypeDef* _tim;
	private:
		//TIM_HandleTypeDef* _tim;
};
