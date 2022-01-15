#pragma once
#include "stm32f1xx_hal.h"
#include <stdint.h>

class RAILCONTROL
{
	public:
		RAILCONTROL();

		void setADJRail(bool on);
		void set24VRail(bool on);
		void turnAllOFF();

		bool railADJStatus = false;
		bool rail24VStatus = false;

		float _adjSetpoint = 12.0;
	private:

};
