#pragma once
#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <string>
#include <math.h>

#include "ADC.hpp"
#include "Printer.hpp"
#include "Railcontrol.hpp"
#include "INA226.hpp"

class WATCHDOG
{
	public:
		WATCHDOG(ADC* adc, PRINTER* printer, RAILCONTROL* rail, INA226* ina);

		void faultHandler(uint16_t errorID);
		void fullSelftest();
		bool checkAllVoltages();	//Returns TRUE if a fault is detected
		bool checkBatteryData();


		void setVoltageTolerance(uint8_t percentage);
		void setADJSetpoint(float voltage);


	private:
		ADC* _adc;
		PRINTER* _print;
		RAILCONTROL* _rail;
		INA226* _ina;

		uint8_t _voltageTolerance = 5;
		float _maxCurrent = 8.0;
};
