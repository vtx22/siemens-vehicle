#include "Watchdog.hpp"

WATCHDOG::WATCHDOG(ADC* adc, PRINTER* printer, RAILCONTROL* rail, INA226* ina)
{
	_adc = adc;
	_print = printer;
	_rail = rail;
	_ina = ina;
}


void WATCHDOG::faultHandler(uint16_t errorID)
{
	if(errorID < 100)			//All errors with ID<100 are considered non recoverable, thus the MCU stops executing the main
	{
		__disable_irq();		//No more Interrupts

		_rail->turnAllOFF();	//Disable the ADJ and the 24V Voltage Rail

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);	//Turn STAT1 LED ON for error indication

		std::string message;

		switch(errorID)											//Create Error Message basend on the error ID
		{
			case 1:
				message = "SELFTEST ERROR: VOLTAGE EXCEEDED LIMIT!";
				break;
			case 2:
				message = "SELFTEST ERROR: CURRENT EXCEEDED LIMIT!";
				break;
			default:
				message = "UNKNOWN HARDFAULT!";
				break;
		}

		while(true)												//Message gets send every 5 Seconds in an infinite loop
		{
			_print->printString(message);
			HAL_Delay(5000);
		}
	}
}


void WATCHDOG::fullSelftest()
{
	if(checkAllVoltages())
	{
		faultHandler(1);
	}
	if(checkBatteryData())
	{
		faultHandler(2);
	}
}

bool WATCHDOG::checkBatteryData()
{
	if(_ina->getCurrent() > _maxCurrent)
	{
		return true;
	}

	return false;

}


bool WATCHDOG::checkAllVoltages()
{

	float v[6];
	v[0] = _adc->getVoltage(0); 	//Charger
	v[1] = _adc->getVoltage(1); 	//BAT
	v[2] = _adc->getVoltage(2); 	//ADJ
	v[3] = _adc->getVoltage(3); 	//24
	v[4] = _adc->getVoltage(4); 	//5V
	v[5] = _ina->getBusVol();	//INA BAT Measurement

	if(!((v[0] < 24.5 && v[0] > 22.0) || v[0] < 0.1))	//Check Charger Voltage
	{
		return true;
	}

	if(!((v[1] < 22.0 && v[1] > 11.0) && abs(v[1] - v[5]) < 0.4))	//Check BAT Voltage and compare with INA measurement
	{
		return true;
	}

	if((_rail->railADJStatus && abs(v[2] - _rail->_adjSetpoint) < _rail->_adjSetpoint * _voltageTolerance / 10.0) || (!_rail->railADJStatus && v[2] > 0.1))
	{
		return true;
	}

	if((_rail->rail24VStatus && (v[3] < 23.5 || v[3] > 24.5)) || (!_rail->rail24VStatus && v[3] > 0.1))
	{
		return true;
	}

	if(v[5] > 5.5 || v[5] < 4.6)
	{
		return true;
	}

	return false;
}
