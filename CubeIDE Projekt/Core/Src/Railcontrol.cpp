#include "Railcontrol.hpp"

RAILCONTROL::RAILCONTROL()
{

}

void RAILCONTROL::setADJRail(bool on)		//Toggles the ADJ Rail
{
	if(on)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	}
	railADJStatus = on;
}

void RAILCONTROL::set24VRail(bool on)		//Toggles the 24V Rail
{
	if(on)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

	}
	else
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
	}
	rail24VStatus = on;
}

void RAILCONTROL::turnAllOFF()
{
	setADJRail(0);
	set24VRail(0);
}
