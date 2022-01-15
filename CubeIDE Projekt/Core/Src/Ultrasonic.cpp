#include "Ultrasonic.hpp"

ULTRA::ULTRA(GPIO_TypeDef* portTRIG, uint16_t pinTRIG, GPIO_TypeDef* portECHO, uint16_t pinECHO, TIMER* tim)
{
	_portTRIG = portTRIG;
	_pinTRIG = pinTRIG;
	_portECHO = portECHO;
	_pinECHO = pinECHO;
	_tim = tim;

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = _pinTRIG;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(_portTRIG, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = _pinECHO;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(_portECHO, &GPIO_InitStruct);

}

void ULTRA::sendPulse()
{
	HAL_GPIO_WritePin(_portTRIG, _pinTRIG, GPIO_PIN_SET);
	_tim->delayUS(10);
	HAL_GPIO_WritePin(_portTRIG, _pinTRIG, GPIO_PIN_RESET);
}

float ULTRA::counts2cm(uint16_t counts)
{
	return (_speedOfSound * counts / 1000.0) / 2.0;
}

float ULTRA::getDistanceCM()
{
	//__disable_irq();
	uint16_t counts = 0;

	_tim->_tim->Instance->ARR = 50000;
	sendPulse();

	while((_portECHO->IDR & (1 << _pinECHO)) == 0);		//Wait till the ECHO-Pin pulls HIGH
	_tim->_tim->Instance->CNT = 0;						//Reset Counter
	while((_portECHO->IDR & (1 << _pinECHO)) > 0);		//Wait till the ECHO-Pin pulls LOW
	counts = _tim->_tim->Instance->CNT;					//Save counts (one count per microsecond)

	return (_speedOfSound * counts / 1000.0) / 2.0;
}

