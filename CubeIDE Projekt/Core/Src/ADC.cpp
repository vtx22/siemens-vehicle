#include "ADC.hpp"

ADC::ADC(ADC_HandleTypeDef* adc)
{
	_hadc = adc;
}

uint16_t ADC::getRAWValue(uint8_t channel)
{
	ADC_ChannelConfTypeDef sConfig = {0};

	sConfig.Channel = channel;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if (HAL_ADC_ConfigChannel(_hadc, &sConfig) != HAL_OK)
	{

	}
	HAL_ADC_Start(_hadc);
	uint16_t value = HAL_ADC_GetValue(_hadc);
	//HAL_ADC_Stop(_hadc);
	return value;
}

float ADC::getVoltage(uint8_t channel)
{
	uint16_t raw = getRAWValue(channel);

	if(channel < 4)
	{
		return raw * 3.3 / 4095.0 * 1.0 / _dividers[channel];
	}
	else
	{
		return raw * 3.3 / 4095.0 * 1.0 / _dividers[4];
	}
}
