#include "Servo.hpp"

SERVO::SERVO(TIM_HandleTypeDef* tim) : _tim(tim)
{
	HAL_TIM_PWM_Start(_tim, TIM_CHANNEL_1);

}

void SERVO::setDutyCycle(float duty)
{
	__HAL_TIM_SET_COMPARE(_tim, TIM_CHANNEL_1, pwm2cycle(duty));
}

void SERVO::setAngle(float angle)
{
	setDutyCycle(angle * 100 / 180.0);
}

uint32_t SERVO::pwm2cycle(float pwm)
{
	return (uint32_t)(_lowPeriod + _highPeriod * pwm / 100.0);
}
