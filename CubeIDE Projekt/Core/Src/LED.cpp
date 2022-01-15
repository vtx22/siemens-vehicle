#include "LED.hpp"


RgbColor LED::HsvToRgb(HsvColor hsv)
{
    RgbColor rgb;
    unsigned char region, remainder, p, q, t;

    if (hsv.s == 0)
    {
        rgb.r = hsv.v;
        rgb.g = hsv.v;
        rgb.b = hsv.v;
        return rgb;
    }

    region = hsv.h / 43;
    remainder = (hsv.h - (region * 43)) * 6;

    p = (hsv.v * (255 - hsv.s)) >> 8;
    q = (hsv.v * (255 - ((hsv.s * remainder) >> 8))) >> 8;
    t = (hsv.v * (255 - ((hsv.s * (255 - remainder)) >> 8))) >> 8;

    switch (region)
    {
        case 0:
            rgb.r = hsv.v; rgb.g = t; rgb.b = p;
            break;
        case 1:
            rgb.r = q; rgb.g = hsv.v; rgb.b = p;
            break;
        case 2:
            rgb.r = p; rgb.g = hsv.v; rgb.b = t;
            break;
        case 3:
            rgb.r = p; rgb.g = q; rgb.b = hsv.v;
            break;
        case 4:
            rgb.r = t; rgb.g = p; rgb.b = hsv.v;
            break;
        default:
            rgb.r = hsv.v; rgb.g = p; rgb.b = q;
            break;
    }

    return rgb;
}

HsvColor LED::RgbToHsv(RgbColor rgb)
{
    HsvColor hsv;
    unsigned char rgbMin, rgbMax;

    rgbMin = rgb.r < rgb.g ? (rgb.r < rgb.b ? rgb.r : rgb.b) : (rgb.g < rgb.b ? rgb.g : rgb.b);
    rgbMax = rgb.r > rgb.g ? (rgb.r > rgb.b ? rgb.r : rgb.b) : (rgb.g > rgb.b ? rgb.g : rgb.b);

    hsv.v = rgbMax;
    if (hsv.v == 0)
    {
        hsv.h = 0;
        hsv.s = 0;
        return hsv;
    }

    hsv.s = 255 * long(rgbMax - rgbMin) / hsv.v;
    if (hsv.s == 0)
    {
        hsv.h = 0;
        return hsv;
    }

    if (rgbMax == rgb.r)
        hsv.h = 0 + 43 * (rgb.g - rgb.b) / (rgbMax - rgbMin);
    else if (rgbMax == rgb.g)
        hsv.h = 85 + 43 * (rgb.b - rgb.r) / (rgbMax - rgbMin);
    else
        hsv.h = 171 + 43 * (rgb.r - rgb.g) / (rgbMax - rgbMin);

    return hsv;
}


LED::LED()
{
	setAllNeopixels(0x00, 0x00, 0x00);
}

LED::LED(TIM_HandleTypeDef* tim)
{
	_tim = tim;
	HAL_TIM_Base_Start(_tim);
	setAllNeopixels(0x00, 0x00, 0x00);
}

void LED::setSTAT1(bool on)
{
	if(on)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
	}
}

void LED::setSTAT2(bool on)
{
	if(on)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	}
}


void LED::toggleSTAT(uint8_t stat)
{
	switch(stat)
	{
		case 1:
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);
			break;
		case 2:
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
			break;
		default:
			break;
	}
}

void LED::setNEOPin(bool on)
{
	if(on)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	}
}

void LED::toggleNEOPin()
{
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
}


void LED::animateSine(uint8_t* baseColor, float speed, float scaling, float dimming)
{
	double shift = _updateCNT * speed;

	for(uint8_t leds = 0; leds < _neoLEDS; leds++)
	{
		uint8_t red 	= (baseColor[0] == 0) ? 0 : (baseColor[0] + sin(leds + shift) * scaling * baseColor[0] / 255) * dimming;
		uint8_t green 	= (baseColor[1] == 0) ? 0 : (baseColor[1] + sin(leds + shift) * scaling * baseColor[1] / 255) * dimming;
		uint8_t blue 	= (baseColor[2] == 0) ? 0 : (baseColor[2] + sin(leds + shift) * scaling * baseColor[2] / 255) * dimming;

		setNeopixelNOUP(red, green, blue, leds);
	}
	updateNeopixels();
}


uint8_t hue = 0;
void LED::animateRainbow(float speed, float dimming)
{
	uint8_t step = 1;
	uint8_t scaling = 1;

	for(uint8_t ledX = 0; ledX < _neoLEDS; ledX++)
	{
			HsvColor color = { hue + ledX * scaling, 255, 255 };
			RgbColor rgb = HsvToRgb(color);
			setNeopixelNOUP(rgb.r, rgb.g, rgb.b, ledX);
	}

	if(hue <= 255 - step)
	{
		hue += step;
	}
	else
	{
		hue = 0;
	}

	updateNeopixels();
}

void LED::increaseColor(uint8_t id, uint8_t step, uint8_t led)
{
	switch(id)
	{
	case 1:
		setNeopixelNOUP(ledBufferRED[led] += step, ledBufferGREEN[led], ledBufferBLUE[led], led);
		break;
	case 2:
		setNeopixelNOUP(ledBufferRED[led], ledBufferGREEN[led] += step, ledBufferBLUE[led], led);
		break;
	case 3:
		setNeopixelNOUP(ledBufferRED[led], ledBufferGREEN[led], ledBufferBLUE[led] += step, led);
		break;
	default:
		break;
	}

}

void LED::decreaseColor(uint8_t id, uint8_t step, uint8_t led)
{
	switch(id)
	{
	case 1:
		setNeopixelNOUP(ledBufferRED[led] -= step, ledBufferGREEN[led], ledBufferBLUE[led], led);
		break;
	case 2:
		setNeopixelNOUP(ledBufferRED[led], ledBufferGREEN[led] -= step, ledBufferBLUE[led], led);
		break;
	case 3:
		setNeopixelNOUP(ledBufferRED[led], ledBufferGREEN[led], ledBufferBLUE[led] -= step, led);
		break;
	default:
		break;
	}
}

//Sets all LEDs to the same color of rgb[3] = {red, green, blue}
void LED::setAllNeopixels(uint8_t* rgb)
{
	for(uint8_t led = 0; led < _neoLEDS; led++)
	{
		setNeopixelNOUP(rgb, led);
	}

	updateNeopixels();
}

//Sets all LEDs to the same color of red, green, blue
void LED::setAllNeopixels(uint8_t red, uint8_t green, uint8_t blue)
{
	for(uint8_t led = 0; led < _neoLEDS; led++)
	{
		setNeopixelNOUP(red, green, blue, led);
	}

	updateNeopixels();
}

//Sets one specific LED to the color of rgb[3] = {red, green blue}
void LED::setNeopixel(uint8_t* rgb, uint8_t led)
{
	ledBufferRED[led] = rgb[0];
	ledBufferGREEN[led] = rgb[1];
	ledBufferBLUE[led] = rgb[2];

	updateNeopixels();
}

//Sets one specific LED to the color of red, green, blue
void LED::setNeopixel(uint8_t red, uint8_t green, uint8_t blue, uint8_t led)
{
	ledBufferRED[led] = red;
	ledBufferGREEN[led] = green;
	ledBufferBLUE[led] = blue;

	updateNeopixels();
}

//Sets one specific LED to the color of rgb[3] = {red, green, blue} by writing to the buffer without instant refreshing, useful for loops
void LED::setNeopixelNOUP(uint8_t* rgb, uint8_t led)
{
	ledBufferRED[led] = rgb[0];
	ledBufferGREEN[led] = rgb[1];
	ledBufferBLUE[led] = rgb[2];
}

//Sets one specific LED to the color of red, green, blue by writing to the buffer without instant refreshing, useful for loops
void LED::setNeopixelNOUP(uint8_t red, uint8_t green, uint8_t blue, uint8_t led)
{
	ledBufferRED[led] = red;
	ledBufferGREEN[led] = green;
	ledBufferBLUE[led] = blue;
}

//Updates all LEDs by sending the command string to the LED strip
void LED::updateNeopixels()
{
	setNEOPin(false);

	uint32_t oldLOAD = SysTick->LOAD;	//Save old SysTick values for restoring
	uint32_t oldVAL = SysTick->VAL;


	uint32_t clock = HAL_RCC_GetSysClockFreq();				//72MHz Clock Frequency

	uint8_t bitPeriod 	= clock / 800000;					//1.25us 	= period / bit
	uint8_t t0 			= bitPeriod - (clock / 2500000);	//0.4us		= short pulse period
	uint8_t t1			= bitPeriod - (clock / 1250000);	//0.8us		= long pulse period  (better to calculate than 0.9us)



	SysTick->LOAD = bitPeriod - 1;		//Rollover is the full bit period with approx 1.25us
	SysTick->VAL = 0;					//Start counting at 0 (SysTick counts downwards)



	__disable_irq();										//Disable Interrupts because timings have to be accurate

	for(uint8_t led = 0; led < _neoLEDS; led++)				//Loop through every LED
	{
		uint8_t mask = 0x80;
		uint8_t r = ledBufferRED[led];						//Cache colors
		uint8_t g = ledBufferGREEN[led];					//Cache colors
		uint8_t b = ledBufferBLUE[led];						//Cache colors

		SysTick->VAL = 0;									//Reset SysTick Counter

		for(uint8_t bit = 0; bit < 8; bit++)				//Loop for GREEN color
		{
			uint8_t waittime = ((g & mask) ? t1 : t0);		//HIGH-Time based on 1 or 0

			GPIOA->ODR |= (1 << 8);				//Pin HIGH

			while(SysTick->VAL > waittime);					//Wait for HIGH-Time

			GPIOA->ODR &= ~((1 << 8));			//Pin LOW

			while(SysTick->VAL <= waittime);				//Wait remaining LOW-Time

			SysTick->VAL = 0;								//Reset SysTick Counter

			mask = (mask >> 1);								//Shift Mask to next bit
		}

		mask = 0x80;										//Reset Mask back to first bit
		SysTick->VAL = 0;									//Reset SysTick Counter

		for(uint8_t bit = 0; bit < 8; bit++)				//Loop for RED color
		{
			uint8_t waittime = ((r & mask) ? t1 : t0);		//HIGH-Time based on 1 or 0

			GPIOA->ODR |= (1 << 8);				//Pin HIGH

			while(SysTick->VAL > waittime);					//Wait for HIGH-Time

			GPIOA->ODR &= ~((1 << 8));			//Pin LOW

			while(SysTick->VAL <= waittime);				//Wait remaining LOW-Time

			SysTick->VAL = 0;								//Reset SysTick Counter

			mask = (mask >> 1);								//Shift Mask to next bit
		}

		mask = 0x80;										//Reset Mask back to first bit
		SysTick->VAL = 0;									//Reset SysTick Counter

		for(uint8_t bit = 0; bit < 8; bit++)				//Loop for BLUE color
		{
			uint8_t waittime = ((b & mask) ? t1 : t0);		//HIGH-Time based on 1 or 0

			GPIOA->ODR |= (1 << 8);				//Pin HIGH

			while(SysTick->VAL > waittime);					//Wait for HIGH-Time

			GPIOA->ODR &= ~((1 << 8));			//Pin LOW

			while(SysTick->VAL <= waittime);				//Wait remaining LOW-Time

			SysTick->VAL = 0;								//Reset SysTick Counter

			mask = (mask >> 1);								//Shift Mask to next bit
		}

	}

	for(uint8_t cnt = 0; cnt < 30; cnt++)					//Delay for a few dozent microseconds for the LED Reset
	{
		SysTick->VAL = 0;
		while((SysTick->VAL) >= 50);
	}


	SysTick->LOAD = oldLOAD;	//Restore old SysTick values
	SysTick->VAL = oldVAL;

	__enable_irq();				//Enable Interrupts
}



