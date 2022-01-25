#pragma once
#include <stdint.h>
#include "stm32f1xx_hal.h"
#include <math.h>

enum LED_MODE
{
	LED_ALL_OFF = 0x00,
	LED_BLINK_LEFT = 0x01,
	LED_BLINK_RIGHT = 0x02,
	LED_BLINK_BOTH = 0x03,
	LED_TOGGLE_FRONT = 0x04,
	LED_TOGLLE_BACK = 0x05,
	LED_CIRClE_LIGHT = 0x06,
	LED_IDLE = 0x07,

}
typedef LED_MODE;

class LED
{
	public:
		LED();
		LED(TIM_HandleTypeDef* tim);
		void setSTAT1(bool on);
		void setSTAT2(bool on);
		void toggleSTAT(uint8_t stat);

		void updateNeopixels();

		void animateSine(uint8_t* baseColor, float speed, float scaling, float dimming);
		void animateRainbow(float speed, float dimming);


		void setNeopixel(uint8_t* rgb, uint8_t led);
		void setNeopixel(uint8_t red, uint8_t green, uint8_t blue, uint8_t led);

		void setAllNeopixels(uint8_t* rgb);
		void setAllNeopixels(uint8_t red, uint8_t green, uint8_t blue);

		uint32_t _updateCNT = 0;
		LED_MODE _mode = LED_IDLE;
		LED_MODE _prevMode = LED_IDLE;

		uint8_t frontColor[3] = {255,255,255};
		uint8_t backColor[3] = {255,0,0};
		uint8_t blinkColor[3] = {255,100,0};

	private:
		void setNEOPin(bool on);
		void toggleNEOPin();
		void setNeopixelNOUP(uint8_t* rgb, uint8_t led);
		void setNeopixelNOUP(uint8_t red, uint8_t green, uint8_t blue, uint8_t led);
		void increaseColor(uint8_t id, uint8_t step, uint8_t led);
		void decreaseColor(uint8_t id, uint8_t step, uint8_t led);

		static const uint8_t _neoLEDS = 33;

		//0  - 20 = front light
		//21 - 26 = back light 1
		//27 - 32 = back light 2

		uint8_t ledBufferRED[_neoLEDS];
		uint8_t ledBufferGREEN[_neoLEDS];
		uint8_t ledBufferBLUE[_neoLEDS];
		TIM_HandleTypeDef* _tim;
};


