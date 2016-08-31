#ifndef RGBLED_HPP
#define RGBLED_HPP

#include <Arduino.h>
#include "Color.hpp"

class RGBLed
{
	public:
		RGBLed(const byte REDPIN, const byte GREENPIN, const byte BLUEPIN);
		~RGBLed();
		int setColor(Color color);
	private:
		struct RGBLedPin { byte R, G, B; };
		Color _color;
		RGBLedPin _pin;
		int Map(int input, int input_start, int input_end, int output_start, int output_end);
		int LedColor(const byte color, const byte alpha);
		void WriteColor(const Color C);
};

#endif /* RGBLED_HPP */
