#include "RGBLed.hpp"

#include <Arduino.h>
#include "Color.hpp"


// Member functions definitions including constructor
RGBLed::RGBLed(const byte REDPIN, const byte GREENPIN, const byte BLUEPIN)
{
    _color  = { 0, 0, 0, 0 };
    _pin.R = REDPIN;
    _pin.G = GREENPIN;
    _pin.B = BLUEPIN;
}
RGBLed::~RGBLed(void)
{ }

int RGBLed::Map(int input, int input_start, int input_end, int output_start, int output_end)
{
    int input_range = input_end - input_start;
    int output_range = output_end - output_start;
    return (input - input_start)*output_range / input_range + output_start;
}

int RGBLed::LedColor(const byte color, const byte alpha)
{
    return Map(color * alpha, 0, 255*255, 0, PWMRANGE);
}

void RGBLed::WriteColor(const Color color)
{
    analogWrite(_pin.R, LedColor(color.R, color.A));
    analogWrite(_pin.G, LedColor(color.G, color.A));
    analogWrite(_pin.B, LedColor(color.B, color.A));
}

int RGBLed::setColor(const Color c)
{
    _color = c;
    WriteColor(_color);
    return 1;
}
