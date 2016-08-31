#ifndef COLOR_HPP
#define COLOR_HPP

#include <Arduino.h>

struct Color
{ byte R, G, B, A; };

Color ColorParse(char* data);

#endif /* COLOR_HPP */
