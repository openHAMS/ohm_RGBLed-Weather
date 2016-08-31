#include "Color.hpp"
#include <Arduino.h>
#include <stdlib.h>

Color ColorParse(char* data)
{
	Color c;
	char r[3];
	char g[3];
	char b[3];
	char a[3];

	// data = #rrggbbaa/hex
	String s = String(data);
	s.substring(1, 3).toCharArray(r, 3);
	s.substring(3, 5).toCharArray(g, 3);
	s.substring(5, 7).toCharArray(b, 3);
	s.substring(7, 9).toCharArray(a, 3);

	c.R = strtol(r, NULL, 16);
	c.G = strtol(g, NULL, 16);
	c.B = strtol(b, NULL, 16);
	c.A = strtol(a, NULL, 16);

	return c;
}
