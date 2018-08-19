#pragma once
#include <math.h>

class MathUtils
{
public:
	static constexpr float Pi = 3.14159265358979323846f;

	static inline float ToRadians(float degrees)
	{
		return (Pi / 180.0f) *  degrees;
	}
};
