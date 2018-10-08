#pragma once
#include <math.h>

class MathU
{
public:
	static constexpr float Pi = 3.14159265358979323846f;
	static constexpr float Infinity = FLT_MAX;
	static constexpr float NegativeInfinity = FLT_MIN;
	static constexpr int32 IntMax = INT32_MAX;
	static constexpr float SmallNumber = 0.00001f;

	static inline float ToRadians(float degrees)
	{
		return (Pi / 180.0f) *  degrees;
	}

	static inline int RoundUp(int numToRound, int multiple)
	{
		if (multiple == 0)
			return numToRound;

		int remainder = abs(numToRound) % multiple;
		if (remainder == 0)
			return numToRound;

		if (numToRound < 0)
			return -(abs(numToRound) - remainder);
		else
			return numToRound + multiple - remainder;
	}

	static inline float Max(float a, float b)
	{
		return (a > b) ? a : b;
	}

	static inline float Min(float a, float b)
	{
		return (a < b) ? a : b;
	}

	static inline float Sign(float x)
	{
		return (x >= 0.0f) ? 1.0f : -1.0f;
	}

	static inline bool IsBetweenInclusive(float bound1, float bound2, float testVal)
	{

	}
};
