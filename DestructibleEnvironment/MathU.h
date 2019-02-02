#pragma once

#include <math.h>
#include <limits>

class MathU
{
public:
	static constexpr float Pi = 3.14159265358979323846f;
	static constexpr float Infinity = (std::numeric_limits<float>::max)();
	static constexpr float NegativeInfinity = std::numeric_limits<float>::lowest();
	static constexpr int32 IntMax = (std::numeric_limits<int32>::max)();
	static constexpr float Epsilon = std::numeric_limits<float>::epsilon();

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

	template<class T>
	static inline T Max(T a, T b)
	{
		return (a > b) ? a : b;
	}

	template<class T>
	static inline T Min(T a, T b)
	{
		return (a < b) ? a : b;
	}

	static inline float Sign(float x)
	{
		return (x >= 0.0f) ? 1.0f : -1.0f;
	}

	static inline bool IsBetweenInclusive(float bound1, float bound2, float testVal)
	{
		if (bound1 > bound2)
			return testVal >= bound2 && testVal <= bound1;
		else
			return testVal >= bound1 && testVal <= bound2;
	}

	static inline float Abs(float val)
	{
		return fabs(val);
	}
};
