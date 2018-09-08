#pragma once
#include <assert.h>

class Random
{
public:
	static float Range(float min, float max);
	static int Range(int minInclusive, int maxExclusive);
};