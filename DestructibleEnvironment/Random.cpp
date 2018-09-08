#include "pch.h"
#include "Random.h"
#include <stdlib.h>
#include <time.h>

class SeedRand
{
public:
	SeedRand()
	{
		srand(time(NULL));
	}
};

static SeedRand s = SeedRand();

float Random::Range(float min, float max)
{
	auto p = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
	return min + p * (max - min);
}

int Random::Range(int minInclusive, int maxExclusive)
{
	return rand() % (maxExclusive - minInclusive) + minInclusive;
}