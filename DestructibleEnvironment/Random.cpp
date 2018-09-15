#include "pch.h"
#include "Random.h"
#include <stdlib.h>
#include <time.h>

static void Seed()
{
	static bool seeded = false;

	if (!seeded)
	{
		srand(time(NULL));
		seeded = true;
	}
}

float Random::Range(float min, float max)
{
	Seed();
	auto p = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
	return min + p * (max - min);
}

int Random::Range(int minInclusive, int maxExclusive)
{
	Seed();
	return rand() % (maxExclusive - minInclusive) + minInclusive;
}