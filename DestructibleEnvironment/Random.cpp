#include "pch.h"
#include "Random.h"
#include "Loggers.h"
#include "StringU.h"
#include <stdlib.h>
#include <time.h>

static constexpr auto useRandomSeed = true;

template<bool random>
uint32 GetSeed()
{
	return static_cast<uint32>(time(NULL));
}

template<>
uint32 GetSeed<false>()
{
	return 10U;
}

static void Seed()
{
	static bool seeded = false;

	if (!seeded)
	{
		auto seed = GetSeed<useRandomSeed>();
		Loggers::Physics().Log("Seed - " + StringU::ToString(seed));
		srand(seed);
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