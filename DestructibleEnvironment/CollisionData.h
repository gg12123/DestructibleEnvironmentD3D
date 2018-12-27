#pragma once
#include "Vector3.h"

class Impulse
{
public:
	Impulse(const Vector3& worldImpulse, const Vector3& worldCollPoint, float impact) : 
		WorldImpulse(worldImpulse),
		WorldImpulsePoint(worldCollPoint),
		Impact(impact)
	{
	}

	const Vector3 WorldImpulse;
	const Vector3 WorldImpulsePoint;

	const float Impact;
};
