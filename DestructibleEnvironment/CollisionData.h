#pragma once
#include "Vector3.h"

class Impulse
{
public:
	Impulse(const Vector3& worldImpulse, const Vector3& worldCollPoint, float impact) : 
		WorldImpulse(worldImpulse),
		WorldCollisionPoint(worldCollPoint),
		Impact(impact)
	{
	}

	const Vector3 WorldImpulse;
	const Vector3 WorldCollisionPoint;

	const float Impact;
};
