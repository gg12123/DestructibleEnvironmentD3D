#pragma once
#include "Vector3.h"

class Impulse
{
public:
	Impulse(const Vector3& worldImpulse, const Vector3& worldCollPoint, float impact)
	{
		WorldImpulse = worldImpulse;
		WorldCollisionPoint = worldCollPoint;
		Impact = impact;
	}

	Vector3 WorldImpulse;
	Vector3 WorldCollisionPoint;

	float Impact;
};
