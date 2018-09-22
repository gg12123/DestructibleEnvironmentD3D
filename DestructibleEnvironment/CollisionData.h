#pragma once
#include "Vector3.h"

class CollisionData
{
public:
	Vector3 Normal1To2;
	std::vector<Vector3> Points;
	float Penetration;
};

class Impulse
{
public:
	Vector3 WorldImpulse;
	Vector3 WorldCollisionPoint;

	float Impact;
};
