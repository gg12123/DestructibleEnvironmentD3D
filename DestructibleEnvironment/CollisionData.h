#pragma once
#include "Vector3.h"

class CollisionData
{
public:
	Vector3 Normal1To2;
	Vector3 Position;
	float Penetration;
};

class Impulse
{
public:
	Vector3 WorldImpulse;
	Vector3 LocalImpulse;

	Vector3 WorldCollisionPoint;
	Vector3 LocalCollisionPoint;

	float Impact;
};
