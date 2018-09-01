#pragma once
#include "PhysicsObject.h"

class StaticBody : public PhysicsObject
{
public:
	StaticBody()
	{
		SetMass(10000.0f);
	}
};