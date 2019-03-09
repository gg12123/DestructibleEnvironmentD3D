#pragma once
#include "PhysicsObject.h"

class StaticBody : public PhysicsObject
{
public:
	StaticBody()
	{
		SetMass(1000000.0f);
	}
};