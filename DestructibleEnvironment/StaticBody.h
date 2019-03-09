#pragma once
#include "PhysicsObject.h"

class StaticBody : public PhysicsObject
{
public:
	StaticBody()
	{
		SetInvMass(0.0f);
		SetInvInertia(Matrix3(0.0f));
	}
};