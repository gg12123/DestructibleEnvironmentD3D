#pragma once
#include "PhysicsObject.h"
#include "CompoundShape.h"

class StaticBody : public PhysicsObject
{
public:
	void InitMassProperties(const Transform& refTran)
	{
		SetInvMass(0.0f);
		SetInvInertia(Matrix3());
		CentreAndCache(refTran, Vector3::Zero());
	}
};