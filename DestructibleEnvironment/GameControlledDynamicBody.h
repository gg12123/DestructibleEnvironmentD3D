#pragma once
#include "DynamicBodyProxy.h"

class Rigidbody;

class GameControlledDynamicBody : public DynamicBodyProxy
{
public:
	virtual void FixedUpdate()
	{
	}

protected:
	Shape & RegisterWithPhysics() override;

	void AddForce(const Vector3& force);
	void AddTorque(const Vector3& torque);

private:
	Rigidbody * m_Rigidbody;
};
