#pragma once
#include "Entity.h"
#include "DynamicBodyProxy.h"

class Joint;

class WindmillMotor : public Entity
{
public:
	WindmillMotor(DynamicBodyProxy& millBase)
	{
		m_MillBase = &millBase;
	}

	void Update() override;

protected:
	void Awake() override;

private:
	const Joint* FindAxisAndJoint();

	DynamicBodyProxy * m_MillBase;
	Vector3 m_Axis;
};
