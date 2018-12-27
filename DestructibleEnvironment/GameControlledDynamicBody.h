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
};
