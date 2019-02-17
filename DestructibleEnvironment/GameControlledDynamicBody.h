#pragma once
#include "DynamicBodyProxy.h"

class GameControlledDynamicBody : public DynamicBodyProxy
{
public:
	virtual void FixedUpdate()
	{
	}

protected:
	CompoundShape & RegisterWithPhysics() override;
};
