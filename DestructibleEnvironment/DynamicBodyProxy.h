#pragma once
#include "ShapeProxy.h"
#include "Vector3.h"

class DynamicBodyProxy : public ShapeProxy
{
public:
	DynamicBodyProxy()
	{
	}

	DynamicBodyProxy(Shape& body);

protected:
	Shape & RegisterWithPhysics() override;
};
