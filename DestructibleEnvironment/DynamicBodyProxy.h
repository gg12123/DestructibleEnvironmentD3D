#pragma once
#include "ShapeProxy.h"

class DynamicBodyProxy : public ShapeProxy
{

public:
	DynamicBodyProxy(Shape& shape) : ShapeProxy(shape)
	{
	}

protected:
	Shape & RegisterWithPhysics() override;
};
