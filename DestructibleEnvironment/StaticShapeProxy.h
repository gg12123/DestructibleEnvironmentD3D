#pragma once
#include "ShapeProxy.h"

class StaticShapeProxy : public ShapeProxy
{
protected:
	Shape & RegisterWithPhysics() override;
};