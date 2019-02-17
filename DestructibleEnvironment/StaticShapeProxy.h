#pragma once
#include "ShapeProxy.h"

class StaticShapeProxy : public ShapeProxy
{
protected:
	CompoundShape & RegisterWithPhysics() override;
};