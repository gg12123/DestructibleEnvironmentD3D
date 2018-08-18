#pragma once
#include "Entity.h"
#include "Transform.h"

class Light : public Entity
{
public:
	Vector3 GetLightPosition()
	{
		return GetTransform().GetPosition();
	}
};
