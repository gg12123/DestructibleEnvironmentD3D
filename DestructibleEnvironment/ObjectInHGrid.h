#pragma once
#include "Vector3.h"
#include "LastCheckedAgainst.h"
#include "Bounds.h"

class ObjectInHGrid : public LastCheckedAgainst<const ObjectInHGrid*>
{
public:
	const AABB& GetWorldAABB() const
	{

	}
};
