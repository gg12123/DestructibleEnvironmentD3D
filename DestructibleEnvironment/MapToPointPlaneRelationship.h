#pragma once
#include "Face.h"
#include "ShapePoint.h"

enum class PointPlaneRelationship
{
	PointsAbove,
	PointsBelow
};

class MapToPointPlaneRelationship
{
public:
	PointPlaneRelationship GetRelationship(const Face& f, const ShapePoint& p) const
	{

	}
};
