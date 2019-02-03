#pragma once
#include "Face.h"
#include "ShapePoint.h"
#include "DynamicTwoDArray.h"
#include "Constants.h"

enum class PointPlaneRelationship
{
	PointsAbove,
	PointsBelow
};

class MapToPointPlaneRelationship
{
public:
	PointPlaneRelationship GetRelationship(const ShapePoint& p) const
	{
		return m_Map[p.GetHash()];
	}

	void SetRelationship(const ShapePoint& p, PointPlaneRelationship r)
	{
		m_Map[p.GetHash()] = r;
	}

private:
	DynamicArray<PointPlaneRelationship> m_Map;
};
