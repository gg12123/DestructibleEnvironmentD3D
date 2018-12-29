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
	PointPlaneRelationship GetRelationship(const Face& f, const ShapePoint& p) const
	{
		return m_Map.Get(f.GetHash(), p.GetHash());
	}

	void SetRelationship(const Face& f, const ShapePoint& p, PointPlaneRelationship r)
	{
		m_Map.Get(f.GetHash(), p.GetHash()) = r;
	}

private:
	DynamicTwoDArray<PointPlaneRelationship> m_Map;
};
