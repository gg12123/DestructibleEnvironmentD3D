#pragma once
#include "DynamicArray.h"
#include "ShapePoint.h"
#include "Constants.h"

class MapToShapePointOnReversedFace
{
public:
	ShapePoint & GetPointOnReversedFace(const ShapePoint& p) const
	{
		return *m_Map[p.GetHash()];
	}

	void Add(const ShapePoint& from, ShapePoint& to)
	{
		m_Map[from.GetHash()] = &to;
	}
private:
	DynamicArray<ShapePoint*> m_Map;
};
