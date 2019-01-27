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

	void Clear()
	{
		m_Map.Zero();
	}

	bool IsMappedToPointOnReversedFace(const ShapePoint& from) const
	{
		ShapePoint* p = nullptr;

		if (m_Map.TryGetValue(from.GetHash(), p))
			return p;

		return false;
	}

private:
	DynamicArray<ShapePoint*> m_Map;
};
