#pragma once
#include <array>
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
	std::array<ShapePoint*, Constants::MaxNumPoints> m_Map;
};
