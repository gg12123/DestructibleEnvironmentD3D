#pragma once
#include "ShapeEdge.h"
#include "SplitShapeEdge.h"
#include "MapToNewEdges.h"
#include "CutPathElement.h"

class ShapeEdgesCreator
{
public:
	void CreateEdges(const std::vector<CutPathElement>& cp)
	{

	}

	// called by the reversing code
	void CreateEdge(const ShapePoint& p0, const ShapePoint& p1)
	{

	}

	const auto& GetMapToNewEdges() const
	{
		return m_MapToEdges;
	}

	bool EdgeExistsBetween(const ShapePoint& p0, const ShapePoint& p1) const
	{

	}

private:
	MapToNewEdges m_MapToEdges;
};