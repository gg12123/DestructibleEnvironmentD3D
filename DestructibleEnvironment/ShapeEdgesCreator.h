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
	void CreateEdge(ShapePoint& p0, ShapePoint& p1)
	{

	}

	const auto& GetMapToNewEdges() const
	{
		return m_MapToEdges;
	}

private:
	MapToNewEdges m_MapToEdges;
};