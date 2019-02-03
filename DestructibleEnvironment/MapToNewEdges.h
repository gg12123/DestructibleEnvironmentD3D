#pragma once
#include <vector>
#include "ShapePoint.h"
#include "ShapeEdge.h"
#include "DynamicTriangleArray.h"
#include "Constants.h"

class MapToNewEdges
{
public:
	MapToNewEdges()
	{
	}

	ShapeEdge & GetNewEdge(const ShapePoint& p0, const ShapePoint& p1) const
	{
		auto e = m_Edges.Get(p0.GetHash(), p1.GetHash());
		return *e;
	}

	void AddNewEdge(ShapeEdge& edge)
	{
		AddNewEdge(edge.GetP0(), edge.GetP1(), edge);
	}

	void AddNewEdge(ShapePoint& p0, ShapePoint& p1, ShapeEdge& edge)
	{
		p0.TryAssignHash();
		p1.TryAssignHash();

		m_Edges.Get(p0.GetHash(), p1.GetHash()) = &edge;
	}

private:
	DynamicTriangleArray<ShapeEdge*> m_Edges;
};