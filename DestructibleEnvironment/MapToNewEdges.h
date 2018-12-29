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
		m_Edges.Clear(0);
	}

	bool EdgeExistsBetween(const ShapePoint& p0, const ShapePoint& p1)
	{
		return m_Edges.Get(p0.GetHash(), p1.GetHash());
	}

	ShapeEdge & GetNewEdge(const ShapePoint& p0, const ShapePoint& p1) const
	{
		auto e = m_Edges.Get(p0.GetHash(), p1.GetHash());
		assert(e);
		return *e;
	}

	void AddNewEdge(const ShapePoint& p0, const ShapePoint& p1, ShapeEdge& edge)
	{
		m_Edges.Get(p0.GetHash(), p1.GetHash()) = &edge;

		m_ToClear.emplace_back(p0.GetHash());
		m_ToClear.emplace_back(p1.GetHash());
	}

	void Clear()
	{
		for (auto i = 0U; i < m_ToClear.size(); i += 2)
		{
			auto j = m_ToClear[i];
			auto k = m_ToClear[i + 1U];

			m_Edges.Get(j, k) = nullptr;
		}

		m_ToClear.clear();
	}

private:
	DynamicTriangleArray<ShapeEdge*> m_Edges;
	std::vector<int> m_ToClear;
};