#pragma once
#include <vector>
#include "ShapePoint.h"
#include "ShapeEdge.h"
#include "TwoDArray.h"
#include "Constants.h"

class MapToNewEdges
{
public:
	MapToNewEdges()
	{
		m_Edges.Clear(nullptr);
	}

	bool EdgeExistsBetween(const ShapePoint& p0, const ShapePoint& p1) const
	{
		m_Edges.Get(p0.GetHash(), p1.GetHash());
	}

	ShapeEdge & GetNewEdge(const ShapePoint& p0, const ShapePoint& p1) const
	{
		return *m_Edges.Get(p0.GetHash(), p1.GetHash());
	}

	void AddNewEdge(const ShapePoint& p0, const ShapePoint& p1, ShapeEdge& edge)
	{
		m_Edges.Get(p0.GetHash(), p1.GetHash()) = &edge;
		m_Edges.Get(p1.GetHash(), p0.GetHash()) = &edge;

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
			m_Edges.Get(k, j) = nullptr;
		}

		m_ToClear.clear();
	}

private:
	// TODO - This will contain edges for multiple shapes so is ok to still use max num points for one shape
	TwoDArray<Constants::MaxNumPoints, Constants::MaxNumPoints, ShapeEdge*> m_Edges;
	std::vector<int> m_ToClear;
};