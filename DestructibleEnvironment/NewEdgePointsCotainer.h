#pragma once
#include <vector>
#include "Vector3.h"
#include "ToBeNewPoint.h"

class NewEdgePointsContainer
{
public:
	bool HasNewEdgePointsAt(int index)
	{
		return m_EdgePoints[index].size() > 0;
	}

	void StartReturningEdgesFrom(int index, Vector3 edgeOrigin)
	{
		m_PosOfLastReturned = edgeOrigin;
		m_ActiveIndex = index;
	}

	ToBeNewPoint* GetNextNewEdgePoint()
	{
		auto& points = m_EdgePoints[m_ActiveIndex];

		if (points.size() == 0U)
			return nullptr;

		auto closestDist = MathU::Infinity;
		ToBeNewPoint* next = nullptr;

		for (auto it = points.begin(); it != points.end(); it++)
		{
			auto p = *it;
			auto dist = (p->Position - m_PosOfLastReturned).Magnitude();

			if (dist < closestDist)
			{
				closestDist = dist;
				next = p;
			}
		}

		m_PosOfLastReturned = next->Position;
		return next;
	}

	void AddEdgePoint(int index, ToBeNewPoint& point)
	{
		while (index >= m_EdgePoints.size())
			m_EdgePoints.emplace_back(std::vector<ToBeNewPoint*>());

		m_EdgePoints[index].emplace_back(&point);
	}

private:
	Vector3 m_PosOfLastReturned;
	int m_ActiveIndex;
	std::vector<std::vector<ToBeNewPoint*>> m_EdgePoints;
};
