#pragma once
#include <assert.h>
#include "Polygon2.h"
#include "PoolOfRecyclables.h"

struct PerimPoint
{
	int AssociatedIndex;
	Vector2 Position;

	PerimPoint(int assIndex, const Vector2& pos)
	{
		AssociatedIndex = assIndex;
		Position = pos;
	}
};

class Polygon2Splitter
{
public:
	// input polys must be convex
	void Split(const Polygon2& perimPoly, const Polygon2& containedPoly, const std::vector<int>& mapToOverlaidPerimEdge, std::vector<Polygon2*>& splitResults, PoolOfRecyclables<Polygon2>& polyPool)
	{
		m_PerimPoly = &perimPoly;
		m_ContainedPoly = &containedPoly;
		m_MapToOverlaidPerimEdge = &mapToOverlaidPerimEdge;

		CreateCombinedPerimeterPoints();
		CreateNewPolys(splitResults, polyPool);
	}

private:
	int InsertInitialPerimPoint()
	{
		auto& map = *m_MapToOverlaidPerimEdge;
		if (map[0] > 0)
		{
			auto intPerimEdgeIndex = m_PerimPoly->NextIndex(map[0]);

			m_CombinedPerimPoints.emplace_back(PerimPoint(0, m_PerimPoly->GetPointAt(intPerimEdgeIndex)));
			return intPerimEdgeIndex;
		}

		Vector2 C, D;
		ContainedLineAt(0, C, D);

		auto& points = m_PerimPoly->GetPoints();
		auto count = points.size();

		for (int i = 0; i < count; i++)
		{
			auto A = points[i];
			auto B = points[(i + 1) % count];

			Vector2 intPoint;
			if (Vector2::LinesIntersect(A, B, C, D, intPoint))
			{
				m_CombinedPerimPoints.emplace_back(PerimPoint(0, intPoint));
				return i;
			}
		}
		assert(false);
	}

	bool EdgesIntersect(int perimEdgeIndex, int containedEdgeIndex, Vector2& intPoint)
	{
		auto& map = *m_MapToOverlaidPerimEdge;
		auto equalPerimEdge = map[containedEdgeIndex];

		if (equalPerimEdge > 0)
		{
			auto nextPerimEdge = m_PerimPoly->NextIndex(equalPerimEdge);
			intPoint = m_PerimPoly->GetPointAt(nextPerimEdge);
			return perimEdgeIndex == nextPerimEdge;
		}

		Vector2 C, D;
		ContainedLineAt(containedEdgeIndex, C, D);

		auto A = m_PerimPoly->GetPointAt(perimEdgeIndex);
		auto B = m_PerimPoly->GetPointAt(m_PerimPoly->NextIndex(perimEdgeIndex));

		return Vector2::LinesIntersect(A, B, C, D, intPoint);
	}

	void ContainedLineAt(int index, Vector2& P0, Vector2& P1)
	{
		static constexpr float polyMaxBound = 100.0f;

		P0 = m_ContainedPoly->GetPointAt(index);
		P1 = m_ContainedPoly->GetPointAt(m_ContainedPoly->NextIndex(index));

		P1 += polyMaxBound * (P1 - P0).Normalized();
	}

	void CreateCombinedPerimeterPoints()
	{
		m_CombinedPerimPoints.clear();

		auto currContIndex = 1;
		auto currPerimIndex = InsertInitialPerimPoint();
		auto originalPerimPointsAdded = 0;
		auto originalPerimCount = m_PerimPoly->GetPointCount();

		while (currContIndex != 0)
		{
			Vector2 intPoint;
			while ((currContIndex != 0) && EdgesIntersect(currPerimIndex, currContIndex, intPoint))
			{
				m_CombinedPerimPoints.emplace_back(PerimPoint(currContIndex, intPoint));
				currContIndex = m_ContainedPoly->NextIndex(currContIndex);
			}

			if (originalPerimPointsAdded < originalPerimCount)
			{
				auto B = m_PerimPoly->GetPointAt(m_PerimPoly->NextIndex(currPerimIndex));

				m_CombinedPerimPoints.emplace_back(PerimPoint(-1, B));
				currPerimIndex = m_PerimPoly->NextIndex(currPerimIndex);
			}
		}
	}

	void CreateNewPolys(std::vector<Polygon2*>& splitResults, PoolOfRecyclables<Polygon2>& polyPool)
	{
		auto startIndex = 0;
		auto size = m_CombinedPerimPoints.size();
		auto& mapToOverlaidPerimEdge = *m_MapToOverlaidPerimEdge;

		do
		{
			auto& newPoly = polyPool.Recycle();
			newPoly.Add(m_CombinedPerimPoints[startIndex].Position);

			auto index = (startIndex + 1) % size;

			while (m_CombinedPerimPoints[index].AssociatedIndex < 0)
			{
				newPoly.Add(m_CombinedPerimPoints[index].Position);
				index = (index + 1) % size;
			}
			newPoly.Add(m_CombinedPerimPoints[index].Position);
			
			auto& pp = m_CombinedPerimPoints[index];
			
			if (mapToOverlaidPerimEdge[pp.AssociatedIndex] < 0)
			{
				auto i1 = m_ContainedPoly->NextIndex(pp.AssociatedIndex);
				auto i2 = pp.AssociatedIndex;

				newPoly.Add(m_ContainedPoly->GetPointAt(i1));
				newPoly.Add(m_ContainedPoly->GetPointAt(i2));

				splitResults.emplace_back(&newPoly);
			}

			startIndex = index;
		} while (startIndex != 0);
	}

	const Polygon2* m_PerimPoly;
	const Polygon2* m_ContainedPoly;

	const std::vector<int>* m_MapToOverlaidPerimEdge;

	std::vector<PerimPoint> m_CombinedPerimPoints;
};
