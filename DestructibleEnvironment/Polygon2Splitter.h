#pragma once
#include <assert.h>
#include "Polygon2.h"

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
	void Split(const Polygon2& perimPoly, const Polygon2& containedPoly, std::vector<Polygon2>& splitResults)
	{
		m_PerimPoly = &perimPoly;
		m_ContainedPoly = &containedPoly;

		CreateCombinedPerimeterPoints();
		CreateNewPolys(splitResults);
	}

private:
	int InsertInitialPerimPoint()
	{
		Vector2 C, D;
		ContainedLineAt(0, C, D);

		auto points = m_PerimPoly->GetPoints();

		auto data = points.Data();
		auto count = points.Size();

		for (int i = 0; i < count; i++)
		{
			auto A = data[i];
			auto B = data[(i + 1) % count];

			Vector2 intPoint;
			if (Vector2::LinesIntersect(A, B, C, D, intPoint))
			{
				m_CombinedPerimPoints.emplace_back(PerimPoint(0, intPoint));
				return i;
			}
		}
		assert(false);
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

		Vector2 C, D;

		auto currContIndex = 1;
		auto currPerimIndex = InsertInitialPerimPoint();
		auto originalPerimPointsAdded = 0;
		auto originalPerimCount = m_PerimPoly->GetPointCount();

		while (currContIndex != 0)
		{
			auto A = m_PerimPoly->GetPointAt(currPerimIndex);
			auto B = m_PerimPoly->GetPointAt(m_PerimPoly->NextIndex(currPerimIndex));

			ContainedLineAt(currContIndex, C, D);

			Vector2 intPoint;

			while ((currContIndex != 0) && Vector2::LinesIntersect(A, B, C, D, intPoint))
			{
				m_CombinedPerimPoints.emplace_back(PerimPoint(currContIndex, intPoint));
				currContIndex = m_ContainedPoly->NextIndex(currContIndex);

				ContainedLineAt(currContIndex, C, D);
			}

			if (originalPerimPointsAdded < originalPerimCount)
			{
				m_CombinedPerimPoints.emplace_back(PerimPoint(-1, B));
				currPerimIndex = m_PerimPoly->NextIndex(currPerimIndex);
			}
		}
	}

	void CreateNewPolys(std::vector<Polygon2>& splitResults)
	{
		auto startIndex = 0;
		auto size = m_CombinedPerimPoints.size();

		do
		{
			Polygon2 newPoly;
			newPoly.Add(m_CombinedPerimPoints[startIndex].Position);

			auto index = (startIndex + 1) % size;

			while (m_CombinedPerimPoints[index].AssociatedIndex < 0)
			{
				newPoly.Add(m_CombinedPerimPoints[index].Position);
				index = (index + 1) % size;
			}
			newPoly.Add(m_CombinedPerimPoints[index].Position);
			
			auto& pp = m_CombinedPerimPoints[index];

			auto i1 = m_ContainedPoly->NextIndex(pp.AssociatedIndex);
			auto i2 = pp.AssociatedIndex;

			newPoly.Add(m_CombinedPerimPoints[i1].Position);
			newPoly.Add(m_CombinedPerimPoints[i2].Position);
			
			splitResults.emplace_back(std::move(newPoly));
			startIndex = index;
		} while (startIndex != 0);
	}

	const Polygon2* m_PerimPoly;
	const Polygon2* m_ContainedPoly;

	std::vector<PerimPoint> m_CombinedPerimPoints;
};
