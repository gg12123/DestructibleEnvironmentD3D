#pragma once
#include <vector>
#include "Vector2.h"
#include "ReadOnlyView.h"

class EdgeCastHit
{
public:
	Vector2 HitPoint;
	int HitEdgeIndex;

	EdgeCastHit(const Vector2& hp, int i)
	{
		HitPoint = hp;
		HitEdgeIndex = i;
	}
};

class Polygon2
{
public:
	void RayCastAllEdges(const Vector2& origin, const Vector2& dir, std::vector<EdgeCastHit>& hitPoints) const
	{
		RayCastAllEdges(origin, dir, -1, hitPoints);
	}

	void RayCastAllEdges(const Vector2& origin, const Vector2& dir, int edgeMask, std::vector<EdgeCastHit>& hitPoints) const
	{
		auto count = m_Points.size();
		auto pEnd = origin + 100.0f * dir; //TODO ....

		for (auto i = 0U; i < count; i++)
		{
			if (i == edgeMask)
				continue;

			auto& p0 = m_Points[i];
			auto& p1 = m_Points[(i + 1) % count];

			Vector2 intPoint;
			if (Vector2::LinesIntersect(p0, p1, origin, pEnd, intPoint))
				hitPoints.emplace_back(EdgeCastHit(intPoint, i));
		}
	}

	bool BoundsOverlapWith(const Polygon2& other) const
	{

	}

	bool PointIsInsideAssumingConvex(const Vector2& p) const
	{

	}

	bool PointIsInsideAssumingConvex(const Vector2& p, float insideTol) const
	{
		// must be inside by at least the tolerance
	}

	bool PointIsOutsideAssumingConvex(const Vector2& p, float outsideTol) const
	{
		// must be outside by at least the tolerance
	}

	auto GetPointCount() const
	{
		return m_Points.size();
	}

	auto GetPointAt(int index) const
	{
		return m_Points[index];
	}

	const auto& GetPoints() const
	{
		return m_Points;
	}

	void Clear()
	{
		m_Points.clear();
	}

	void Add(const Vector2& p)
	{
		m_Points.emplace_back(p);
	}

	int NextIndex(int index) const
	{
		return (index + 1) % m_Points.size();
	}

	Vector2 GetDirectionAt(int index) const
	{

	}

	Vector2 GetNormalAt(int index) const
	{

	}

	Vector2 GetCentre() const
	{

	}

private:
	std::vector<Vector2> m_Points;
};