#pragma once
#include <vector>
#include "Vector2.h"
#include "PointInPolyCase.h"
#include "MathU.h"

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
	void EnsureCorrectWindingDirection()
	{
		if (!AreaGivesCorrectWinding(CalculateSignedArea()))
		{

		}
	}

	float CalculateSignedArea() const
	{

	}

	static bool AreaGivesCorrectWinding(float a)
	{
		// polys must be clockwise
		// negative area implies clockwise
		return a < 0.0f;
	}

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

	PointInPolyCase PointIsInsideAssumingConvex(const Vector2& p) const
	{

	}

	PointInPolyCase PointIsInside(const Vector2& p) const
	{
		auto wn = 0;
		auto c = m_Points.size();

		for (auto i = 0U; i < c; i++)
		{
			auto next = (i + 1) % c;
			auto amountLeft = AmountLeft(m_Points[i], m_Points[next], p);

			if (amountLeft == 0.0f)
			{
				if (MathU::IsBetweenInclusive(m_Points[i].x, m_Points[next].x, p.x))
					return PointInPolyCase::OnBoundary;
			}

			if ((m_Points[next].y  > p.y) && (m_Points[i].y <= p.y)) // upward crossing
			{ 
				if (amountLeft > 0.0f)
					wn++;
			}
			else if ((m_Points[next].y <= p.y) && (m_Points[i].y > p.y)) // downward crossing
			{
				if (amountLeft < 0.0f)
					wn--;
			}
		}

		return (wn == 0) ? PointInPolyCase::Outside : PointInPolyCase::Inside;
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

	int PreviousIndex(int index) const
	{
		auto s = m_Points.size();
		return (index - 1 + s) % s;
	}

	Vector2 GetDirectionAt(int index) const
	{

	}

	Vector2 GetNormalAt(int index) const
	{
		// this points to outside the poly - assuing the winding of the poly is correct.
	}

	Vector2 GetCentre() const
	{

	}

	// returns a negative number for clockwise and a positive number for anti-clockwise
	float CalculateWindingAt(int index)
	{
		auto& p = m_Points[index];
		return -Vector2::Cross2D(p - m_Points[PreviousIndex(index)], m_Points[NextIndex(index)] - p);
	}

private:
	static inline float AmountLeft(const Vector2& lineP0, const Vector2& lineP1, const Vector2& p)
	{
		return ((lineP1.x - lineP0.x) * (p.y - lineP0.y) - (p.x - lineP0.x) * (lineP1.y - lineP0.y));
	}

	std::vector<Vector2> m_Points;
};