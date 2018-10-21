#pragma once
#include <stack>
#include "Polygon2.h"
#include "PoolOfRecyclables.h"

class ConvexPolyCreator
{
public:
	void ToConvexPieces(Polygon2& poly, std::vector<Polygon2*>& convexPieces, PoolOfRecyclables<Polygon2>& polyPool)
	{
		m_ConcavePolys.push(&poly);

		while (!m_ConcavePolys.empty())
		{
			auto p = m_ConcavePolys.top();
			m_ConcavePolys.pop();
			Process(*p, convexPieces, polyPool);
		}
	}

private:
	void Process(Polygon2& poly, std::vector<Polygon2*>& convexPieces, PoolOfRecyclables<Polygon2>& polyPool)
	{
		auto count = poly.GetPointCount();

		for (auto i = 0; i < count; i++)
		{
			if (poly.CalculateWindingAt(i) > MathU::SmallNumber)
			{
				// positive winding implies anti clockwise, which is the wrong winding direction

				PerformCast(poly, i);

				auto& c1 = polyPool.Recycle();
				auto& c2 = polyPool.Recycle();

				c1.Clear();
				c2.Clear();

				c1.Add(m_IntPoint);

				auto c1End = poly.NextIndex(m_IAnchor);
				for (int i = m_IPlus; i != c1End; i = poly.NextIndex(i))
					c1.Add(poly.GetPointAt(i));

				auto c2End = m_IPlus;
				for (int i = m_IAnchor; i != c2End; i = poly.NextIndex(i))
					c2.Add(poly.GetPointAt(i));

				c2.Add(m_IntPoint);

				m_ConcavePolys.push(&c1);
				m_ConcavePolys.push(&c2);
				return;
			}
		}
		convexPieces.emplace_back(&poly);
	}

	void PerformCast(const Polygon2& poly, int incorrectWindingCorner)
	{
		m_IAnchor = incorrectWindingCorner;

		auto masked1 = m_IAnchor;
		auto masked2 = poly.PreviousIndex(masked1);

		auto castDir = poly.GetDirectionAt(masked2);
		auto origin = poly.GetPointAt(m_IAnchor);

		auto closestDist = MathU::Infinity;
		auto indexOfClosest = -1;
		Vector2 intPoint;

		auto& points = poly.GetPoints();
		auto count = points.size();
		for (int i = 0; i < count; i++)
		{
			if (i == masked1 || i == masked2)
				continue;

			auto next = (i + 1) % count;

			auto& p0 = points[i];
			auto& p1 = points[next];

			if (Vector2::CastLineIntersect(origin, castDir, p0, p1, intPoint))
			{
				auto dist = (intPoint - origin).Magnitude();
				if (dist < closestDist)
				{
					closestDist = dist;
					indexOfClosest = i;
					m_IntPoint = intPoint;
				}
			}
		}

		assert(indexOfClosest != -1);

		m_IMinus = indexOfClosest;
		m_IPlus = poly.NextIndex(m_IMinus);
	}

	std::stack<Polygon2*> m_ConcavePolys;

	int m_IPlus;
	int m_IMinus;
	int m_IAnchor;
	Vector2 m_IntPoint;
};
