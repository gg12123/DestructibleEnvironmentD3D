#pragma once
#include <vector>
#include <assert.h>
#include "MathU.h"
#include "Polygon2.h"

class PolyIntersectionPoint
{
public:
	const Polygon2* ActivePoly;
	const Polygon2* OtherPoly;
	int EdgeOnActivePoly;
	int EdgeOnOtherPoly;
	Vector2 Position;

	PolyIntersectionPoint(const Vector2& pos, const Polygon2& poly1, const Polygon2& poly2, int edgeOn1, int edgeOn2)
	{
		if (Vector2::Dot(poly1.GetDirectionAt(edgeOn1), poly2.GetNormalAt(edgeOn2)) < 0.0f)
		{
			ActivePoly = &poly1;
			EdgeOnActivePoly = edgeOn1;
			OtherPoly = &poly2;
			EdgeOnOtherPoly = edgeOn2;
		}
		else
		{
			ActivePoly = &poly2;
			EdgeOnActivePoly = edgeOn2;
			OtherPoly = &poly1;
			EdgeOnOtherPoly = edgeOn1;
		}
	}

	bool operator != (const PolyIntersectionPoint& rhs)
	{
		return (ActivePoly != rhs.ActivePoly) || (OtherPoly != rhs.OtherPoly) ||
			(EdgeOnActivePoly != rhs.EdgeOnActivePoly) || (EdgeOnOtherPoly != rhs.EdgeOnOtherPoly);
	}
};

class Polygon2IntersectionFinder
{
public:
	// inputs must be convex
	bool FindIntersection(const Polygon2& poly1, const Polygon2& poly2, Polygon2& intersection)
	{
		if (!poly1.BoundsOverlapWith(poly2))
			return false;

		m_Intersection = &intersection;

		if (FindInitialIntersectionPoint(poly1, poly2))
		{
			FindIntersectionOfOverlappingPolys();
			return true;
		}

		auto poly1PointOutsidePoly2 = PointOnPolyIsOutsideOther(poly1, poly2);
		auto poly2PointOutsidePoly1 = PointOnPolyIsOutsideOther(poly2, poly1);

		if (poly1PointOutsidePoly2 && poly2PointOutsidePoly1)
			return false;

		if (!poly1PointOutsidePoly2 && !poly2PointOutsidePoly1)
		{
			intersection = poly1; // The input polys are the same.
			return true;
		}

		if (poly1PointOutsidePoly2)
		{
			intersection = poly2;
			return true;
		}

		if (poly2PointOutsidePoly1)
		{
			intersection = poly1;
			return true;
		}
		assert(false);
	}

private:
	bool PointOnPolyIsOutsideOther(const Polygon2& poly, const Polygon2& other)
	{
		auto& polyPoints = poly.GetPoints();

		for (auto it = polyPoints.begin(); it != polyPoints.end(); it++)
		{
			if (other.PointIsOutsideAssumingConvex(*it, MathU::SmallNumber))
				return true;
		}
		return false;
	}

	void FindIntersectionOfOverlappingPolys()
	{
		auto startIntPoint = m_CurrentIntersectionPoint;
		do
		{
			m_Intersection->Add(m_CurrentIntersectionPoint.Position);

			Vector2 nextCastPoint;
			auto edgeForNextCast = AddInsidePoints(nextCastPoint);

			CastToNextIntersectionPoint(nextCastPoint, edgeForNextCast);
		} while (m_CurrentIntersectionPoint != startIntPoint);
	}

	void CastToNextIntersectionPoint(const Vector2& currPos, int activePolyCurrEdgeIndex)
	{
		auto activePoly = m_CurrentIntersectionPoint.ActivePoly;
		auto otherPoly = m_CurrentIntersectionPoint.OtherPoly;

		auto castDir = activePoly->GetDirectionAt(activePolyCurrEdgeIndex);

		auto closestDist = MathU::Infinity;
		EdgeCastHit* closestHit = nullptr;

		auto edgeMaskOnOther = activePolyCurrEdgeIndex != m_CurrentIntersectionPoint.EdgeOnActivePoly ?
			-1 : m_CurrentIntersectionPoint.EdgeOnOtherPoly;

		m_CastHitPoints.clear();
		otherPoly->RayCastAllEdges(currPos, castDir, edgeMaskOnOther, m_CastHitPoints);

		for (auto it = m_CastHitPoints.begin(); it != m_CastHitPoints.end(); it++)
		{
			auto dist = ((*it).HitPoint - currPos).Magnitude();

			if (dist < closestDist)
			{
				closestDist = dist;
				closestHit = &(*it);
			}
		}

		m_CurrentIntersectionPoint = PolyIntersectionPoint(closestHit->HitPoint, *activePoly, *otherPoly, activePolyCurrEdgeIndex, closestHit->HitEdgeIndex);
	}

	bool FindInitialIntersectionPoint(const Polygon2& poly1, const Polygon2& poly2)
	{
		auto& poly1Points = poly1.GetPoints();
		auto& poly2Points = poly2.GetPoints();

		auto poly1Count = poly1Points.size();
		auto poly2Count = poly2Points.size();

		for (auto i = 0U; i < poly1Count; i++)
		{
			auto& aP0 = poly1Points[i];
			auto& aP1 = poly1Points[(i + 1) % poly1Count];

			for (auto j = 0U; j < poly2Count; j++)
			{
				auto& bP0 = poly2Points[j];
				auto& bP1 = poly2Points[(j + 1) % poly2Count];

				Vector2 intPoint;
				if (Vector2::LinesIntersect(aP0, aP1, bP0, bP1, intPoint))
				{
					m_CurrentIntersectionPoint = PolyIntersectionPoint(intPoint, poly1, poly2, i, j);
					return true;
				}
			}
		}
		return false;
	}

	int AddInsidePoints(Vector2& nextCastPoint)
	{
		nextCastPoint = m_CurrentIntersectionPoint.Position;

		auto activePoly = m_CurrentIntersectionPoint.ActivePoly;
		auto otherPoly = m_CurrentIntersectionPoint.OtherPoly;

		auto activePolyEdgeIndex = m_CurrentIntersectionPoint.EdgeOnActivePoly;
		auto nextIndex = activePoly->NextIndex(activePolyEdgeIndex);

		auto maybeInsideP = activePoly->GetPointAt(nextIndex);

		while (otherPoly->PointIsInsideAssumingConvex(maybeInsideP))
		{
			m_Intersection->Add(maybeInsideP);
			nextCastPoint = maybeInsideP;
			activePolyEdgeIndex = nextIndex;
			nextIndex = activePoly->NextIndex(nextIndex);
			maybeInsideP = activePoly->GetPointAt(nextIndex);
		}
		return activePolyEdgeIndex;
	}

	PolyIntersectionPoint m_CurrentIntersectionPoint;
	Polygon2* m_Intersection;
	std::vector<EdgeCastHit> m_CastHitPoints;
};