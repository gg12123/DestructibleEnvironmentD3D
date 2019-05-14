#pragma once
#include "SatOptimisedCollisionDetection.h"
#include "Bounds.h"

class ContactPointFinder
{
private:
	static constexpr auto Tol = 0.0001f;

	void AddNewContactPoint(const Vector3& p)
	{
		for (auto& existing : m_ContactPoints)
		{
			if (Vector3::ProjectOnPlane(m_ContactPlaneNormal, p - existing).MagnitudeSqr() < Tol)
				return;
		}
		m_ContactPoints.emplace_back(p);
	}

	void FindPointsInContact(const SatInputShape& shape, std::vector<uint8>& isInContact, SimdStdVector<Vector3>& pointsInContact,
		const ContactPlane& contactPlane, BoundsCalculator& boundsCalc)
	{
		auto n = contactPlane.GetNormal();

		auto minBound = contactPlane.GetContactMin() - Tol;
		auto maxBound = contactPlane.GetContactMax() + Tol;

		for (auto& p : shape.GetPoints())
		{
			auto comp = Vector3::Dot(n, p);
			auto inContact = (comp >= minBound) && (comp <= maxBound);
			isInContact.emplace_back(inContact);

			if (inContact)
				pointsInContact.emplace_back(p);

			boundsCalc.Update(p);
		}
	}

	void FindEdgesInContact(const SatInputShape& shape, std::vector<int>& edgesInContact, const std::vector<uint8>& pointIsInContact, const AABB& othersAABB)
	{
		auto& edges = shape.GetEdgeIndexsPoints();
		auto& points = shape.GetPoints();

		for (auto i = 0u; i < edges.size(); i += 2u)
		{
			auto e0 = edges[i];
			auto e1 = edges[i + 1];
			if (pointIsInContact[e0] && pointIsInContact[e1])
			{
				if (othersAABB.IntersectsSegment(points[e0], points[e1]))
				{
					edgesInContact.emplace_back(e0);
					edgesInContact.emplace_back(e1);
				}
			}
		}
	}

	bool EdgesOverlapInContactPlane(const Vector3& p0A, const Vector3& p1A, const Vector3& p0B, const Vector3& p1B)
	{
		auto da = Vector3::Cross(p1A - p0A, m_ContactPlaneNormal);
		auto db = Vector3::Cross(p1B - p0B, m_ContactPlaneNormal);

		return (Vector3::Dot(p0A - p0B, db) * Vector3::Dot(p1A - p0B, db) <= 0.0f) &&
			(Vector3::Dot(p0B - p0A, da) * Vector3::Dot(p1B - p0A, da) <= 0.0f);
	}

	void FindEdgeOverlapContactPoints(const SatInputShape& shapeA, const SatInputShape& shapeB)
	{
		auto& pointsA = shapeA.GetPoints();
		auto& pointsB = shapeB.GetPoints();

		for (auto i = 0u; i < m_ShapeAEdgesInContact.size(); i += 2u)
		{
			auto& p0A = pointsA[m_ShapeAEdgesInContact[i]];
			auto& p1A = pointsA[m_ShapeAEdgesInContact[i + 1]];

			for (auto j = 0u; j < m_ShapeBEdgesInContact.size(); j += 2u)
			{
				auto& p0B = pointsB[m_ShapeBEdgesInContact[j]];
				auto& p1B = pointsB[m_ShapeBEdgesInContact[j + 1]];

				if (EdgesOverlapInContactPlane(p0A, p1A, p0B, p1B))
				{
					Vector3 c, d;
					if (Vector3::FindClosestPointsBetweenLines(p0A, p1A, p0B, p1B, c, d))
						AddNewContactPoint((c + d) / 2.0f);
				}
			}
		}
	}

	bool PointIsInShape(const SatInputShape& shape, const Vector3& p)
	{
		auto& normals = shape.GetFaceNormals();
		auto& p0Indexs = shape.GetFaceP0Indexs();
		auto& points = shape.GetPoints();

		auto itNormal = normals.begin();
		auto itP0 = p0Indexs.begin();

		for (; itNormal != normals.end(); itNormal++, itP0++)
		{
			auto& n = *itNormal;
			auto& p0 = points[*itP0];

			if (Vector3::Dot(p - p0, n) > Tol)
				return false;
		}
		return true;
	}

	void FindPointInPolyContactPoints(const SatInputShape& shape, const AABB& shapeAABB, const SimdStdVector<Vector3>& points)
	{
		for (auto& p : points)
		{
			if (shapeAABB.ContainsPoint(p))
			{
				if (PointIsInShape(shape, p))
					AddNewContactPoint(p);
			}
		}
	}

public:
	const auto& FindContactPoints(const SatInputShape& shapeA, const SatInputShape& shapeB, const ContactPlane& contactPlane)
	{
		m_ContactPlaneNormal = contactPlane.GetNormal();

		m_ShapeABounds.Reset();
		m_ShapeBBounds.Reset();

		m_ShapeAPointIsInContact.clear();
		m_ShapeBPointIsInContact.clear();
		m_ShapeAInContactPoints.clear();
		m_ShapeBInContactPoints.clear();
		m_ShapeAEdgesInContact.clear();
		m_ShapeBEdgesInContact.clear();
		m_ContactPoints.clear();

		FindPointsInContact(shapeA, m_ShapeAPointIsInContact, m_ShapeAInContactPoints, contactPlane, m_ShapeABounds);
		FindPointsInContact(shapeB, m_ShapeBPointIsInContact, m_ShapeBInContactPoints, contactPlane, m_ShapeBBounds);

		auto shapeAAABB = m_ShapeABounds.ToAABB();
		auto shapeBAABB = m_ShapeBBounds.ToAABB();

		shapeAAABB.Fatten(Tol);
		shapeBAABB.Fatten(Tol);

		FindEdgesInContact(shapeA, m_ShapeAEdgesInContact, m_ShapeAPointIsInContact, shapeBAABB);
		FindEdgesInContact(shapeB, m_ShapeBEdgesInContact, m_ShapeBPointIsInContact, shapeAAABB);

		FindEdgeOverlapContactPoints(shapeA, shapeB);

		if (m_ShapeAInContactPoints.size() > 2u)
			FindPointInPolyContactPoints(shapeA, shapeAAABB, m_ShapeBInContactPoints);

		if (m_ShapeBInContactPoints.size() > 2u)
			FindPointInPolyContactPoints(shapeB, shapeBAABB, m_ShapeAInContactPoints);

		return m_ContactPoints;
	}

private:
	BoundsCalculator m_ShapeABounds;
	BoundsCalculator m_ShapeBBounds;

	// Points that are inside the contact plane
	std::vector<uint8> m_ShapeAPointIsInContact;
	std::vector<uint8> m_ShapeBPointIsInContact;

	SimdStdVector<Vector3> m_ShapeAInContactPoints;
	SimdStdVector<Vector3> m_ShapeBInContactPoints;

	// Edges that are inside the contact plane,
	// and inside the others AABB.
	std::vector<int> m_ShapeAEdgesInContact;
	std::vector<int> m_ShapeBEdgesInContact;

	SimdStdVector<Vector3> m_ContactPoints;

	Vector3 m_ContactPlaneNormal;
};