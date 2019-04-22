#pragma once
#include "SatOptimisedCollisionDetection.h"
#include "ContactPoints.h"

class ContactPointFinder
{
private:
	static constexpr auto Tol = 0.0001f;

	bool ContactPointExistsAlready(const Vector3& p) const
	{
		for (auto& existing : m_EcContactPoints)
		{
			if (Vector3::ProjectOnPlane(m_ContactPlaneNormal, p - existing.Point).MagnitudeSqr() < Tol)
				return true;
		}

		for (auto& existing : m_PipContactPoints)
		{
			if (Vector3::ProjectOnPlane(m_ContactPlaneNormal, p - existing.Point).MagnitudeSqr() < Tol)
				return true;
		}
		return false;
	}

	void FindPointsInContact(const SatInputShape& shape, std::vector<bool>& isInContact, std::vector<Vector3>& pointsInContact, const ContactPlane& contactPlane)
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
		}
	}

	void FindEdgesInContact(const SatInputShape& shape, std::vector<int>& edgesInContact, const std::vector<bool>& pointIsInContact)
	{
		auto& edges = shape.GetEdgeIndexsPoints();

		for (auto i = 0u; i < edges.size(); i += 2u)
		{
			auto e0 = edges[i];
			auto e1 = edges[i + 1];
			if (pointIsInContact[e0] && pointIsInContact[e1])
			{
				edgesInContact.emplace_back(i);
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
		auto& edgesA = shapeA.GetEdgeIndexsPoints();
		auto& pointsB = shapeB.GetPoints();
		auto& edgesB = shapeB.GetEdgeIndexsPoints();

		for (auto i = 0u; i < m_ShapeAEdgesInContact.size(); i++)
		{
			auto edgeIndexA = m_ShapeAEdgesInContact[i];

			auto& p0A = pointsA[edgesA[edgeIndexA]];
			auto& p1A = pointsA[edgesA[edgeIndexA + 1]];

			for (auto j = 0u; j < m_ShapeBEdgesInContact.size(); j++)
			{
				auto edgeIndexB = m_ShapeBEdgesInContact[j];

				auto& p0B = pointsB[edgesB[edgeIndexB]];
				auto& p1B = pointsB[edgesB[edgeIndexB + 1]];

				if (EdgesOverlapInContactPlane(p0A, p1A, p0B, p1B))
				{
					Vector3 c, d;
					if (Vector3::FindClosestPointsBetweenLines(p0A, p1A, p0B, p1B, c, d))
					{
						auto p = (c + d) / 2.0f;
						if (!ContactPointExistsAlready(p))
							m_EcContactPoints.emplace_back(ContactPointEC(p, edgeIndexA, edgeIndexB));
					}
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

	void FindPointInPolyContactPoints(const SatInputShape& shape, const std::vector<Vector3>& points)
	{
		for (auto i = 0u; i < points.size(); i++)
		{
			auto& p = points[i];

			if (PointIsInShape(shape, p) && !ContactPointExistsAlready(p))
			{
				m_PipContactPoints.emplace_back(ContactPointPIP(p, i, shape.GetId()));
			}
		}
	}

public:
	void FindContactPoints(const SatInputShape& shapeA, const SatInputShape& shapeB, const ContactPlane& contactPlane)
	{
		m_ContactPlaneNormal = contactPlane.GetNormal();

		m_ShapeAPointIsInContact.clear();
		m_ShapeBPointIsInContact.clear();
		m_ShapeAInContactPoints.clear();
		m_ShapeBInContactPoints.clear();
		m_ShapeAEdgesInContact.clear();
		m_ShapeBEdgesInContact.clear();
		m_EcContactPoints.clear();
		m_PipContactPoints.clear();

		FindPointsInContact(shapeA, m_ShapeAPointIsInContact, m_ShapeAInContactPoints, contactPlane);
		FindPointsInContact(shapeB, m_ShapeBPointIsInContact, m_ShapeBInContactPoints, contactPlane);

		FindEdgesInContact(shapeA, m_ShapeAEdgesInContact, m_ShapeAPointIsInContact);
		FindEdgesInContact(shapeB, m_ShapeBEdgesInContact, m_ShapeBPointIsInContact);

		FindEdgeOverlapContactPoints(shapeA, shapeB);
		FindPointInPolyContactPoints(shapeA, m_ShapeBInContactPoints);
		FindPointInPolyContactPoints(shapeB, m_ShapeAInContactPoints);
	}

	const auto GetEcContactPoints() const
	{
		return m_EcContactPoints;
	}

	const auto GetPipContactPoints() const
	{
		return m_PipContactPoints;
	}

private:
	std::vector<bool> m_ShapeAPointIsInContact;
	std::vector<bool> m_ShapeBPointIsInContact;

	std::vector<Vector3> m_ShapeAInContactPoints;
	std::vector<Vector3> m_ShapeBInContactPoints;

	std::vector<int> m_ShapeAEdgesInContact;
	std::vector<int> m_ShapeBEdgesInContact;

	std::vector<ContactPointEC> m_EcContactPoints;
	std::vector<ContactPointPIP> m_PipContactPoints;

	Vector3 m_ContactPlaneNormal;
};
