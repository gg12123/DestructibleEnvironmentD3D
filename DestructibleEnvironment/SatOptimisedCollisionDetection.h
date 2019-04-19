#pragma once
#include <vector>
#include "Vector3.h"
#include "CollisionData.h"
#include "Shape.h"

class SatInputShape
{
public:
	Vector3 FindSupportVertex(const Vector3& dir) const
	{
		auto maxComp = MathU::NegativeInfinity;
		const Vector3* support = nullptr;

		for (auto& p : *m_Points)
		{
			auto comp = Vector3::Dot(p, dir);
			if (comp > maxComp)
			{
				maxComp = comp;
				support = &p;
			}
		}
		return *support;
	}

	const auto& GetPoints() const
	{
		return *m_Points;
	}

	const auto& GetFaceNormals() const
	{
		return *m_FaceNormals;
	}

	const auto& GetFaceP0Indexs() const
	{
		return *m_FaceP0Indexs;
	}

	const auto& GetEdgeIndexsPoints() const
	{
		return *m_EdgeIndexsPoints;
	}

	const auto& GetEdgeIndexsFaces() const
	{
		return *m_EdgeIndexsFaces;
	}

	const auto& GetCentre() const
	{
		return m_Centre;
	}

	SatInputShape(const std::vector<int>& edgeIndexesPoints, const std::vector<int>& edgeIndexesFaces,
		const std::vector<Vector3>& points, const std::vector<Vector3>& faceNormals,
		const std::vector<int>& faceP0Indexs, const Vector3& centre) :
		m_EdgeIndexsPoints(&edgeIndexesPoints),
		m_EdgeIndexsFaces(&edgeIndexesFaces),
		m_Points(&points),
		m_FaceNormals(&faceNormals),
		m_FaceP0Indexs(&faceP0Indexs),
		m_Centre(centre)
	{
	}

private:
	const std::vector<int>* m_EdgeIndexsPoints;
	const std::vector<int>* m_EdgeIndexsFaces;

	const std::vector<Vector3>* m_Points;
	const std::vector<Vector3>* m_FaceNormals;
	const std::vector<int>* m_FaceP0Indexs;

	Vector3 m_Centre;
};


class SatOptimisedCollisionDetection
{
private:
	void UpdateContactPlane(float sep, const Vector3& n, const Vector3& closest1, const Vector3& closest2)
	{
		// make the plane p0 the average of the closest points in the direction of n
		auto pen = MathU::Abs(sep);
		if (pen < m_CurrentContactPlane.GetPeneration())
		{
			auto dot1 = Vector3::Dot(n, closest1);
			auto dot2 = Vector3::Dot(n, closest2);
			m_CurrentContactPlane = dot1 > dot2 ? ContactPlane(dot2, dot1, n) : ContactPlane(dot1, dot2, n);
		}
	}

	bool CheckFaceNormals(const SatInputShape& shapeFaces, const SatInputShape& shapeOther)
	{
		auto& normals = shapeFaces.GetFaceNormals();
		auto& p0Indexs = shapeFaces.GetFaceP0Indexs();
		auto& points = shapeFaces.GetPoints();

		auto itNormal = normals.begin();
		auto itP0 = p0Indexs.begin();

		for (; itNormal != normals.end(); itNormal++, itP0++)
		{
			auto& n = *itNormal;
			auto& p0 = points[*itP0];

			auto support = shapeOther.FindSupportVertex(-n);
			auto sep = Vector3::Dot(n, support - p0);

			if (sep > 0.0f)
			{
				m_SeperationDirection = n;
				return true;
			}

			UpdateContactPlane(sep, n, p0, support);
		}
		return false;
	}

	bool EdgesAreOnMinowskiDiff(const Vector3& A, const Vector3& B, const Vector3& C, const Vector3& D) const
	{
		auto bCrossA = Vector3::Cross(B, A);
		auto dCrossC = Vector3::Cross(D, C);

		auto CBA = Vector3::Dot(C, bCrossA);
		auto DBA = Vector3::Dot(D, bCrossA);
		auto ADC = Vector3::Dot(A, dCrossC);
		auto BDC = Vector3::Dot(B, dCrossC);

		return (CBA * DBA < 0.0f) &&
			(ADC * BDC < 0.0f) && (CBA * BDC > 0.0f);
	}

	bool CheckEdgeNormals(const SatInputShape& shapeA, const SatInputShape& shapeB)
	{
		auto& edgesPointsA = shapeA.GetEdgeIndexsPoints();
		auto& edgesFacesA = shapeA.GetEdgeIndexsFaces();
		auto& pointsA = shapeA.GetPoints();
		auto& faceNormalsA = shapeA.GetFaceNormals();

		auto& edgesPointsB = shapeB.GetEdgeIndexsPoints();
		auto& edgesFacesB = shapeB.GetEdgeIndexsFaces();
		auto& pointsB = shapeB.GetPoints();
		auto& faceNormalsB = shapeB.GetFaceNormals();

		auto itPointsA = edgesPointsA.begin();
		auto itFacesA = edgesFacesA.begin();

		for (; itPointsA != edgesPointsA.end(); itPointsA += 2, itFacesA += 2)
		{
			auto n0A = faceNormalsA[*itFacesA];
			auto n1A = faceNormalsA[*(itFacesA + 1)];

			auto p0A = pointsA[*itPointsA];
			auto p1A = pointsA[*(itPointsA + 1)];

			auto itPointsB = edgesPointsB.begin();
			auto itFacesB = edgesFacesB.begin();

			for (; itPointsB != edgesPointsB.end(); itPointsB += 2, itFacesB += 2)
			{
				auto n0B = faceNormalsB[*itFacesB];
				auto n1B = faceNormalsB[*(itFacesB + 1)];

				auto p0B = pointsB[*itPointsB];
				auto p1B = pointsB[*(itPointsB + 1)];

				if (EdgesAreOnMinowskiDiff(n0A, n1A, -n0B, -n1B))
				{
					// The face normal points from A to B
					auto sepNormal = Vector3::Cross(p1A - p0A, p1B - p0B).
						InDirectionOf(p0A - shapeA.GetCentre()).
						Normalized();

					auto sep = Vector3::Dot(sepNormal, p0B - p0A);

					if (sep > 0.0f)
					{
						m_SeperationDirection = sepNormal;
						return true;
					}

					UpdateContactPlane(sep, sepNormal, p0A, p0B);
				}
			}
		}
		return false;
	}

public:
	bool DetectCollision(const SatInputShape& shapeA, const SatInputShape& shapeB)
	{
		m_CurrentContactPlane = ContactPlane(0.0f, MathU::Infinity, Vector3::Zero());

		if (CheckFaceNormals(shapeA, shapeB))
			return false;

		if (CheckFaceNormals(shapeB, shapeA))
			return false;

		if (CheckEdgeNormals(shapeA, shapeB))
			return false;

		return true;
	}

	const Vector3& GetSeperationDirection() const
	{
		return m_SeperationDirection;
	}

	const ContactPlane& GetContactPlane() const
	{
		return m_CurrentContactPlane;
	}

private:
	ContactPlane m_CurrentContactPlane;
	Vector3 m_SeperationDirection;
};

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
				edgesInContact.emplace_back(e0);
				edgesInContact.emplace_back(e1);
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

	void FindPointInPolyContactPoints(const SatInputShape& shape, const std::vector<Vector3>& points)
	{
		for (auto& p : points)
		{
			if (PointIsInShape(shape, p))
				AddNewContactPoint(p);
		}
	}

public:
	const auto& FindContactPoints(const SatInputShape& shapeA, const SatInputShape& shapeB, const ContactPlane& contactPlane)
	{
		m_ContactPlaneNormal = contactPlane.GetNormal();

		m_ShapeAPointIsInContact.clear();
		m_ShapeBPointIsInContact.clear();
		m_ShapeAInContactPoints.clear();
		m_ShapeBInContactPoints.clear();
		m_ShapeAEdgesInContact.clear();
		m_ShapeBEdgesInContact.clear();
		m_ContactPoints.clear();

		FindPointsInContact(shapeA, m_ShapeAPointIsInContact, m_ShapeAInContactPoints, contactPlane);
		FindPointsInContact(shapeB, m_ShapeBPointIsInContact, m_ShapeBInContactPoints, contactPlane);

		FindEdgesInContact(shapeA, m_ShapeAEdgesInContact, m_ShapeAPointIsInContact);
		FindEdgesInContact(shapeB, m_ShapeBEdgesInContact, m_ShapeBPointIsInContact);

		FindEdgeOverlapContactPoints(shapeA, shapeB);
		FindPointInPolyContactPoints(shapeA, m_ShapeBInContactPoints);
		FindPointInPolyContactPoints(shapeB, m_ShapeAInContactPoints);

		return m_ContactPoints;
	}

private:
	std::vector<bool> m_ShapeAPointIsInContact;
	std::vector<bool> m_ShapeBPointIsInContact;

	std::vector<Vector3> m_ShapeAInContactPoints;
	std::vector<Vector3> m_ShapeBInContactPoints;

	std::vector<int> m_ShapeAEdgesInContact;
	std::vector<int> m_ShapeBEdgesInContact;

	std::vector<Vector3> m_ContactPoints;

	Vector3 m_ContactPlaneNormal;
};