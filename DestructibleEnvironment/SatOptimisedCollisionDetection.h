#pragma once
#include <vector>
#include "Vector3.h"
#include "CollisionData.h"

class SatInputShape
{
public:
	Vector3 FindSupportVertex(const Vector3& dir) const
	{

	}

	const auto& GetPoints() const
	{
		return *m_Points;
	}

	const auto& GetFaceNormals() const
	{
		return *m_FaceNormals;
	}

	const auto& GetFaceP0s() const
	{
		return *m_FaceP0s;
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

private:
	const std::vector<int>* m_EdgeIndexsPoints;
	const std::vector<int>* m_EdgeIndexsFaces;

	const std::vector<Vector3>* m_Points;
	const std::vector<Vector3>* m_FaceNormals;
	const std::vector<Vector3>* m_FaceP0s;

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
			auto contactPoint = closest1 + 0.5f * Vector3::Dot(n, closest2 - closest1) * n;
			m_CurrentContactPlane = ContactPlane(contactPoint, n, pen);
		}
	}

	bool CheckFaceNormals(const SatInputShape& shapeFaces, const SatInputShape& shapeOther)
	{
		auto& normals = shapeFaces.GetFaceNormals();
		auto& p0s = shapeFaces.GetFaceP0s();

		auto itNormal = normals.begin();
		auto itP0 = p0s.begin();

		for (; itNormal != normals.end(); itNormal++, itP0++)
		{
			auto& n = *itNormal;
			auto& p0 = *itP0;

			auto support = shapeOther.FindSupportVertex(-n);
			auto sep = Vector3::Dot(n, support - p0);

			if (sep > 0.0f)
				return true;

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

		for (; itPointsA != edgesPointsA.end(); itPointsA++, itFacesA++)
		{
			auto n0A = faceNormalsA[*itFacesA];
			auto n1A = faceNormalsA[*(itFacesA + 1)];

			auto p0A = pointsA[*itPointsA];
			auto p1A = pointsA[*(itPointsA + 1)];

			auto itPointsB = edgesPointsB.begin();
			auto itFacesB = edgesFacesB.begin();

			for (; itPointsB != edgesPointsB.end(); itPointsB++, itFacesB++)
			{
				auto n0B = faceNormalsB[*itFacesB];
				auto n1B = faceNormalsB[*(itFacesB + 1)];

				auto p0B = pointsB[*itPointsB];
				auto p1B = pointsB[*(itPointsB + 1)];

				if (EdgesAreOnMinowskiDiff(n0A, n1A, n0B, n1B))
				{
					// The face normal points from A to B
					auto sepNormal = Vector3::Cross(p1A - p0A, p1B - p0B).
						InDirectionOf(p0A - shapeA.GetCentre()).
						Normalized();

					auto sep = Vector3::Dot(sepNormal, p0B - p0A);

					if (sep > 0.0f)
						return true;

					UpdateContactPlane(sep, sepNormal, p0A, p0B);
				}
			}
		}
	}

public:
	bool DetectCollision(const SatInputShape& shapeA, const SatInputShape& shapeB)
	{
		m_CurrentContactPlane = ContactPlane(Vector3::Zero(), Vector3::Zero(), MathU::Infinity);
	}

	const Vector3& GetSeperationDirection() const
	{

	}

	const ContactPlane& GetContactPlane() const
	{

	}

private:
	ContactPlane m_CurrentContactPlane;
};