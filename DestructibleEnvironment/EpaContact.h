#pragma once
#include <array>
#include <vector>
#include "GjkCollisionDetection.h"
#include "Vector3.h"
#include "CollisionData.h"

class EpaContact
{
private:
	class MinowskiDifferenceFace
	{
	public:
		MinowskiDifferenceFace(const Vector3& p0, const Vector3& p1, const Vector3& p2, const Vector3& normalRef) :
			m_Points{p0, p1, p2}
		{
			m_N = Vector3::Cross(p1 - p0, p2 - p0).InDirectionOf(normalRef).Normalized();
		}

		bool CanExpand(const GjkInputShape& shapeA, const GjkInputShape& shapeB, Vector3& sv) const
		{
			static constexpr auto tol = 0.001f;

			sv = GjkCollisionDetection::GetMinowskiDiffSupportVertex(shapeA, shapeB, m_N);
			return MathU::Abs(Vector3::Dot(m_Points[0], m_N) - Vector3::Dot(sv, m_N)) > tol;
		}

		MinowskiDifferenceFace Expand(const Vector3& sv, int fromEdge) const
		{
			return MinowskiDifferenceFace(m_Points[fromEdge], m_Points[(fromEdge + 1) % 3], sv, m_N);
		}

		float ProjectedDistanceFromOrigin() const
		{
			return MathU::Abs(Vector3::Dot(m_Points[0], m_N));
		}

		ContactManifold ToContact() const
		{

		}

	private:
		std::array<Vector3, 3> m_Points;
		Vector3 m_N;
	};

	void InitFaces(const GjkCollisionDetection::SetQ& q)
	{
		auto& qPoints = q.GetPoints();

		m_Faces.clear();
		m_Faces.emplace_back(MinowskiDifferenceFace(qPoints[0], qPoints[1], qPoints[2], qPoints[0] - qPoints[3]));
		m_Faces.emplace_back(MinowskiDifferenceFace(qPoints[1], qPoints[2], qPoints[3], qPoints[3] - qPoints[0]));
		m_Faces.emplace_back(MinowskiDifferenceFace(qPoints[0], qPoints[1], qPoints[3], qPoints[1] - qPoints[2]));
		m_Faces.emplace_back(MinowskiDifferenceFace(qPoints[0], qPoints[2], qPoints[3], qPoints[2] - qPoints[1]));
	}

	int FindClosestFaceToOrigin() const
	{
		auto closestDist = MathU::Infinity;
		auto indexOfClosest = 0u;

		for (auto i = 0u; i < m_Faces.size(); i++)
		{
			auto dist = m_Faces[i].ProjectedDistanceFromOrigin();
			if (dist < closestDist)
			{
				closestDist = dist;
				indexOfClosest = i;
			}
		}
		return indexOfClosest;
	}

	void ExpandFace(int index, const Vector3& sv)
	{
		auto toExpand = m_Faces[index];

		m_Faces[index] = toExpand.Expand(sv, 0);
		m_Faces.emplace_back(toExpand.Expand(sv, 1));
		m_Faces.emplace_back(toExpand.Expand(sv, 2));
	}

public:
	ContactManifold FindContact(const GjkInputShape& shapeA, const GjkInputShape& shapeB,
		const GjkCollisionDetection::SetQ& q)
	{
		Vector3 sv;
		while (true)
		{
			auto closest = FindClosestFaceToOrigin();

			if (!m_Faces[closest].CanExpand(shapeA, shapeB, sv))
				return m_Faces[closest].ToContact();

			ExpandFace(closest, sv);
		}
	}

private:
	std::vector<MinowskiDifferenceFace> m_Faces;
};
