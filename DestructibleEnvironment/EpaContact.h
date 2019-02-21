#pragma once
#include <array>
#include <vector>
#include "GjkCollisionDetection.h"
#include "Vector3.h"
#include "CollisionData.h"

class EpaContact
{
private:
	using MinowPoint = GjkCollisionDetection::MinowskiDiffPoint;

	class MinowskiDifferenceFace
	{
	public:
		MinowskiDifferenceFace(const MinowPoint& p0, const MinowPoint& p1, const MinowPoint& p2, const Vector3& normalRef) :
			m_Points{p0, p1, p2}
		{
			m_N = Vector3::Cross(p1.Value - p0.Value, p2.Value - p0.Value).InDirectionOf(normalRef).Normalized();
		}

		bool CanExpand(const GjkInputShape& shapeA, const GjkInputShape& shapeB, MinowPoint& sv) const
		{
			static constexpr auto tol = 0.001f;

			sv = GjkCollisionDetection::GetMinowskiDiffSupportVertex(shapeA, shapeB, m_N);
			return MathU::Abs(Vector3::Dot(m_Points[0].Value, m_N) - Vector3::Dot(sv.Value, m_N)) > tol;
		}

		MinowskiDifferenceFace Expand(const MinowPoint& sv, int fromEdge) const
		{
			return MinowskiDifferenceFace(m_Points[fromEdge], m_Points[(fromEdge + 1) % 3], sv, m_N);
		}

		float ProjectedDistanceFromOrigin() const
		{
			return MathU::Abs(Vector3::Dot(m_Points[0].Value, m_N));
		}

		ContactManifold ToContact(const GjkInputShape& shapeA, const GjkInputShape& shapeB) const
		{
			auto p = Vector3::Dot(m_Points[0].Value, m_N) * m_N;

			float a, b, c;
			Vector3::CalculateBaryCords(m_Points[0].Value, m_Points[1].Value, m_Points[2].Value, p, a, b, c);

			auto pointOnA = a * shapeA.GetPointAt(m_Points[0].OriginalA) + b * shapeA.GetPointAt(m_Points[1].OriginalA) + c * shapeA.GetPointAt(m_Points[2].OriginalA);
			auto pointOnB = a * shapeB.GetPointAt(m_Points[0].OriginalB) + b * shapeB.GetPointAt(m_Points[1].OriginalB) + c * shapeB.GetPointAt(m_Points[2].OriginalB);

			return ContactManifold((pointOnA + pointOnB) / 2.0f, m_N);
		}

	private:
		std::array<MinowPoint, 3> m_Points;
		Vector3 m_N;
	};

	void InitFaces(const GjkCollisionDetection::SetQ& q)
	{
		auto& qPoints = q.GetPoints();

		m_Faces.clear();
		m_Faces.emplace_back(MinowskiDifferenceFace(qPoints[0], qPoints[1], qPoints[2], qPoints[0].Value - qPoints[3].Value));
		m_Faces.emplace_back(MinowskiDifferenceFace(qPoints[1], qPoints[2], qPoints[3], qPoints[3].Value - qPoints[0].Value));
		m_Faces.emplace_back(MinowskiDifferenceFace(qPoints[0], qPoints[1], qPoints[3], qPoints[1].Value - qPoints[2].Value));
		m_Faces.emplace_back(MinowskiDifferenceFace(qPoints[0], qPoints[2], qPoints[3], qPoints[2].Value - qPoints[1].Value));
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

	void ExpandFace(int index, const MinowPoint& sv)
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
		InitFaces(q);

		MinowPoint sv;
		while (true)
		{
			auto closest = FindClosestFaceToOrigin();

			if (!m_Faces[closest].CanExpand(shapeA, shapeB, sv))
				return m_Faces[closest].ToContact(shapeA, shapeB);

			ExpandFace(closest, sv);
		}
	}

private:
	std::vector<MinowskiDifferenceFace> m_Faces;
};
