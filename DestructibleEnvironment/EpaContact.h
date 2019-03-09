#pragma once
#include <array>
#include <vector>
#include "GjkCollisionDetection.h"
#include "Vector3.h"
#include "CollisionData.h"
#include "DynamicTriangleArray.h"

class EpaContact
{
private:
	using MinowPoint = GjkCollisionDetection::MinowskiDiffPoint;

	class Edge
	{
	public:
		int P0;
		int P1;

		Edge(int p0, int p1) : P0(p0), P1(p1)
		{
		}
	};

	class MinowskiDifferenceFace
	{
	private:
		Vector3 FindPointToRefFrom(const std::vector<MinowPoint>& points, const Vector3& p0, const Vector3& n) const
		{
			auto maxComp = MathU::NegativeInfinity;
			const Vector3* refP = nullptr;

			for (auto& p : points)
			{
				auto comp = MathU::Abs(Vector3::Dot(p.Value - p0, n));
				if (comp > maxComp)
				{
					maxComp = comp;
					refP = &(p.Value);
				}
			}
			return *refP;
		}

	public:
		MinowskiDifferenceFace(int p0, int p1, int p2, const std::vector<MinowPoint>& points) :
			m_P0(p0), m_P1(p1), m_P2(p2)
		{
			auto p0Val = points[p0].Value;

			auto n = Vector3::Cross(points[p1].Value - p0Val, points[p2].Value - p0Val);

			if (MathU::Abs(Vector3::Dot(n, p0Val)) > 0.001f)
			{
				m_N = n.InDirectionOf(p0Val).Normalized();
			}
			else
			{
				auto refP = FindPointToRefFrom(points, p0Val, n);
				m_N = n.InDirectionOf(p0Val - refP).Normalized();
			}

			m_D = Vector3::Dot(m_N, p0Val);
		}

		bool CanExpand(const GjkInputShape& shapeA, const GjkInputShape& shapeB, MinowPoint& sv) const
		{
			static constexpr auto tol = 0.0001f;

			sv = GjkCollisionDetection::GetMinowskiDiffSupportVertex(shapeA, shapeB, m_N);

			auto x = m_D;
			auto y = Vector3::Dot(sv.Value, m_N);

			return MathU::Abs(x - y) > tol;
		}

		float ProjectedDistanceFromOrigin() const
		{
			return m_D;
		}

		const auto& Normal() const
		{
			return m_N;
		}

		ContactPlane ToContact(const GjkInputShape& shapeA, const GjkInputShape& shapeB, const std::vector<MinowPoint>& points) const
		{
			auto p = m_D * m_N;

			float a, b, c;
			Vector3::CalculateBaryCords(points[m_P0].Value, points[m_P1].Value, points[m_P2].Value, p, a, b, c);

			auto pointOnA = a * shapeA.GetPointAt(points[m_P0].OriginalA) + b * shapeA.GetPointAt(points[m_P1].OriginalA) + c * shapeA.GetPointAt(points[m_P2].OriginalA);
			auto pointOnB = a * shapeB.GetPointAt(points[m_P0].OriginalB) + b * shapeB.GetPointAt(points[m_P1].OriginalB) + c * shapeB.GetPointAt(points[m_P2].OriginalB);

			auto pen = (pointOnA - pointOnB).Magnitude();

			return ContactPlane((pointOnA + pointOnB) / 2.0f, m_N, pen);
		}

		const auto& P0() const
		{
			return m_P0;
		}

		const auto& P1() const
		{
			return m_P1;
		}

		const auto& P2() const
		{
			return m_P2;
		}

	private:
		int m_P0;
		int m_P1;
		int m_P2;
		Vector3 m_N;
		float m_D;
	};

	void InitPoints(const GjkCollisionDetection::SetQ& q)
	{
		m_Points.clear();
		m_Points.insert(m_Points.begin(), q.GetPoints().begin(), q.GetPoints().end());
	}

	void InitFaces()
	{
		m_Faces.clear();
		m_Faces.emplace_back(MinowskiDifferenceFace(0, 1, 2, m_Points));
		m_Faces.emplace_back(MinowskiDifferenceFace(1, 2, 3, m_Points));
		m_Faces.emplace_back(MinowskiDifferenceFace(0, 1, 3, m_Points));
		m_Faces.emplace_back(MinowskiDifferenceFace(0, 2, 3, m_Points));
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

	void RemoveFace(const MinowskiDifferenceFace& f)
	{
		m_TimesRemovedFrom.Get(f.P0(), f.P1())++;
		if (m_TimesRemovedFrom.Get(f.P0(), f.P1()) == 1)
			m_EdgesRemovedFrom.emplace_back(Edge(f.P0(), f.P1()));

		m_TimesRemovedFrom.Get(f.P1(), f.P2())++;
		if (m_TimesRemovedFrom.Get(f.P1(), f.P2()) == 1)
			m_EdgesRemovedFrom.emplace_back(Edge(f.P1(), f.P2()));

		m_TimesRemovedFrom.Get(f.P2(), f.P0())++;
		if (m_TimesRemovedFrom.Get(f.P2(), f.P0()) == 1)
			m_EdgesRemovedFrom.emplace_back(Edge(f.P2(), f.P0()));
	}

	void RemoveFaces(const MinowPoint& sv)
	{
		m_FacesNext.clear();
		m_EdgesRemovedFrom.clear();

		auto svPoint = sv.Value;

		for (auto& f : m_Faces)
		{
			auto& n = f.Normal();
			auto p0 = f.ProjectedDistanceFromOrigin() * n;

			if (Vector3::Dot(svPoint - p0, n) > 0.0f)
			{
				RemoveFace(f);
			}
			else
			{
				m_FacesNext.emplace_back(f);
			}
		}

		m_Faces.swap(m_FacesNext);
	}

	void CreateNewFaces(int newPoint)
	{
		for (auto& e : m_EdgesRemovedFrom)
		{
			if (m_TimesRemovedFrom.Get(e.P0, e.P1) == 1)
			{
				m_Faces.emplace_back(MinowskiDifferenceFace(e.P0, e.P1, newPoint, m_Points));
			}

			m_TimesRemovedFrom.Get(e.P0, e.P1) = 0;
		}
	}

	void Expand(const MinowPoint& sv)
	{
		m_Points.emplace_back(sv);
		RemoveFaces(sv);
		CreateNewFaces(m_Points.size() - 1);
	}

public:
	ContactPlane FindContact(const GjkInputShape& shapeA, const GjkInputShape& shapeB,
		const GjkCollisionDetection::SetQ& q)
	{
		InitPoints(q);
		InitFaces();

		MinowPoint sv;
		while (true)
		{
			auto closest = FindClosestFaceToOrigin();

			if (!m_Faces[closest].CanExpand(shapeA, shapeB, sv))
				return m_Faces[closest].ToContact(shapeA, shapeB, m_Points);

			Expand(sv);
		}
	}

private:
	std::vector<MinowskiDifferenceFace> m_Faces;
	std::vector<MinowskiDifferenceFace> m_FacesNext;
	std::vector<MinowPoint> m_Points;
	DynamicTriangleArray<int> m_TimesRemovedFrom;
	std::vector<Edge> m_EdgesRemovedFrom;
};
