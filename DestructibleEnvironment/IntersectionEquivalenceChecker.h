#pragma once
#include <vector>
#include <memory>
#include "EdgeFaceIntersection.h"
#include "Face.h"
#include "ShapeEdge.h"
#include "Vector3.h"
#include "MathU.h"
#include "ShapePoint.h"

class EdgesPiercedFaces
{
public:
	bool HasIntersection(const EdgeFaceIntersection& inter) const
	{
		for (auto it = m_Intersections.begin(); it != m_Intersections.end(); it++)
		{
			if (inter == *it)
				return true;
		}
		return false;
	}

	void Clear()
	{
		m_Intersections.clear();
		m_Edge = nullptr;
		m_FaceNearestP0 = m_FaceNearestP1 = nullptr;
	}

	void Init(const ShapeEdge& edge)
	{
		Clear();
		m_Edge = &edge;
	}

	bool IsInitialised() const
	{
		return m_Edge;
	}

	void AddIntersection(const EdgeFaceIntersection& inter)
	{
		m_Intersections.emplace_back(inter);
	}

	void OnAllIntersectionsAdded()
	{
		auto P0 = m_Edge->GetP0().GetPoint();
		auto P1 = m_Edge->GetP1().GetPoint();

		auto closestDistP0 = MathU::Infinity;
		auto closestDistP1 = MathU::Infinity;

		for (auto it = m_Intersections.begin(); it != m_Intersections.end(); it++)
		{
			auto intPoint = it->GetIntPoint();

			auto distP0 = (P0 - intPoint).MagnitudeSqr();
			auto distP1 = (P1 - intPoint).MagnitudeSqr();

			if (distP0 < closestDistP0)
			{
				closestDistP0 = distP0;
				m_FaceNearestP0 = &it->GetFace();
			}

			if (distP1 < closestDistP1)
			{
				closestDistP1 = distP1;
				m_FaceNearestP1 = &it->GetFace();
			}
		}
	}

	bool ImpliesPointIsOutside(const ShapePoint& p) const
	{
		auto pEqualsP0 = &p == &m_Edge->GetP0();

		auto dirToPoint = pEqualsP0 ? -m_Edge->GetDirFromP0ToP1() : m_Edge->GetDirFromP0ToP1();
		auto n = pEqualsP0 ? m_FaceNearestP0->GetNormal() : m_FaceNearestP1->GetNormal();

		return Vector3::Dot(dirToPoint, n) > 0.0f;
	}

	bool MakesEdgeEdgeIntersection(const EdgeFaceIntersection& inter1, const EdgeFaceIntersection& inter2)
	{
		auto dir = m_Edge->GetDirFromP0ToP1();

		auto dot1 = Vector3::Dot(dir, inter1.GetFace().GetNormal());
		auto dot2 = Vector3::Dot(dir, inter2.GetFace().GetNormal());

		if (dot1 * dot2 < 0.0f)
			return false;

		auto edgeP0 = m_Edge->GetP0().GetPoint();

		auto dist1 = (inter1.GetIntPoint() - edgeP0).MagnitudeSqr();
		auto dist2 = (inter2.GetIntPoint() - edgeP0).MagnitudeSqr();

		auto distLow = MathU::Min(dist1, dist2);
		auto distHigh = MathU::Max(dist1, dist2);

		for (auto it = m_Intersections.begin(); it != m_Intersections.end(); it++)
		{
			auto& inter = *it;
			auto dist = (inter.GetIntPoint() - edgeP0).MagnitudeSqr();

			if (dist >= distLow && dist <= distHigh)
			{
				auto dot = Vector3::Dot(dir, inter.GetFace().GetNormal());

				if (dot * dot1 < 0.0f)
					return false;
			}
		}
		return true;
	}

private:
	const Face* m_FaceNearestP0 = nullptr;
	const Face* m_FaceNearestP1 = nullptr;

	const ShapeEdge* m_Edge = nullptr;

	std::vector<EdgeFaceIntersection> m_Intersections;
};

class IntersectionEquivalenceChecker
{
private:
	void EnsureIntersectionIsAdded(const EdgeFaceIntersection& inter)
	{
		auto& edge = inter.GetEdge();
		edge.TryAssignHash();

		auto h = edge.GetHash();

		while (m_EdgesPiercedFaces->NumRecycled() <= h)
			m_EdgesPiercedFaces->Recycle().Clear();

		auto& edgesInts = m_EdgesPiercedFaces->At(h);

		if (!edgesInts.IsInitialised())
			edgesInts.Init(edge);

		if (!edgesInts.HasIntersection(inter))
		{
			edgesInts.AddIntersection(inter);
			edgesInts.OnAllIntersectionsAdded();
		}
	}

public:
	void SetIntersections(const std::vector<EdgeFaceIntersection>& inters)
	{
		m_EdgesPiercedFaces->Reset();

		for (auto it = inters.begin(); it != inters.end(); it++)
		{
			auto& edge = it->GetEdge();
			
			edge.TryAssignHash();

			while (m_EdgesPiercedFaces->NumRecycled() <= edge.GetHash())
				m_EdgesPiercedFaces->Recycle().Clear();

			auto& edgesInts = m_EdgesPiercedFaces->At(edge.GetHash());

			if (!edgesInts.IsInitialised())
				edgesInts.Init(edge);

			edgesInts.AddIntersection(*it);
		}

		for (auto it = m_EdgesPiercedFaces->Begin(); it != m_EdgesPiercedFaces->End(); it++)
		{
			if (it->IsInitialised())
				it->OnAllIntersectionsAdded();
		}
	}

	bool AreEquivalent(const EdgeFaceIntersection& inter1, const EdgeFaceIntersection& inter2)
	{
		EnsureIntersectionIsAdded(inter1);
		EnsureIntersectionIsAdded(inter2);

		auto& e1 = inter1.GetEdge();
		auto& e2 = inter2.GetEdge();

		if (&e1 == &e2)
		{
			return m_EdgesPiercedFaces->At(e1.GetHash()).MakesEdgeEdgeIntersection(inter1, inter2);
		}
		else
		{
			auto conn = e1.GetConnection(e2);

			if (conn)
				return m_EdgesPiercedFaces->At(e1.GetHash()).ImpliesPointIsOutside(*conn) != m_EdgesPiercedFaces->At(e2.GetHash()).ImpliesPointIsOutside(*conn);
		}

		return false;
	}

private:
	std::unique_ptr<PoolOfRecyclables<EdgesPiercedFaces>> m_EdgesPiercedFaces;
};
