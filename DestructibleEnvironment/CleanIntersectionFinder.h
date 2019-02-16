#pragma once
#include <vector>
#include <array>
#include <memory>
#include "Shape.h"
#include "ShapePoint.h"
#include "ShapeEdge.h"
#include "EdgeFaceIntersection.h"
#include "Constants.h"
#include "CollectionU.h"
#include "PoolOfRecyclables.h"
#include "DynamicTwoDArray.h"
#include "IntersectionLoop.h"
#include "DynamicTriangleArray.h"

class IntersectionPair
{
public:
	IntersectionPair()
	{
		Reset();
	}

	void Reset()
	{
		m_NumIntersRegistered = 0;
		m_IsUsed = false;
		m_Face = nullptr;
	}

	void Register(const EdgeFaceIntersection& inter)
	{
		assert(m_NumIntersRegistered < 2);

		if (m_NumIntersRegistered == 0)
		{
			m_Inter1 = inter;
		}
		else
		{
			m_Inter2 = inter;
		}

		m_NumIntersRegistered++;
	}

	void SetFace(const Face& f)
	{
		m_Face = &f;
	}

	const auto& GetInter1() const
	{
		return m_Inter1;
	}

	const auto& GetInter2() const
	{
		return m_Inter2;
	}

	bool IsUsed() const
	{
		return m_IsUsed;
	}

	void MarkUsed()
	{
		m_IsUsed = true;
	}

	int NumIntersRegistered() const
	{
		return m_NumIntersRegistered;
	}
	
	bool Contains(const EdgeFaceIntersection& inter) const
	{
		return inter == m_Inter1 || inter == m_Inter2;
	}

	bool IsComplete() const
	{
		return m_NumIntersRegistered == 2;
	}

	auto& GetFace() const
	{
		return *m_Face;
	}

	const auto& GetOther(const EdgeFaceIntersection& inter)
	{
		if (inter == m_Inter1)
			return m_Inter2;

		if (inter == m_Inter2)
			return m_Inter1;

		assert(false);
		return m_Inter1;
	}

private:
	EdgeFaceIntersection m_Inter1;
	EdgeFaceIntersection m_Inter2;
	int m_NumIntersRegistered;
	bool m_IsUsed;
	const Face* m_Face = nullptr;
};

static_assert(std::is_trivially_copyable<IntersectionPair>::value, "Intersection pair must be trivially copyable.");

class MapToIntersectionPairs
{
private:
	void RegisterIntersection(const EdgeFaceIntersection& inter, Face& f)
	{
		f.TryAssignHash();

		auto i = f.GetHash();
		auto& pair = m_InteractionPairs[i];

		pair.Register(inter);

		if (pair.NumIntersRegistered() == 1)
		{
			m_SlotsWithInters.emplace_back(i);
			pair.SetFace(f);
		}
	}

public:
	void RegisterIntersection(const EdgeFaceIntersection& inter)
	{
		auto& pe = inter.GetEdge();

		RegisterIntersection(inter, pe.GetFace1());
		RegisterIntersection(inter, pe.GetFace2());
	}

	void Clear()
	{
		for (auto& slot : m_SlotsWithInters)
			m_InteractionPairs[slot].Reset();

		m_SlotsWithInters.clear();
	}

	IntersectionPair* GetUnUsedPair()
	{
		for (auto& slot : m_SlotsWithInters)
		{
			auto& pair = m_InteractionPairs[slot];
			if (!pair.IsUsed())
				return &pair;
		}
		return nullptr;
	}

	auto& GetIntersectionPair(const Face& face)
	{
		return m_InteractionPairs[face.GetHash()];
	}

private:
	DynamicArray<IntersectionPair> m_InteractionPairs;
	std::vector<int> m_SlotsWithInters;
};

class IntersectionLinker
{
private:
	bool FindLoop(IntersectionLoop& loop, IntersectionPair& startPair)
	{
		auto currPair = &startPair;
		auto prevInter = startPair.GetInter1(); // Could be either.

		while (!currPair->IsUsed())
		{
			auto& inter = currPair->GetOther(prevInter);
			auto& nextFace = inter.GetEdge().GetOther(currPair->GetFace());

			currPair->MarkUsed();
			loop.AddIntersection(inter);

			currPair = &m_InterPairs.GetIntersectionPair(nextFace);
			prevInter = inter;
		}

		return currPair == &startPair;
	}

public:
	IntersectionLinker()
	{
		m_IntersectionLoops = std::unique_ptr<PoolOfRecyclables<IntersectionLoop>>(
			new PoolOfRecyclables<IntersectionLoop>(3));
	}

	bool FindLoops(std::vector<IntersectionLoop*>& loops)
	{
		m_IntersectionLoops->Reset();

		auto start = m_InterPairs.GetUnUsedPair();
		while (start)
		{
			auto& loop = m_IntersectionLoops->Recycle();
			loop.Clear();

			if (!FindLoop(loop, *start))
				return false;

			loops.emplace_back(&loop);
			start = m_InterPairs.GetUnUsedPair();
		}
		return true;
	}

	void RegisterIntersection(const EdgeFaceIntersection& inter)
	{
		m_InterPairs.RegisterIntersection(inter);
	}

	void Clear()
	{
		m_InterPairs.Clear();
	}

private:
	MapToIntersectionPairs m_InterPairs;
	std::unique_ptr<PoolOfRecyclables<IntersectionLoop>> m_IntersectionLoops;
};

class CleanIntersectionFinder
{
private:
	void DeterminePointPlaneRelationships(const std::vector<ShapePoint*>& points, const Plane& plane)
	{
		auto planeP0 = plane.GetP0();
		auto planeN = plane.GetNormal();

		for (auto p : points)
		{
			auto comp = Vector3::Dot((p->GetPoint() - planeP0), planeN);
			auto r = comp >= 0.0f ? PointPlaneRelationship::PointsAbove : PointPlaneRelationship::PointsBelow;

			p->TryAssignHash();
			m_PointPlaneRelationshipMap.SetRelationship(*p, r);
		}
	}

	void FindIntersection(ShapeEdge& piercingEdge)
	{
		auto& p0 = piercingEdge.GetP0();
		auto& p1 = piercingEdge.GetP1();

		auto p0Rel = m_PointPlaneRelationshipMap.GetRelationship(p0);
		auto p1Rel = m_PointPlaneRelationshipMap.GetRelationship(p1);

		if (p1Rel != p0Rel)
		{
			// Always find the intersection point with above and below points in the same order so
			// that when the same edge on two different shapes is split, the int points are exacty
			// the same.
			Vector3 pAbove;
			Vector3 pBelow;
			if (p1Rel == PointPlaneRelationship::PointsAbove)
			{
				pAbove = p1.GetPoint();
				pBelow = p0.GetPoint();
			}
			else
			{
				pAbove = p0.GetPoint();
				pBelow = p1.GetPoint();
			}

			Vector3 intPoint;
			assert(Vector3::LinePlaneIntersection(m_SplitPlane.GetP0(), m_SplitPlane.GetNormal(), pAbove, pBelow, intPoint));

			static Face nullFace;
			m_Linker.RegisterIntersection(EdgeFaceIntersection(nullFace, piercingEdge, intPoint));
			m_IntersectionCount++;
		}
	}

	void FindIntersections(const std::vector<ShapeEdge*>& piercingEdges)
	{
		m_IntersectionCount = 0;

		for (auto e : piercingEdges)
			FindIntersection(*e);
	}

	bool LinkIntersections(std::vector<IntersectionLoop*>& loops)
	{
		return m_Linker.FindLoops(loops);
	}

public:
	enum class Result
	{
		IntersectionsFound,
		ShapesAllAbove,
		ShapesAllBelow,
		Error
	};

	CleanIntersectionFinder()
	{
	}

	Result FindCleanIntersections(const Shape& shape, const Plane& splitPlane, std::vector<IntersectionLoop*>& loops)
	{
		m_SplitPlane = splitPlane;

		DeterminePointPlaneRelationships(shape.GetPointObjects(), splitPlane);
		FindIntersections(shape.GetEdgeObjects());

		Result res;
		if (m_IntersectionCount > 0)
		{
			res = LinkIntersections(loops) ? Result::IntersectionsFound : Result::Error;
			m_Linker.Clear();
		}
		else
		{
			res = m_PointPlaneRelationshipMap.GetRelationship(*shape.GetPointObjects()[0]) == PointPlaneRelationship::PointsAbove ?
				Result::ShapesAllAbove :
				Result::ShapesAllBelow;
		}

		return res;
	}

	const auto& GetPointPlaneMap() const
	{
		return m_PointPlaneRelationshipMap;
	}

private:
	IntersectionLinker m_Linker;
	MapToPointPlaneRelationship m_PointPlaneRelationshipMap;
	int m_IntersectionCount;
	Plane m_SplitPlane;
};
