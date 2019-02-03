#pragma once
#include <vector>
#include <array>
#include <memory>
#include "Shape.h"
#include "ShapePoint.h"
#include "ShapeEdge.h"
#include "EdgeEdgeRelationship.h"
#include "EdgeFaceIntersection.h"
#include "Constants.h"
#include "CollectionU.h"
#include "PoolOfRecyclables.h"
#include "DynamicTwoDArray.h"
#include "IntersectionLoop.h"
#include "DynamicTriangleArray.h"

class FaceFaceInteraction
{
public:
	FaceFaceInteraction()
	{
		Reset();
	}

	void Reset()
	{
		m_NumIntersRegistered = 0;
		m_IsUsed = false;
		m_Face1 = m_Face2 = nullptr;
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

	void SetFaces(const Face& f1, const Face& f2)
	{
		m_Face1 = &f1;
		m_Face2 = &f2;
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

	auto& GetFace1() const
	{
		return *m_Face1;
	}

	auto& GetFace2() const
	{
		return *m_Face2;
	}

private:
	EdgeFaceIntersection m_Inter1;
	EdgeFaceIntersection m_Inter2;
	int m_NumIntersRegistered;
	bool m_IsUsed;
	const Face* m_Face1 = nullptr;
	const Face* m_Face2 = nullptr;
};

static_assert(std::is_trivially_copyable<FaceFaceInteraction>::value, "Face face interaction must be trivially copyable.");

class MapToFaceFaceInteractions
{
private:
	struct InteractionSlot
	{
		const int Row;
		const int Col;

		InteractionSlot(int row, int col) : Row(row), Col(col)
		{
		}
	};

	void RegisterIntersection(const EdgeFaceIntersection& inter, const Face& faceA, const Face& faceB)
	{
		auto row = faceA.GetHash();
		auto col = faceB.GetHash();

		auto& interac = m_Interactions.Get(row, col);
		interac.Register(inter);

		if (interac.NumIntersRegistered() == 1)
		{
			m_SlotsWithInters.emplace_back(InteractionSlot(row, col));
			interac.SetFaces(faceA, faceB);
		}
	}

public:
	void RegisterIntersection(const EdgeFaceIntersection& inter)
	{
		auto& pf = inter.GetFace();
		auto& pe = inter.GetEdge();

		RegisterIntersection(inter, pf, pe.GetFace1());
		RegisterIntersection(inter, pf, pe.GetFace2());
	}

	void Clear()
	{
		for (auto& slot : m_SlotsWithInters)
			m_Interactions.Get(slot.Row, slot.Col).Reset();

		m_SlotsWithInters.clear();
	}

	FaceFaceInteraction* GetUnUsedInteraction()
	{
		for (auto& slot : m_SlotsWithInters)
		{
			auto& interac = m_Interactions.Get(slot.Row, slot.Col);
			if (!interac.IsUsed())
				return &interac;
		}
		return nullptr;
	}

	auto& GetInteraction(const Face& faceA, const Face& faceB)
	{
		return m_Interactions.Get(faceA.GetHash(), faceB.GetHash());
	}

private:
	DynamicTriangleArray<FaceFaceInteraction> m_Interactions;
	std::vector<InteractionSlot> m_SlotsWithInters;
};

class IntersectionLinker
{
private:
	EdgeFaceIntersection GetNextIntPoint(const FaceFaceInteraction& prevR, const FaceFaceInteraction& currR) const
	{
		return prevR.Contains(currR.GetInter1()) ? currR.GetInter2() : currR.GetInter1();
	}

	FaceFaceInteraction& GetNextInteration(const FaceFaceInteraction& prevR, const FaceFaceInteraction& currR)
	{
		auto endIntPoint = GetNextIntPoint(prevR, currR);

		auto& face1 = endIntPoint.GetFace();

		auto& e = endIntPoint.GetEdge();
		auto& oldFace = e.IsAttachedTo(currR.GetFace1()) ? currR.GetFace1() : currR.GetFace2();
		auto& face2 = e.GetOther(oldFace);

		return m_FaceFaceInteractions.GetInteraction(face1, face2);
	}

	FaceFaceInteraction& InitialPrevInteration(const FaceFaceInteraction& startR)
	{
		auto& i = startR.GetInter1();

		auto& face1 = i.GetFace(); // The pierced face is in startR

		auto& other = (&face1 == &startR.GetFace1()) ? startR.GetFace2() : startR.GetFace1(); // Now get the other face in startR
		auto& face2 = i.GetEdge().GetOther(other);

		return m_FaceFaceInteractions.GetInteraction(face1, face2);
	}

	bool FindLoop(IntersectionLoop& loop, FaceFaceInteraction& startR)
	{
		auto prevR = &InitialPrevInteration(startR);
		auto currR = &startR;

		while (!currR->IsUsed())
		{
			if (!currR->IsComplete())
				return false;

			currR->MarkUsed();
			loop.AddIntersection(GetNextIntPoint(*prevR, *currR));

			auto temp = prevR;
			prevR = currR;
			currR = &GetNextInteration(*temp, *currR);
		}

		return &startR == currR;
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

		auto start = m_FaceFaceInteractions.GetUnUsedInteraction();
		while (start)
		{
			auto& loop = m_IntersectionLoops->Recycle();
			loop.Clear();

			if (!FindLoop(loop, *start))
				return false;

			loops.emplace_back(&loop);
			start = m_FaceFaceInteractions.GetUnUsedInteraction();
		}
		return true;
	}

	void RegisterIntersection(const EdgeFaceIntersection& inter)
	{
		m_FaceFaceInteractions.RegisterIntersection(inter);
	}

	void Clear()
	{
		m_FaceFaceInteractions.Clear();
	}

private:
	MapToFaceFaceInteractions m_FaceFaceInteractions;
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

			m_PointPlaneRelationshipMap.SetRelationship(*p, r);
		}
	}

	void FindIntersection(ShapeEdge& piercingEdge)
	{
		auto& p0 = piercingEdge.GetP0();
		auto& p1 = piercingEdge.GetP1();

		if (m_PointPlaneRelationshipMap.GetRelationship(p1) != m_PointPlaneRelationshipMap.GetRelationship(p0))
		{
			Vector3 intPoint;
			assert(Vector3::LinePlaneIntersection(m_SplitterFace.GetPlaneP0(), m_SplitterFace.GetNormal(), p0.GetPoint(), p1.GetPoint(), intPoint));

			m_Linker.RegisterIntersection(EdgeFaceIntersection(m_SplitterFace, piercingEdge, intPoint));
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

	template<class T>
	void AssignHashes(const std::vector<T*>& objs)
	{
		for (auto o : objs)
			o->AssignHash();
	}

	template<class T>
	void UnAssignHashes(const std::vector<T*>& objs)
	{
		for (auto o : objs)
			o->ResetHash();
	}

	void AssignHashes(const Shape& s)
	{
		Face::ResetNextHashCounter();
		ShapePoint::ResetNextHashCounter();

		m_SplitterFace.AssignHash();
		AssignHashes(s.GetPointObjects());
		AssignHashes(s.GetFaces());
	}

	void UnAssignHashes(const Shape& s)
	{
		m_SplitterFace.ResetHash();
		UnAssignHashes(s.GetPointObjects());
		UnAssignHashes(s.GetFaces());
	}

	void InitSplitterFace(const Plane& splitPlane)
	{
		assert(false);
	}

public:
	CleanIntersectionFinder()
	{
	}

	bool FindCleanIntersections(const Shape& shape, const Plane& splitPlane, std::vector<IntersectionLoop*>& loops)
	{
		AssignHashes(shape);

		DeterminePointPlaneRelationships(shape.GetPointObjects(), splitPlane);
		FindIntersections(shape.GetEdgeObjects());

		auto valid = m_IntersectionCount > 0 ? LinkIntersections(loops) : false;
		m_Linker.Clear();

		UnAssignHashes(shape);

		return valid;
	}

private:
	Face m_SplitterFace;
	IntersectionLinker m_Linker;
	MapToPointPlaneRelationship m_PointPlaneRelationshipMap;
	int m_IntersectionCount;
};
