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
#include "TwoDArray.h"
#include "IntersectionLoop.h"
#include "TriangleArray.h"

class FaceFaceInteraction
{
public:
	FaceFaceInteraction()
	{
		Reset();
	}

	void Reset()
	{
		m_IntersRegistered = 0;
		m_IsUsed = false;
	}

	void Register(const EdgeFaceIntersection& inter)
	{
		assert(m_IntersRegistered < 2);

		if (m_IntersRegistered == 0)
		{
			m_Inter1 = inter;
		}
		else
		{
			m_Inter2 = inter;
		}

		m_IntersRegistered++;
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

private:
	EdgeFaceIntersection m_Inter1;
	EdgeFaceIntersection m_Inter2;
	int m_IntersRegistered;
	bool m_IsUsed;
};

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

		m_Interactions.Get(row, col).Register(inter);
		m_SlotsWithInters.emplace_back(InteractionSlot(row, col));
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
				&interac;
		}
		return nullptr;
	}

private:
	TriangleArray<Constants::MaxNumFaces, FaceFaceInteraction> m_Interactions;
	std::vector<InteractionSlot> m_SlotsWithInters;
};

class IntersectionLinker
{
private:

public:
	IntersectionLinker()
	{
		m_IntersectionLoops = std::unique_ptr<PoolOfRecyclables<IntersectionLoop>>(
			new PoolOfRecyclables<IntersectionLoop>(3));
	}

	bool FindLoops(std::vector<IntersectionLoop*>& loops)
	{
		
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
	const EdgeEdgeRelationship& GetEdgeEdgeRelationship(const ShapeEdge& edgeFaces, const ShapeEdge& piercingEdge) const
	{
		return m_EdgeEdgeRelationships.Get(edgeFaces.GetHash(), piercingEdge.GetHash());
	}

	void SetEdgeEdgeRelationship(const ShapeEdge& edgeFaces, const ShapeEdge& piercingEdge, const EdgeEdgeRelationship& r)
	{
		m_EdgeEdgeRelationships.Get(edgeFaces.GetHash(), piercingEdge.GetHash()) = r;
	}

	EdgeEdgeRelationship MakeConsistentRelationship(const EdgeEdgeRelationship& rConstrained, const ShapeEdge& piercingEdge, const ShapeEdge& edgeFaces)
	{
		auto& bridged = *rConstrained.GetBridgedFace(m_PointPlaneRelationshipMap);
		return EdgeEdgeRelationship(edgeFaces, piercingEdge, bridged, rConstrained.ImpliesIntersectionFor(bridged), m_PointPlaneRelationshipMap);
	}

	void InitConsistentEdgeEdgeRelationship(const ShapeEdge& peInConstrined, const ShapeEdge& efInConstrined, const EdgeEdgeRelationship& rConstrained)
	{
		SetEdgeEdgeRelationship(efInConstrined, peInConstrined, rConstrained);
		SetEdgeEdgeRelationship(peInConstrined, efInConstrined, MakeConsistentRelationship(rConstrained, efInConstrined, peInConstrined));
	}

	void InitEdgeEdgeRelationship(const ShapeEdge& e0, const ShapeEdge& e1)
	{
		auto r0 = EdgeEdgeRelationship(e0, e1, m_PointPlaneRelationshipMap);
		auto r1 = EdgeEdgeRelationship(e1, e0, m_PointPlaneRelationshipMap);

		if (r0.IsConstrained() && r1.IsConstrained())
		{
			SetEdgeEdgeRelationship(e1, e0, r0);
			SetEdgeEdgeRelationship(e0, e1, r1);
		}
		else if (r0.IsConstrained())
		{
			InitConsistentEdgeEdgeRelationship(e0, e1, r0);
		}
		else if (r1.IsConstrained())
		{
			InitConsistentEdgeEdgeRelationship(e1, e0, r1);
		}
		else
		{
			auto b0 = r0.GetBridgedFace(m_PointPlaneRelationshipMap);
			auto b1 = r1.GetBridgedFace(m_PointPlaneRelationshipMap);
			if (b0)
			{
				InitConsistentEdgeEdgeRelationship(e0, e1, r0);
			}
			else if (b1)
			{
				InitConsistentEdgeEdgeRelationship(e1, e0, r1);
			}
			else
			{
				SetEdgeEdgeRelationship(e1, e0, r0);
				SetEdgeEdgeRelationship(e0, e1, r1);
			}
		}
	}

	void DeterminePointPlaneRelationships(const std::vector<ShapePoint*>& points, const std::vector<Face*>& faces)
	{
		for (auto f : faces)
		{
			auto n = f->GetNormal();
			auto p0 = f->GetPlaneP0();

			for (auto p : points)
			{
				auto comp = Vector3::Dot((p->GetPoint() - p0), n);
				auto r = comp >= 0.0f ? PointPlaneRelationship::PointsAbove : PointPlaneRelationship::PointsBelow;

				m_PointPlaneRelationshipMap.SetRelationship(*f, *p, r);
			}
		}
	}

	void DetermineEdgeEdgeRelationships(const std::vector<ShapeEdge*>& edgesA, const std::vector<ShapeEdge*>& edgesB)
	{
		for (auto edgeAp : edgesA)
		{
			auto& a = *edgeAp;

			for (auto edgeBp : edgesB)
			{
				InitEdgeEdgeRelationship(a, *edgeBp);
			}
		}
	}

	void FindIntersection(Face& face, ShapeEdge& piercingEdge)
	{
		auto& facesEdges = face.GetEdgeObjects();

		for (auto faceEdge : facesEdges)
		{
			if (!GetEdgeEdgeRelationship(*faceEdge, piercingEdge).ImpliesIntersectionFor(face))
				return;
		}

		Vector3 intPoint;
		assert(Vector3::LinePlaneIntersection(face.GetPlaneP0(), face.GetNormal(), piercingEdge.GetP0().GetPoint(), piercingEdge.GetP1().GetPoint(), intPoint));

		m_Linker.RegisterIntersection(EdgeFaceIntersection(face, piercingEdge, intPoint));
	}

	void FindIntersections(const std::vector<Face*>& faces, const std::vector<ShapeEdge*>& piercingEdges)
	{
		for (auto f : faces)
			for (auto e : piercingEdges)
				FindIntersection(*f, *e);
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

	}

	void UnAssignHashes(const Shape& s)
	{

	}

	void AssignHashes(const Shape& shapeA, const Shape& shapeB)
	{
		Face::ResetNextHashCounter();
		ShapeEdge::ResetNextHashCounter();
		ShapePoint::ResetNextHashCounter();

		AssignHashes(shapeA);
		AssignHashes(shapeB);
	}

	void UnAssignHashes(const Shape& shapeA, const Shape& shapeB)
	{
		Face::ResetNextHashCounter();
		ShapeEdge::ResetNextHashCounter();
		ShapePoint::ResetNextHashCounter();

		UnAssignHashes(shapeA);
		UnAssignHashes(shapeB);
	}

public:
	CleanIntersectionFinder()
	{
	}

	// Both shapes must have the same local space.
	// Loops are recycled so must be used before the next call.
	bool FindCleanIntersections(const Shape& shapeA, const Shape& shapeB, std::vector<IntersectionLoop*>& loops)
	{
		AssignHashes(shapeA, shapeB);

		DeterminePointPlaneRelationships(shapeA.GetPointObjects(), shapeB.GetFaces());
		DeterminePointPlaneRelationships(shapeB.GetPointObjects(), shapeA.GetFaces());

		DetermineEdgeEdgeRelationships(shapeB.GetEdgeObjects(), shapeA.GetEdgeObjects());

		FindIntersections(shapeA.GetFaces(), shapeB.GetEdgeObjects());
		FindIntersections(shapeB.GetFaces(), shapeA.GetEdgeObjects());

		auto valid = LinkIntersections(loops);
		m_Linker.Clear();
		UnAssignHashes(shapeA, shapeB);

		return valid;
	}

private:
	IntersectionLinker m_Linker;
	MapToPointPlaneRelationship m_PointPlaneRelationshipMap;
	TwoDArray<Constants::MaxNumEdges, Constants::MaxNumEdges, EdgeEdgeRelationship> m_EdgeEdgeRelationships;
};
