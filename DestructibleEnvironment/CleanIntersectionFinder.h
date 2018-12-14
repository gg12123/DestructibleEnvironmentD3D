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

class IntersectionsByPiercingEdge
{

public:
	const auto& GetIntersections(const ShapeEdge& edge) const
	{
		return m_Intersections[edge.GetHash()];
	}

	bool GetAnyIntersection(EdgeFaceIntersection& inter) const
	{
		for (auto& index : m_IndexsWithIntersections)
		{
			if (m_Intersections[index].size() > 0u)
			{
				inter = m_Intersections[index][0];
				return true;
			}
		}
		return false;
	}

	void RemoveIntersection(const EdgeFaceIntersection& inter)
	{
		CollectionU::Remove(m_Intersections[inter.GetEdge().GetHash()], inter);

		// No point removing the index from 'indexs with intersections' becuase I
		// think it will slow things down.
	}

	void RegisterIntersection(const EdgeFaceIntersection& inter)
	{
		auto index = inter.GetEdge().GetHash();

		m_Intersections[index].emplace_back(inter);
		m_IndexsWithIntersections.emplace_back(index);
	}

	void Clear()
	{
		for (auto& index : m_IndexsWithIntersections)
			m_Intersections[index].clear();

		m_IndexsWithIntersections.clear();
	}

	// For when the caller knows that it has removed all the intersections
	void ClearFast()
	{
		m_IndexsWithIntersections.clear();
	}

private:
	std::vector<int> m_IndexsWithIntersections;
	std::array<std::vector<EdgeFaceIntersection>, Constants::MaxNumEdges> m_Intersections; // This is keyed by cut shape edges so could use less memory.
};

class IntersectionLinker
{
private:
	bool TargetReached(const std::vector<EdgeFaceIntersection>& targets, const EdgeFaceIntersection& curr) const
	{
		return CollectionU::Contains(targets, curr);
	}

	EdgeFaceIntersection CastToNext(const EdgeFaceIntersection& curr, Face& travelFace, Face& currOtherFace, const std::vector<EdgeFaceIntersection>& targets) const
	{
		auto& ownerOfEdge = curr.GetEdge().IsAttachedTo(travelFace) ? travelFace : currOtherFace;

		auto castOrigin = curr.GetIntPoint();
		auto castDir = Vector3::Cross(travelFace.GetNormal(), currOtherFace.GetNormal()).InDirectionOf(-ownerOfEdge.GetEdgeNormal(curr.GetEdge()));

		auto mag = castDir.Magnitude();
		assert(mag > 0.0f);
		castDir /= mag;

		auto hitTravelFace = travelFace.CastToEdgeInside(castOrigin, castDir);
		auto travelImpliedIntersection = EdgeFaceIntersection(currOtherFace, *hitTravelFace.Edge, hitTravelFace.IntPoint);

		if (TargetReached(targets, travelImpliedIntersection))
			return travelImpliedIntersection;

		auto hitOtherFace = currOtherFace.CastToEdgeInside(castOrigin, castDir);
		return EdgeFaceIntersection(travelFace, *hitOtherFace.Edge, hitOtherFace.IntPoint);
	}

	bool LinkToNext(const EdgeFaceIntersection& start, Face& travelFace, const std::vector<EdgeFaceIntersection>& targets, EdgeFaceIntersection& nextStart)
	{
		// TODO - This is fairly likly to fail. Need to use something like A*.

		auto curr = start;
		auto currOtherFace = &start.GetFace();
		auto targetReached = false;

		while (!targetReached)
		{
			curr = CastToNext(curr, travelFace, *currOtherFace, targets);
			targetReached = TargetReached(targets, curr);

			auto& currEdge = curr.GetEdge();
			if (currEdge.IsAttachedTo(*currOtherFace))
			{
				currOtherFace = &currEdge.GetOther(*currOtherFace);
			}
			else
			{
				if (!targetReached)
					return false;
			}
			m_CurrLoop->AddIntersection(curr);
		}
		nextStart = curr;
		return true;
	}

	void FindTargetIntersections(const Face& travelFace, IntersectionsByPiercingEdge& intersByEdge, std::vector<EdgeFaceIntersection>& targets)
	{
		for (auto edge : travelFace.GetEdgeObjects())
		{
			for (auto& inter : intersByEdge.GetIntersections(*edge))
				targets.emplace_back(inter);
		}
	}

public:
	bool FindLoop(IntersectionLoop& loop, IntersectionsByPiercingEdge& intersByEdge, const EdgeFaceIntersection& startInter)
	{
		m_CurrLoop = &loop;

		auto currStart = startInter;
		auto currTravelFace = &currStart.GetEdge().GetFace1();

		do
		{
			m_Targets.clear();
			FindTargetIntersections(*currTravelFace, intersByEdge, m_Targets);

			static EdgeFaceIntersection nextStart;
			if (!LinkToNext(currStart, *currTravelFace, m_Targets, nextStart))
				return false;

			currStart = nextStart;
			currTravelFace = &currStart.GetEdge().GetOther(*currTravelFace);
			intersByEdge.RemoveIntersection(currStart);

		} while (currStart != startInter);

		return true;
	}

private:
	IntersectionLoop * m_CurrLoop = nullptr;
	std::vector<EdgeFaceIntersection> m_Targets;
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

		m_Intersections.RegisterIntersection(EdgeFaceIntersection(face, piercingEdge, intPoint));
	}

	void FindIntersections(const std::vector<Face*>& faces, const std::vector<ShapeEdge*>& piercingEdges)
	{
		for (auto f : faces)
			for (auto e : piercingEdges)
				FindIntersection(*f, *e);
	}

	int IntersectionCount(const Face& f) const
	{
		auto c = 0;

		for (auto e : f.GetEdgeObjects())
			c += m_Intersections.GetIntersections(*e).size();

		return c;
	}

	bool ValidateIntersections(const std::vector<Face*>& facesWithPiercingEdges) const
	{
		for (auto f : facesWithPiercingEdges)
		{
			if ((IntersectionCount(*f) % 2) != 0)
				return false;
		}
		return true;
	}

	bool LinkIntersections(std::vector<IntersectionLoop*>& loops)
	{
		m_IntersectionLoops->Reset();
		EdgeFaceIntersection loopStart;

		while (m_Intersections.GetAnyIntersection(loopStart))
		{
			auto& loop = m_IntersectionLoops->Recycle();
			loop.Clear();

			if (!m_Linker.FindLoop(loop, m_Intersections, loopStart))
				return false;

			loops.emplace_back(&loop);
		}
		return true;
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

	void AssignHashes(const Shape& shapeEdges, const Shape& shapeFaces)
	{
		Face::ResetNextHashCounter();
		ShapeEdge::ResetNextHashCounter();
		ShapePoint::ResetNextHashCounter();

		AssignHashes(shapeEdges.GetEdgeObjects());
		AssignHashes(shapeEdges.GetPointObjects());

		AssignHashes(shapeFaces.GetEdgeObjects());
		AssignHashes(shapeFaces.GetFaces());
	}

	void UnAssignHashes(const Shape& shapeEdges, const Shape& shapeFaces)
	{
		Face::ResetNextHashCounter();
		ShapeEdge::ResetNextHashCounter();
		ShapePoint::ResetNextHashCounter();

		UnAssignHashes(shapeEdges.GetEdgeObjects());
		UnAssignHashes(shapeEdges.GetPointObjects());

		UnAssignHashes(shapeFaces.GetEdgeObjects());
		UnAssignHashes(shapeFaces.GetFaces());
	}

public:
	CleanIntersectionFinder()
	{
		m_IntersectionLoops = std::unique_ptr<PoolOfRecyclables<IntersectionLoop>>(
			new PoolOfRecyclables<IntersectionLoop>(3));
	}

	// Both shapes must have the same local space.
	// Loops are recycled so must be used before the next call.
	bool FindCleanIntersections(const Shape& shapeEdges, const Shape& shapeFaces, std::vector<IntersectionLoop*>& loops)
	{
		AssignHashes(shapeEdges, shapeFaces);

		DeterminePointPlaneRelationships(shapeEdges.GetPointObjects(), shapeFaces.GetFaces());
		DetermineEdgeEdgeRelationships(shapeFaces.GetEdgeObjects(), shapeEdges.GetEdgeObjects());
		FindIntersections(shapeFaces.GetFaces(), shapeEdges.GetEdgeObjects());

		auto valid = false;

		if (ValidateIntersections(shapeEdges.GetFaces()) && LinkIntersections(loops))
		{
			m_Intersections.ClearFast();
			valid = true;
		}
		else
		{
			m_Intersections.Clear();
		}

		UnAssignHashes(shapeEdges, shapeFaces);
		return valid;
	}

private:
	IntersectionsByPiercingEdge m_Intersections;
	IntersectionLinker m_Linker;
	std::unique_ptr<PoolOfRecyclables<IntersectionLoop>> m_IntersectionLoops;
	MapToPointPlaneRelationship m_PointPlaneRelationshipMap;
	TwoDArray<Constants::MaxNumEdges, Constants::MaxNumEdges, EdgeEdgeRelationship> m_EdgeEdgeRelationships;
};
