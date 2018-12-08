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
		m_Intersections[inter.GetEdge().GetHash()].emplace_back(inter);
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

class IntersectionLoop
{
public:
	void AddIntersection(const EdgeFaceIntersection& inter)
	{

	}

	void Clear()
	{

	}
};

class IntersectionLinker
{
private:
	bool TargetReached(const std::vector<EdgeFaceIntersection>& targets, const EdgeFaceIntersection& curr) const
	{
		return CollectionU::Contains(targets, curr);
	}

	EdgeFaceIntersection CastToNext(const EdgeFaceIntersection& curr, Face& travelFace, Face& currOtherFace) const
	{
		auto& ownerOfEdge = curr.GetEdge().IsAttachedTo(travelFace) ? travelFace : currOtherFace;

		auto castOrigin = curr.GetIntPoint();
		auto castDir = Vector3::Cross(travelFace.GetNormal(), currOtherFace.GetNormal()).InDirectionOf(-ownerOfEdge.GetEdgeNormal(curr.GetEdge()));

		auto mag = castDir.Magnitude();
		assert(mag > 0.0f);
		castDir /= mag;

		auto hitTravelFace = travelFace.CastToEdgeInside(castOrigin, castDir);
		auto hitOtherFace = currOtherFace.CastToEdgeInside(castOrigin, castDir);

		return hitTravelFace.Distance < hitOtherFace.Distance ?
			EdgeFaceIntersection(currOtherFace, *hitTravelFace.Edge, hitTravelFace.IntPoint) :
			EdgeFaceIntersection(travelFace, *hitOtherFace.Edge, hitOtherFace.IntPoint);
	}

	bool LinkToNext(const EdgeFaceIntersection& start, Face& travelFace, const std::vector<EdgeFaceIntersection>& targets, EdgeFaceIntersection& nextStart)
	{
		// TODO - this is fairly likly to fail. Need to use something like A*.

		auto curr = start;
		auto currOtherFace = &start.GetFace();
		auto targetReached = false;

		while (!targetReached)
		{
			curr = CastToNext(curr, travelFace, *currOtherFace);
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
			intersByEdge.RemoveIntersection(currStart);

		} while (currStart != startInter);
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

	}

	void DeterminePointPlaneRelationships(const std::vector<ShapePoint*>& points, const std::vector<Face*>& faces)
	{

	}

	void DetermineEdgeEdgeRelationships(const std::vector<ShapeEdge*>& piercedFacesEdges, const std::vector<ShapeEdge*>& piercingEdges)
	{

	}

	void FindIntersection(const Face& face, const ShapeEdge& piercingEdge)
	{
		auto& facesEdges = face.GetEdgeObjects();

		for (auto faceEdge : facesEdges)
		{
			if (!GetEdgeEdgeRelationship(*faceEdge, piercingEdge).ImpliesIntersectionFor(face))
				return;
		}

		// register the intersection
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

public:
	// Both shapes must have the same local space.
	// Loops are recycled so must be used before the next call.
	bool FindCleanIntersections(const Shape& shapeEdges, const Shape& shapeFaces, std::vector<IntersectionLoop*>& loops)
	{
		// assign hashes

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

		// un-assign hashes
		return valid;
	}

private:
	IntersectionsByPiercingEdge m_Intersections;
	IntersectionLinker m_Linker;
	std::unique_ptr<PoolOfRecyclables<IntersectionLoop>> m_IntersectionLoops;
};
