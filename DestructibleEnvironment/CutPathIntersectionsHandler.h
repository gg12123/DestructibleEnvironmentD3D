#pragma once
#include "EdgeFaceIntersection.h"
#include "IntersectionEquivalenceChecker.h"
#include "CutPathElement.h"
#include "Shape.h"
#include "Face.h"

class CutPathIntersectionsHandler
{
private:
	void RemoveEquivalentIntersections(const EdgeFaceIntersection& interReached)
	{
		m_IntersForCPStartOther.clear();

		for (auto it = m_IntersForCPStart.begin(); it != m_IntersForCPStart.end(); it++)
		{
			auto& inter = *it;

			if ((inter != interReached) && !m_EquivalenceChecker.AreEquivalent(inter, interReached))
				m_IntersForCPStartOther.emplace_back(inter);
		}

		m_IntersForCPStart.swap(m_IntersForCPStartOther);
	}

	EdgeFaceIntersection ToInteresection(const CutPathElement& element)
	{
		return EdgeFaceIntersection(element.GetPiercedFace(), element.GetPiercingEdge(), element.GetIntPoint());
	}

	bool IntersectionAlreadyReached(const EdgeFaceIntersection& inter) const
	{
		for (auto it = m_ReachedIntersections.begin(); it != m_ReachedIntersections.end(); it++)
		{
			if (m_EquivalenceChecker.AreEquivalent(inter, *it))
				return true;
		}
		return false;
	}

public:
	// all inersections found between cut shape edges and original shape faces
	void Init(std::vector<EdgeFaceIntersection>& inters, const Shape& origShape)
	{
		m_IntersForCPStart.clear();
		m_IntersForCPStart.swap(inters);

		m_OriginalShape = &origShape;

		m_ReachedIntersections.clear();
	}

	bool GetNextStartIntersection(EdgeFaceIntersection& nextStart)
	{
		if (m_IntersForCPStart.size() > 0)
		{
			nextStart = m_IntersForCPStart[0];
			RemoveEquivalentIntersections(nextStart);
			m_ReachedIntersections.emplace_back(nextStart);
			return true;
		}
		return false;
	}

	bool NewCutPathElementReached(const CutPathElement& element)
	{
		auto inter = ToInteresection(element);

		if (inter == m_CurrStartIntersection)
			return true;

		if (&inter.GetFace().GetOwnerShape() == m_OriginalShape)
		{
			if ((inter != m_CurrStartIntersection) && m_EquivalenceChecker.AreEquivalent(inter, m_CurrStartIntersection))
				return false; // TODO - this can be corrected for.

			RemoveEquivalentIntersections(inter);
		}

		if (IntersectionAlreadyReached(inter))
			return false;

		m_ReachedIntersections.emplace_back(inter);
		return true;
	}

private:
	std::vector<EdgeFaceIntersection> m_IntersForCPStart;
	std::vector<EdgeFaceIntersection> m_IntersForCPStartOther;

	std::vector<EdgeFaceIntersection> m_ReachedIntersections;

	EdgeFaceIntersection m_CurrStartIntersection;
	IntersectionEquivalenceChecker m_EquivalenceChecker;

	const Shape* m_OriginalShape;
};
