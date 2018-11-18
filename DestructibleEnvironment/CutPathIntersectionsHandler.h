#pragma once
#include "EdgeFaceIntersection.h"
#include "IntersectionEquivalenceChecker.h"
#include "CutPathElement.h"
#include "Shape.h"
#include "Face.h"

class CutPathIntersectionsHandler
{
private:
	void UseIntersection(const EdgeFaceIntersection& interToUse)
	{
		m_IntersForCPStartOther.clear();

		for (auto it = m_IntersForCPStart.begin(); it != m_IntersForCPStart.end(); it++)
		{
			auto& inter = *it;

			if ((inter != interToUse) && !m_EquivalenceChecker.AreEquivalent(inter, interToUse))
				m_IntersForCPStartOther.emplace_back(inter);
		}

		m_IntersForCPStart.swap(m_IntersForCPStartOther);
	}

	EdgeFaceIntersection ToInteresection(const CutPathElement& element)
	{
		return EdgeFaceIntersection(element.GetPiercedFace(), element.GetPiercingEdge());
	}

public:
	// all inersections found between cut shape edges and original shape faces
	void Init(std::vector<EdgeFaceIntersection>& inters, const Shape& origShape)
	{
		m_EquivalenceChecker.SetIntersections(inters);

		m_IntersForCPStart.clear();
		m_IntersForCPStart.swap(inters);

		m_OriginalShape = &origShape;
	}

	bool GetNextStartIntersection(EdgeFaceIntersection& nextStart)
	{
		if (m_IntersForCPStart.size() > 0)
		{
			nextStart = m_IntersForCPStart[0];
			UseIntersection(nextStart);
			return true;
		}
		return false;
	}

	bool NewCutPathElementReached(const CutPathElement& element)
	{
		auto inter = ToInteresection(element);

		if (&inter.GetFace().GetShape() == m_OriginalShape)
		{
			if ((inter != m_CurrStartIntersection) && m_EquivalenceChecker.AreEquivalent(inter, m_CurrStartIntersection))
				return false; // TODO - this can be corrected for.

			UseIntersection(inter);
			
		}
		return true;
	}

private:
	std::vector<EdgeFaceIntersection> m_IntersForCPStart;
	std::vector<EdgeFaceIntersection> m_IntersForCPStartOther;

	EdgeFaceIntersection m_CurrStartIntersection;
	IntersectionEquivalenceChecker m_EquivalenceChecker;

	const Shape* m_OriginalShape;
};
