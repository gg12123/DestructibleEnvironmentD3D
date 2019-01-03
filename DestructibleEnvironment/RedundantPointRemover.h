#pragma once
#include <vector>
#include "Face.h"
#include "ShapeEdge.h"
#include "IterationAboutShape.h"
#include "CutPathElement.h"
#include "ShapeEdgesCreator.h"
#include "ShapeElementPool.h"
#include "CollectionU.h"
#include "MapToShapePointOnReversedFace.h"

class RedundantPointRemover
{
private:
	void FindCoPlanarFaces()
	{
		for (auto e : m_EdgesAboutPoint)
		{
			if (e->BridgesCoPlanarFaces())
			{
				m_Fa0 = &e->GetFace1();
				m_Fa1 = &e->GetFace2();
				m_Ea01 = e;
				return;
			}
		}
		assert(false);
	}

	void FindOtherFace()
	{
		for (auto f : m_FacesAboutPoint)
		{
			if (f != m_Fa1 && f != m_Fa0)
			{
				m_Fb = f;
				return;
			}
		}
		assert(false);
	}

	void RemovePoint(ShapePoint& pR, ShapePoint& beforePr, ShapePoint& afterPr)
	{
		m_FacesAboutPoint.clear();
		m_EdgesAboutPoint.clear();

		auto& e0 = m_EdgesCreator->GetMapToNewEdges().GetNewEdge(pR, beforePr);

		IterationAboutShape::FindEdgesAndFacesAboutPoint(pR, e0, m_EdgesAboutPoint, m_FacesAboutPoint);

		assert(m_EdgesAboutPoint.size() == 3u);
		assert(m_FacesAboutPoint.size() == 3u);

		FindCoPlanarFaces();
		FindOtherFace();

		m_EdgesCreator->CreateEdge(beforePr, afterPr);
		auto& newEdge = m_EdgesCreator->GetMapToNewEdges().GetNewEdge(beforePr, afterPr);

		m_Fa0->MergeWith(*m_Fa1, *m_Ea01);
		m_Fa0->RemovePoint(pR, newEdge);
		m_Fb->RemovePoint(pR, newEdge);

		// Clear this face to indicate to the owner shape that it is
		// not required.
		m_Fa1->Clear();

		// Everything is returned to the pool here, even though some things
		// may be accesed again by this algorithm. I think that is ok for now though.

		FacePool::Return(*m_Fa1);
		PointPool::Return(pR);

		for (auto e : m_EdgesAboutPoint)
			EdgePool::Return(*e);
	}

	template<CollectionU::IterationDir itDir>
	ShapePoint& FindNextRequiredPoint(int indexOfPr, const std::vector<CutPathElement>& cp)
	{
		auto start = CollectionU::MoveIndex<std::vector<CutPathElement>, itDir>(cp, indexOfPr);

		for (auto i = start;
			i != indexOfPr;
			i = CollectionU::MoveIndex<std::vector<CutPathElement>, itDir>(cp, i))
		{
			if (!cp[i].GetPiercingEdge().BridgesCoPlanarFaces())
				return cp[i].GetPoint();
		}

		assert(false);
		return cp[0].GetPoint();
	}

public:
	void Init(ShapeEdgesCreator& edges, const MapToShapePointOnReversedFace& toPointOnRev)
	{
		m_EdgesCreator = &edges;
		m_ToPointOnRev = &toPointOnRev;
	}

	void RemovePoints(const std::vector<CutPathElement>& cp)
	{
		for (auto i = 0u; i < cp.size(); i++)
		{
			if (cp[i].GetPiercingEdge().BridgesCoPlanarFaces())
			{
				auto& pR = cp[i].GetPoint();
				auto& before = FindNextRequiredPoint<CollectionU::IterationDir::PrevDir>(i, cp);
				auto& after = FindNextRequiredPoint<CollectionU::IterationDir::NextDir>(i, cp);

				RemovePoint(pR, before, after);

				RemovePoint(m_ToPointOnRev->GetPointOnReversedFace(pR),
					m_ToPointOnRev->GetPointOnReversedFace(before),
					m_ToPointOnRev->GetPointOnReversedFace(after));
			}
		}
	}

private:
	std::vector<ShapeEdge*> m_EdgesAboutPoint;
	std::vector<Face*> m_FacesAboutPoint;

	Face* m_Fa0;
	Face* m_Fa1;
	ShapeEdge* m_Ea01;
	Face* m_Fb;

	ShapeEdgesCreator* m_EdgesCreator;
	const MapToShapePointOnReversedFace* m_ToPointOnRev;
};
