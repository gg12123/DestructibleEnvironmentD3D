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

	Face& FindEdgesToRemoveAndExtend()
	{
		auto faceToLoseEdge = m_Fa0->GetPointObjects().size() > m_Fa1->GetPointObjects().size() ? m_Fa0 : m_Fa1;

		m_EbToExtend = nullptr;
		m_EbToRemove = nullptr;

		for (auto e : m_EdgesAboutPoint)
		{
			if (e != m_Ea01)
			{
				if (e->IsAttachedTo(*faceToLoseEdge))
				{
					m_EbToRemove = e;
				}
				else
				{
					m_EbToExtend = e;
				}
			}
		}

		assert(m_EbToExtend && m_EbToRemove);
		return *faceToLoseEdge;
	}

	void EnsureEdgeIsEdgeMap(ShapeEdge& e) const
	{
		e.GetP0().TryAssignHash();
		e.GetP1().TryAssignHash();

		m_EdgesCreator->GetMapToNewEdges().AddNewEdge(e.GetP0(), e.GetP1(), e);
	}

	void RemovePoint(ShapePoint& pR, ShapePoint& beforePr, ShapePoint& afterPr)
	{
		m_FacesAboutPoint.clear();
		m_EdgesAboutPoint.clear();

		auto& e0 = m_EdgesCreator->GetMapToNewEdges().GetNewEdge(pR, beforePr);
		IterationAboutShape::FindEdgesAndFacesAboutPoint(pR, e0, m_EdgesAboutPoint, m_FacesAboutPoint);

		assert(m_EdgesAboutPoint.size() == 3u);

		FindCoPlanarFaces();
		FindOtherFace();

		if (m_Fa1->GetPointObjects().size() > 3u || m_Fa0->GetPointObjects().size() > 3u)
		{
			auto& faceToLoseEdge = FindEdgesToRemoveAndExtend();
			auto& faceToHaveExtendedEdge = &faceToLoseEdge == m_Fa0 ? *m_Fa1 : *m_Fa0;
			auto& pointToGainEdge = m_EbToRemove->GetOther(pR);

			m_Ea01->ReplacePoint(pR, pointToGainEdge);
			m_EbToExtend->ReplacePoint(pR, pointToGainEdge);

			faceToLoseEdge.RemovePointAndEdge(pR, *m_EbToRemove);
			faceToHaveExtendedEdge.ReplacePointObject(pR, pointToGainEdge);
			m_Fb->RemovePointAndEdge(pR, *m_EbToRemove);

			// Not sure if I also need to clear these edges from the map before re-adding
			// them in their new state.
			EnsureEdgeIsEdgeMap(*m_Ea01);
			EnsureEdgeIsEdgeMap(*m_EbToExtend);
			
			EdgePool::Return(*m_EbToRemove);
		}
		else
		{
			m_EdgesCreator->CreateEdge(beforePr, afterPr);
			auto& newEdge = m_EdgesCreator->GetMapToNewEdges().GetNewEdge(beforePr, afterPr);

			m_Fa0->MergeWith(*m_Fa1, *m_Ea01);
			m_Fa0->RemovePoint(pR, newEdge);
			m_Fb->RemovePoint(pR, newEdge);

			// Clear this face to indicate to the owner shape that it is
			// not required. Dont return it to the pool yet becuase it is
			// still referenced by its owner shape. Let the owner shape do it later.
			m_Fa1->Clear();

			for (auto e : m_EdgesAboutPoint)
				EdgePool::Return(*e);
		}

		PointPool::Return(pR);
	}

	template<CollectionU::IterationDir itDir>
	ShapePoint& FindNextNonRemovedPoint(int indexOfPr, const std::vector<CutPathElement>& cp)
	{
		auto start = CollectionU::MoveIndex<std::vector<CutPathElement>, itDir>(cp, indexOfPr);

		for (auto i = start;
			i != indexOfPr;
			i = CollectionU::MoveIndex<std::vector<CutPathElement>, itDir>(cp, i))
		{
			auto& p = cp[i].GetPoint();
			if (!CollectionU::Contains(m_PointsRemoved, &p))
				return p;
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
		m_PointsRemoved.clear();

		for (auto i = 0u; i < cp.size(); i++)
		{
			if (cp[i].GetPiercingEdge().BridgesCoPlanarFaces())
			{
				auto& pR = cp[i].GetPoint();
				auto& before = FindNextNonRemovedPoint<CollectionU::IterationDir::PrevDir>(i, cp);
				auto& after = FindNextNonRemovedPoint<CollectionU::IterationDir::NextDir>(i, cp);

				RemovePoint(pR, before, after);

				RemovePoint(m_ToPointOnRev->GetPointOnReversedFace(pR),
					m_ToPointOnRev->GetPointOnReversedFace(before),
					m_ToPointOnRev->GetPointOnReversedFace(after));

				m_PointsRemoved.emplace_back(&pR);
			}
		}
	}

private:
	std::vector<ShapeEdge*> m_EdgesAboutPoint;
	std::vector<Face*> m_FacesAboutPoint;

	std::vector<ShapePoint*> m_PointsRemoved;

	Face* m_Fa0;
	Face* m_Fa1;
	Face* m_Fb;

	ShapeEdge* m_EbToRemove;
	ShapeEdge* m_EbToExtend;
	ShapeEdge* m_Ea01;

	ShapeEdgesCreator* m_EdgesCreator;
	const MapToShapePointOnReversedFace* m_ToPointOnRev;
};
