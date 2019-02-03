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

	void FindEdgesToRemoveAndExtend(const Face& faceToLoseEdge)
	{
		m_EbToExtend = nullptr;
		m_EbToRemove = nullptr;

		for (auto e : m_EdgesAboutPoint)
		{
			if (e != m_Ea01)
			{
				if (e->IsAttachedTo(faceToLoseEdge))
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
	}

	void EnsureEdgeIsEdgeMap(ShapeEdge& e) const
	{
		e.GetP0().TryAssignHash();
		e.GetP1().TryAssignHash();

		m_EdgesCreator->GetMapToNewEdges().AddNewEdge(e.GetP0(), e.GetP1(), e);
	}

	bool PointIsRedundant(const ShapePoint& p, ShapeEdge& attachedToP) const
	{
		static std::vector<ShapeEdge*> edges;
		static std::vector<Face*> faces;

		edges.clear();
		faces.clear();

		IterationAboutShape::FindEdgesAndFacesAboutPoint(p, attachedToP, edges, faces);

		if (edges.size() == 3u)
		{
			for (auto e : edges)
			{
				if (e->BridgesCoPlanarFaces())
					return true;
			}
		}
		return false;
	}

	bool FaceCanLoseEdge(const Face& f, const ShapePoint& pR)
	{
		if (f.GetPointObjects().size() > 3u)
		{
			FindEdgesToRemoveAndExtend(f);
			return !PointIsRedundant(m_EbToRemove->GetOther(pR), *m_EbToRemove);
		}
		return false;
	}

	bool CanDoShiftEdgeMethod(const ShapePoint& pR, Face** faceToLoseEdge)
	{
		if (FaceCanLoseEdge(*m_Fa0, pR))
		{
			*faceToLoseEdge = m_Fa0;
			return true;
		}

		if (FaceCanLoseEdge(*m_Fa1, pR))
		{
			*faceToLoseEdge = m_Fa1;
			return true;
		}
		return false;
	}

	int NumSharedEdges(const Face& f0, const Face& f1) const
	{
		auto num = 0;
		for (auto e : f0.GetEdgeObjects())
		{
			if (e->IsAttachedTo(f1))
				num++;
		}
		return num;
	}

	bool CanDoMergeFaceMethod() const
	{
		return NumSharedEdges(*m_Fa0, *m_Fa1) == 1;
	}

	void RemovePoint(ShapePoint& pR, ShapePoint& beforePr, ShapePoint& afterPr)
	{
		m_FacesAboutPoint.clear();
		m_EdgesAboutPoint.clear();

		auto& e0 = m_EdgesCreator->GetMapToNewEdges().GetNewEdge(pR, beforePr);
		IterationAboutShape::FindEdgesAndFacesAboutPoint(pR, e0, m_EdgesAboutPoint, m_FacesAboutPoint);

		assert(m_EdgesAboutPoint.size() == 3u || m_EdgesAboutPoint.size() == 2u);

		if (m_EdgesAboutPoint.size() == 2u)
		{
			m_EdgesCreator->CreateEdge(beforePr, afterPr);
			auto& newEdge = m_EdgesCreator->GetMapToNewEdges().GetNewEdge(beforePr, afterPr);

			m_FacesAboutPoint[0]->RemovePoint(pR, newEdge);
			m_FacesAboutPoint[1]->RemovePoint(pR, newEdge);

			for (auto e : m_EdgesAboutPoint)
				EdgePool::Return(*e);
		}
		else
		{
			FindCoPlanarFaces();
			FindOtherFace();

			Face* faceToLoseEdge = nullptr;

			if (CanDoShiftEdgeMethod(pR, &faceToLoseEdge))
			{
				auto& faceToHaveExtendedEdge = faceToLoseEdge == m_Fa0 ? *m_Fa1 : *m_Fa0;
				auto& pointToGainEdge = m_EbToRemove->GetOther(pR);

				m_Ea01->ReplacePoint(pR, pointToGainEdge);
				m_EbToExtend->ReplacePoint(pR, pointToGainEdge);

				faceToLoseEdge->RemovePointAndEdge(pR, *m_EbToRemove);
				faceToHaveExtendedEdge.ReplacePointObject(pR, pointToGainEdge);
				m_Fb->RemovePointAndEdge(pR, *m_EbToRemove);

				// Not sure if I also need to clear these edges from the map before re-adding
				// them in their new state.
				EnsureEdgeIsEdgeMap(*m_Ea01);
				EnsureEdgeIsEdgeMap(*m_EbToExtend);

				EdgePool::Return(*m_EbToRemove);
			}
			else if (CanDoMergeFaceMethod())
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
			else
			{
				assert(false);
			}
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
