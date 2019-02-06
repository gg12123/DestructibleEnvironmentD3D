#pragma once
#include <vector>
#include "Face.h"
#include "ShapeEdge.h"
#include "IterationAboutShape.h"
#include "ShapeElementPool.h"
#include "CollectionU.h"
#include "NewShapeGeometryCreator.h"

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

		auto& e0 = m_NewEdges->GetNewEdge(pR, beforePr);
		IterationAboutShape::FindEdgesAndFacesAboutPoint(pR, e0, m_EdgesAboutPoint, m_FacesAboutPoint);

		assert(m_EdgesAboutPoint.size() == 3u);

		FindCoPlanarFaces();
		FindOtherFace();

		auto& newEdge = EdgePool::Take(beforePr, afterPr);
		m_NewEdges->AddNewEdge(newEdge);

		m_Fa0->MergeWith(*m_Fa1, *m_Ea01);
		m_Fa0->RemovePoint(pR, newEdge);
		m_Fb->RemovePoint(pR, newEdge);

		// Clear this face to indicate to the owner shape that it is
		// not required. Dont return it to the pool yet becuase it is
		// still referenced by its owner shape. Let the owner shape do it later.
		m_Fa1->Clear();

		for (auto e : m_EdgesAboutPoint)
			EdgePool::Return(*e);

		PointPool::Return(pR);
	}

	template<CollectionU::IterationDir itDir>
	ShapeEdge& FindNextEdgeNotRemovedFrom(int indexOfPr, const std::vector<EdgeFaceIntersection>& inters)
	{
		auto start = CollectionU::MoveIndex<std::vector<EdgeFaceIntersection>, itDir>(inters, indexOfPr);

		for (auto i = start;
			i != indexOfPr;
			i = CollectionU::MoveIndex<std::vector<EdgeFaceIntersection>, itDir>(inters, i))
		{
			auto& e = inters[i].GetEdge();
			if (!CollectionU::Contains(m_EdgesRemovedFrom, &e))
				return e;
		}

		assert(false);
		return inters[0].GetEdge();
	}

public:
	void Init(MapToNewEdges& edges, const MapToNewPoints& points)
	{
		m_NewEdges = &edges;
		m_NewPoints = &points;
	}

	void RemovePoints(const IntersectionLoop& loop)
	{
		m_EdgesRemovedFrom.clear();

		auto& inters = loop.GetIntersections();

		for (auto i = 0u; i < inters.size(); i++)
		{
			auto& e = inters[i].GetEdge();

			if (e.BridgesCoPlanarFaces())
			{
				auto& before = FindNextEdgeNotRemovedFrom<CollectionU::IterationDir::PrevDir>(i, inters);
				auto& after = FindNextEdgeNotRemovedFrom<CollectionU::IterationDir::NextDir>(i, inters);

				RemovePoint(m_NewPoints->GetPointAbove(e), m_NewPoints->GetPointAbove(before), m_NewPoints->GetPointAbove(after));
				RemovePoint(m_NewPoints->GetPointBelow(e), m_NewPoints->GetPointBelow(before), m_NewPoints->GetPointBelow(after));

				m_EdgesRemovedFrom.emplace_back(&e);
			}
		}
	}

private:
	std::vector<ShapeEdge*> m_EdgesAboutPoint;
	std::vector<Face*> m_FacesAboutPoint;

	std::vector<ShapeEdge*> m_EdgesRemovedFrom;

	Face* m_Fa0;
	Face* m_Fa1;
	Face* m_Fb;

	ShapeEdge* m_Ea01;

	MapToNewEdges* m_NewEdges;
	const MapToNewPoints* m_NewPoints;
};
