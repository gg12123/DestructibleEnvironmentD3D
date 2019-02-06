#pragma once
#include <vector>
#include <memory>
#include "Vector3.h"
#include "ShapeElementPool.h"
#include "NewShapeGeometryCreator.h"

class FaceSplitter
{
private:
	struct PointWithIndex
	{
		ShapePoint* Point;
		int Index;

		PointWithIndex(ShapePoint& p, int i) : Point(&p), Index(i)
		{
		}
	};

	void FindPointsAboveAndBelow(const Face& f)
	{
		m_PointsAbove.clear();
		m_PointsBelow.clear();

		auto& points = f.GetPointObjects();
		auto& edges = f.GetEdgeObjects();

		for (auto i = 0u; i < points.size(); i++)
		{
			auto& p = *points[i];
			auto& pNext = *points[(i + 1u) % points.size()];

			auto pRel = m_PointPlaneMap->GetRelationship(p);
			auto pNextRel = m_PointPlaneMap->GetRelationship(pNext);

			if (pRel == PointPlaneRelationship::PointsAbove)
			{
				m_PointsAbove.emplace_back(PointWithIndex(p, i));
			}
			else
			{
				m_PointsBelow.emplace_back(PointWithIndex(p, i));
			}

			if (pNextRel != pRel)
			{
				auto& se = *edges[i];
				m_PointsAbove.emplace_back(PointWithIndex(m_NewPoints->GetPointAbove(se), -1));
				m_PointsBelow.emplace_back(PointWithIndex(m_NewPoints->GetPointBelow(se), -1));
			}
		}
	}

	Face& CreateNewFace(const Face& orignal, const std::vector<PointWithIndex>& points)
	{
		auto& edges = orignal.GetEdgeObjects();
		auto& newFace = FacePool::Take();

		for (auto i = 0u; i < points.size(); i++)
		{
			auto& p = points[i];
			auto& pNext = points[(i + 1u) % points.size()];

			auto& e = p.Index == -1 || pNext.Index == -1 ?
				m_NewEdges->GetNewEdge(*p.Point, *pNext.Point) :
				*edges[i];

			if (e.IsAttachedTo(orignal))
				e.DeRegisterFace(orignal);

			newFace.AddPoint(*p.Point, e);
		}

		newFace.SetNormal(orignal.GetNormal());
		return newFace;
	}

public:
	void Init(const MapToNewPoints& mapPoints, const MapToNewEdges& mapEdges, const MapToPointPlaneRelationship& pointPlaneMap)
	{
		m_NewPoints = &mapPoints;
		m_NewEdges = &mapEdges;
		m_PointPlaneMap = &pointPlaneMap;
	}

	void Split(const IntersectionLoop& loop, std::vector<Face*>& newFacesAbove, std::vector<Face*>& newFacesBelow)
	{
		for (auto i = 0; i < loop.GetCount(); i++)
		{
			auto& original = loop.GetFaceEntered(i);

			FindPointsAboveAndBelow(original);

			newFacesAbove.emplace_back(&CreateNewFace(original, m_PointsAbove));
			newFacesBelow.emplace_back(&CreateNewFace(original, m_PointsBelow));
		}
	}

private:
	const MapToNewPoints* m_NewPoints;
	const MapToNewEdges* m_NewEdges;
	const MapToPointPlaneRelationship* m_PointPlaneMap;

	std::vector<PointWithIndex> m_PointsAbove;
	std::vector<PointWithIndex> m_PointsBelow;
};