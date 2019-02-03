#pragma once
#include <vector>
#include <memory>
#include "Vector3.h"
#include "FacesCutPaths.h"
#include "FacesPointsIterator.h"
#include "ShapeEdgesCreator.h"
#include "ShapeElementPool.h"
#include "NewShapeGeometryCreator.h"

class FaceSplitter
{
private:
	void FindPointsAboveAndBelow(const Face& f)
	{
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
				m_PointsAbove.emplace_back(&p);
			}
			else
			{
				m_PointsBelow.emplace_back(&p);
			}

			if (pNextRel != pRel)
			{
				auto& se = *edges[i];
				m_PointsAbove.emplace_back(&m_NewPoints->GetPointAbove(se));
				m_PointsBelow.emplace_back(&m_NewPoints->GetPointBelow(se));
			}
		}
	}

public:
	void Init(const MapToNewPoints& mapPoints, const MapToNewEdges& mapEdges, const MapToPointPlaneRelationship& pointPlaneMap)
	{

	}

	void Split(const IntersectionLoop& loop, std::vector<Face*>& facesAbove, std::vector<Face*>& facesBelow)
	{

	}

private:
	const MapToNewPoints* m_NewPoints;
	const MapToNewEdges* m_NewEdges;
	const MapToPointPlaneRelationship* m_PointPlaneMap;

	std::vector<ShapePoint*> m_PointsAbove;
	std::vector<ShapePoint*> m_PointsBelow;
};