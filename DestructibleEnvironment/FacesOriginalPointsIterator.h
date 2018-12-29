#pragma once
#include "Face.h"
#include "FacesCutPath.h"
#include "ShapePoint.h"
#include "SplitShapeEdge.h"
#include "ShapeEdge.h"
#include "MapToShapePointOnReversedFace.h"
#include "MapToNewEdges.h"
#include "ConcaveFace.h"

class FacesOriginalPointsIterator
{
public:
	template<FaceRelationshipWithOtherShape inOrOut>
	ShapePoint& Iterate(int startIndex)
	{
		auto& edges = m_OriginalFace->GetEdgeObjects();
		auto& points = m_OriginalFace->GetPointObjects();

		auto i = startIndex;
		while (!edges[i]->IsSplit())
		{
			auto& edge = *edges[i];

			edge.DeRegisterFace(*m_OriginalFace);

			m_NewFace->AddPoint(*points[i], edge);
			i = m_OriginalFace->NextPointIndex(i);
		}

		auto& se = edges[i]->GetSplitEdge();
		auto& np = FacesCutPath::GetNewPointFromCpPoint<inOrOut>(se.GetNext(*points[i]), *m_MapToReversed);

		auto& endOp = *points[i];
		m_NewFace->AddPoint(endOp, m_MapToEdges->GetNewEdge(endOp, np));

		return FacesCutPath::GetNewPointFromCpPoint<FaceRelationshipWithOtherShape::InIntersection>(se.GetNext(*points[i]), *m_MapToReversed);
	}

	void InitMaps(const MapToShapePointOnReversedFace& map, MapToNewEdges& edgeMap)
	{
		m_MapToEdges = &edgeMap;
		m_MapToReversed = &map;
	}

	void InitFaces(const Face& orig, ConcaveFace& newFace)
	{
		m_OriginalFace = &orig;
		m_NewFace = &newFace;
	}

private:
	const Face * m_OriginalFace = nullptr;
	ConcaveFace* m_NewFace = nullptr;
	const MapToShapePointOnReversedFace* m_MapToReversed = nullptr;
	MapToNewEdges* m_MapToEdges = nullptr;
};