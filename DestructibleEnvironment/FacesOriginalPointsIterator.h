#pragma once
#include "Face.h"
#include "FacesCutPath.h"
#include "ShapePoint.h"
#include "SplitShapeEdge.h"
#include "ShapeEdge.h"
#include "MapToShapePointOnReversedFace.h"
#include "MapToNewEdges.h"

class FacesOriginalPointsIterator
{
public:
	template<FaceRelationshipWithOtherShape inOrOut>
	ShapePoint& Iterate(int startIndex)
	{
		auto& edges = m_OriginalFace->GetEdgeObjects();
		auto& points = m_OriginalFace->GetPointObjects();
		auto& dirs = m_OriginalFace->GetEdgeDirections();

		auto i = startIndex;
		while (!edges[i]->IsSplit())
		{
			auto& edge = *edges[i];

			edge.DeRegisterFace(*m_OriginalFace);

			m_NewFace->AddPoint(*points[i], dirs[i], edge);
			i = m_OriginalFace->NextPointIndex(i);
		}

		auto& se = edges[i]->GetSplitEdge();
		auto& np = FacesCutPath::GetNewPointFromCpPoint<inOrOut>(se.GetNext(*points[i]), *m_MapToReversed);

		auto endOp = *points[i];
		m_NewFace->AddPoint(endOp, dirs[i], m_MapToEdges->GetNewEdge(endOp, np));

		return np;
	}

	void InitMaps(const MapToShapePointOnReversedFace& map, const MapToNewEdges& edgeMap)
	{
		m_MapToEdges = &edgeMap;
		m_MapToReversed = &map;
	}

	void InitFaces(const Face& orig, Face& newFace)
	{
		m_OriginalFace = &orig;
		m_NewFace = &newFace;
	}

private:
	const Face * m_OriginalFace = nullptr;
	Face* m_NewFace = nullptr;
	const MapToShapePointOnReversedFace* m_MapToReversed = nullptr;
	const MapToNewEdges* m_MapToEdges = nullptr;
};