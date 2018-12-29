#pragma once
#include "Face.h"
#include "FacesCutPath.h"
#include "ShapePoint.h"
#include "SplitShapeEdge.h"
#include "ShapeEdge.h"
#include "MapToShapePointOnReversedFace.h"
#include "MapToNewEdges.h"

class FacesCutPathIterator
{
public:
	template<FaceRelationshipWithOtherShape inOrOut>
	void IteratePath(const FacesCutPath& cp, const ShapePoint& start)
	{
		CalculateTravelParams(cp, start);

		auto curr = m_StartIndex;
		while (curr != m_EndIndex)
		{
			auto nextIndex = cp.GetNextIndex(curr, m_TravelDir);

			auto& cpe = cp.GetElement(curr);

			auto& cpPoint = FacesCutPath::GetNewPointFromCpPoint<inOrOut>(cpe.GetPoint(), *m_MapToReversed);
			auto& nextCpPoint = FacesCutPath::GetNewPointFromCpPoint<inOrOut>(cp.GetElement(nextIndex).GetPoint(), *m_MapToReversed);

			m_NewFace->AddPoint(cpPoint, m_MapToEdges->GetNewEdge(cpPoint, nextCpPoint));
			curr = nextIndex;
		}

		auto& endCpe = cp.GetElement(curr);

		auto& piercingEdge = endCpe.GetPiercingEdge();
		auto& splitEdge = piercingEdge.GetSplitEdge();

		auto& nextPoint = splitEdge.GetNext(endCpe, piercingEdge.GetEnd(*m_OriginalFace));
		auto& endCpPoint = FacesCutPath::GetNewPointFromCpPoint<inOrOut>(endCpe.GetPoint(), *m_MapToReversed);

		m_NewFace->AddPoint(endCpPoint, m_MapToEdges->GetNewEdge(endCpPoint, nextPoint));

		m_NextPoint = &nextPoint;
		m_EndPiercingEdge = &piercingEdge;
	}

	ShapePoint& GetNextPoint()
	{
		return *m_NextPoint;
	}

	ShapeEdge& GetEndEdge()
	{
		return *m_EndPiercingEdge;
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
	void CalculateTravelParams(const FacesCutPath& cp, const ShapePoint& start)
	{
		auto& firstPoint = cp.GetElement(cp.GetFirstIndex()).GetPoint();

		if (&firstPoint == &start)
		{
			m_StartIndex = cp.GetFirstIndex();
			m_EndIndex = cp.GetFinalIndex();
			m_TravelDir = 1;
		}
		else
		{
			m_EndIndex = cp.GetFirstIndex();
			m_StartIndex = cp.GetFinalIndex();
			m_TravelDir = -1;
		}
	}

	ConcaveFace * m_NewFace = nullptr;
	const Face* m_OriginalFace = nullptr;

	int m_StartIndex;
	int m_EndIndex;
	int m_TravelDir;

	ShapePoint * m_NextPoint;
	ShapeEdge * m_EndPiercingEdge;

	const MapToShapePointOnReversedFace* m_MapToReversed = nullptr;
	MapToNewEdges* m_MapToEdges = nullptr;
};
