#pragma once
#include "Face.h"
#include "FacesCutPath.h"
#include "ShapePoint.h"
#include "SplitShapeEdge.h"
#include "ShapeEdge.h"

class FacesCutPathIterator
{
public:
	// maybe template this for the mapped version
	void IteratePath(const FacesCutPath& cp, const ShapePoint& start)
	{
		CalculateTravelParams(cp, start);

		auto curr = m_StartIndex;
		while (curr != m_EndIndex)
		{
			auto& cpe = cp.GetElement(curr);

			m_NewFace->AddPoint(cpe.GetPoint(), cp.GetDirToNext(curr, m_TravelDir), ); // get edge between cpe point and next cp point
			curr = cp.GetNextIndex(curr, m_TravelDir);
		}

		auto& endCpe = cp.GetElement(curr);

		auto& piercingEdge = endCpe.GetPiercingEdge();
		auto& splitEdge = piercingEdge.GetSplitEdge();

		auto& nextPoint = splitEdge.GetNext(endCpe.GetPoint(), piercingEdge.GetEnd(*m_OriginalFace));
		auto dir = piercingEdge.GetDirection(*m_OriginalFace);

		m_NewFace->AddPoint(endCpe.GetPoint(), dir, ); // get edge between endCpe point and nextPoint

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

	Face * m_NewFace;
	Face* m_OriginalFace;

	int m_StartIndex;
	int m_EndIndex;
	int m_TravelDir;

	ShapePoint * m_NextPoint;
	ShapeEdge * m_EndPiercingEdge;
};
