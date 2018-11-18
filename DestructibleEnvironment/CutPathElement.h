#pragma once
#include "Vector3.h"
#include "ShapePoint.h"
#include "Face.h"
#include "ShapeEdge.h"

class CutPathElement
{
public:
	CutPathElement(const FaceEdgeCaseResult& castRes, Face& faceExited, Face& facePierced, const Vector3& dirFromPrev)
	{
		m_PiercingEdge = castRes.Edge;
		m_FaceExited = &faceExited;
		m_PiercedFace = &facePierced;
		m_FaceEntered = &m_PiercingEdge->GetOther(*m_FaceExited);
		m_DirFromPrev = dirFromPrev;
	}

	int GetIndexInSplitEdge() const
	{
		return m_IndexInSplitEdge;
	}

	void SetIndexInSplitEdge(int index)
	{
		m_IndexInSplitEdge = index;
	}

	Face & GetPiercedFace() const
	{
		return *m_PiercedFace;
	}

	Face & GetFaceEntered() const
	{
		return *m_FaceEntered;
	}

	Face & GetFaceExited() const
	{
		return *m_FaceExited;
	}

	ShapeEdge& GetPiercingEdge() const
	{
		return *m_PiercingEdge;
	}

	ShapePoint& GetPoint() const
	{
		return *m_Point;
	}

	const Vector3& GetDirFromPrev() const
	{
		return m_DirFromPrev;
	}

	Vector3 GetIntPoint() const
	{
		return m_Point->GetPoint();
	}

private:
	Face* m_PiercedFace;
	Face* m_FaceEntered;
	Face* m_FaceExited;

	ShapeEdge* m_PiercingEdge;
	ShapePoint* m_Point;

	Vector3 m_DirFromPrev;

	int m_IndexInSplitEdge = -1;
};