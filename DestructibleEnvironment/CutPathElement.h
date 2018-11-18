#pragma once
#include "Vector3.h"

class Face;
class ShapePoint;
class ShapeEdge;

class CutPathElement
{
public:
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