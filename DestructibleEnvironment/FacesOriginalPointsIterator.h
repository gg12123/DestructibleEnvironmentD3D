#pragma once
#include "Face.h"
#include "FacesCutPath.h"
#include "ShapePoint.h"
#include "SplitShapeEdge.h"
#include "ShapeEdge.h"

class FacesOriginalPointsIterator
{
public:
	ShapePoint & Iterate(int startIndex)
	{

	}

private:
	Face * m_OriginalFace;
	Face* m_NewFace;
};