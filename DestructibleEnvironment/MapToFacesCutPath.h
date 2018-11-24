#pragma once
#include "Face.h"
#include "ShapePoint.h"
#include "FacesCutPath.h"
#include "TwoDArray.h"
#include "Constants.h"

class MapToFacesCutPath
{
public:
	FacesCutPath & GetPath(const Face& faceExitedOrEntered, const ShapePoint& pointOnCp) const
	{
		return *m_Map.Get(pointOnCp.GetHash(), faceExitedOrEntered.GetHash());
	}

	void AddPath(const Face& faceExitedOrEntered, const ShapePoint& pointOnCp, FacesCutPath& fcp)
	{
		m_Map.Get(pointOnCp.GetHash(), faceExitedOrEntered.GetHash()) = &fcp;
	}

private:
	TwoDArray<Constants::MaxNumPoints, Constants::MaxNumFaces, FacesCutPath*> m_Map;
};
