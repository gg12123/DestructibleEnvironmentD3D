#pragma once
#include "FacesOriginalPointsIterator.h"
#include "FacesCutPathIterator.h"

class FacesPointsIterator
{
public:
	void IterateInside(const FacesCutPath& startCp)
	{
		auto& sp = FindStartPointInside(startCp);
		auto iteratingCp = true;
		auto& cp = startCp;

		ShapePoint* p = nullptr;

		while (p != &sp)
		{

		}
	}

private:
	float N0DotD0(const FacesCutPath& startCp) const
	{
		auto& firstElement = startCp.GetFirstElement();

		auto& d0 = firstElement.GetPiercingEdge().GetDirection(*m_OriginalFace);
		auto& n0 = firstElement.GetPiercedFace().GetNormal();

		return Vector3::Dot(n0, d0);
	}

	ShapePoint & FindStartPointInside(const FacesCutPath& startCp) const
	{
		return N0DotD0(startCp) > 0.0f ? startCp.GetFirstElement().GetPoint() : startCp.GetFinalElement().GetPoint();
	}

	ShapePoint & FindStartPointOutside(const FacesCutPath& startCp) const
	{
		return N0DotD0(startCp) > 0.0f ? startCp.GetFinalElement().GetPoint() : startCp.GetFirstElement().GetPoint();
	}

	Face* m_OriginalFace;
};
