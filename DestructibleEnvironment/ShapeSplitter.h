#pragma once
#include <vector>
#include "IntersectionFinder.h"
#include "FaceSplitter.h"
#include "FaceFaceIntersection.h"

class Shape;
class Face;

class ShapeSplitter
{
public:
	void Split(const Vector3& splitPoint, const Vector3& splitNormal, Shape& orginalShape, Shape& newShape);

	void SetCutShape(Shape& cutter)
	{
		m_CutShape = &cutter;
	}

private:
	void PositionCutShape(const Vector3& splitPoint, const Vector3& splitNormal);
	void ClearRegisteredIntersections();
	void RegisterIntersections();
	void CreateNewFaces();
	void SwapInNewFaces(Shape& newShape);

	IntersectionFinder m_IntersectionFinder;
	FaceSplitter m_FaceSplitter;

	Shape * m_CutShape = nullptr;
	Shape * m_OriginalShape = nullptr;

	std::vector<Face*> m_NewInIntersectionFaces;
	std::vector<Face*> m_NewOutsideFaces;

	std::vector<FaceFaceIntersection<Vector3>> m_FaceFaceIntersections;
};
