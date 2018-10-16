#pragma once
#include <vector>
#include "IntersectionFinder.h"
#include "FaceSplitter.h"
#include "FaceFaceIntersection.h"
#include "PerFaceSplitData.h"

class Shape;
class Face;

class IdGenerator
{
public:
	int NextId()
	{

	}
};

class ShapeSplitter
{
public:
	void Split(const Vector3& splitPoint, const Vector3& splitNormal, Shape& orginalShape, Shape& newShape);

private:
	void PositionCutShape(const Vector3& splitPoint, const Vector3& splitNormal);
	void ClearRegisteredIntersections();
	void RegisterIntersections();
	void DetachFace(Face& face, std::vector<Face*>& detachedFrom);
	void SplitFaces();
	void SplitFaces(const std::vector<Face*>& toSplit, std::vector<Face*>& inIntersection, std::vector<Face*>& outside, std::vector<Face*>& detachedFrom);
	void SwapInNewFaces(Shape& newShape);
	void AddPerFaceData(const std::vector<Face*>& faces, FaceRelationshipWithOtherShape relationship);
	void ReverseFacesForOutside();

	IntersectionFinder m_IntersectionFinder;
	FaceSplitter m_FaceSplitter;

	Shape * m_CutShape = nullptr; // will need a cut shape generator
	Shape * m_OriginalShape = nullptr;

	std::vector<Face*> m_NewInIntersectionFacesFromCut;
	std::vector<Face*> m_NewInIntersectionFacesFromOriginal;

	std::vector<Face*> m_NewOutsideFacesFromCut;
	std::vector<Face*> m_NewOutsideFacesFromOriginal;

	std::vector<Face*> m_FacesDetachedFromCut;
	std::vector<Face*> m_FacesDetachedFromOriginal;

	std::vector<Face*> m_NewOutsideFacesFromReversing;

	std::vector<PerFaceSplitData> m_PerFaceData;

	std::vector<FaceFaceIntersection<Vector3>> m_FaceFaceIntersections;

	IdGenerator m_Ids;
};
