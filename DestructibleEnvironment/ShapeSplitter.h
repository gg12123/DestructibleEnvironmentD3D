#pragma once
#include <vector>
#include "IntersectionFinder.h"
#include "FaceSplitter.h"
#include "FaceLinker.h"
#include "FaceFaceIntersection.h"
#include "PerFaceSplitData.h"
#include "FaceIterator.h"
#include "CutShapeCreator.h"

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
	ShapeSplitter()
	{
		m_FaceLinker.SetPerFaceData(m_PerFaceData);
	}

	void Split(const Vector3& splitPoint, const Vector3& splitNormal, Shape& orginalShape, std::vector<Shape*>& newShapes);

private:
	void ClearRegisteredIntersections();
	void RegisterIntersections();
	void DetachFace(Face& face, std::vector<Face*>& detachedFrom);
	void SplitFaces();
	void SplitFaces(const std::vector<Face*>& toSplit, std::vector<Face*>& inIntersection, std::vector<Face*>& outside, std::vector<Face*>& detachedFrom, std::vector<Face*>& notSplit);
	void AddPerFaceData(const std::vector<Face*>& faces, FaceRelationshipWithOtherShape relationship);
	void ReverseFacesForOutside();
	void ReturnUnusedCutShapeFaces();
	void EnsureDetachedFromFacesAreFullyLinked();
	void InitNewShapes(const std::vector<Shape*>& newShapes);

	template<class T>
	void LinkForDetachedFrom(T& arg)
	{
		m_FaceLinker.Link(arg, m_FaceNotSplitFromCut, m_FaceNotSplitFromOriginal, m_NewInIntersectionFacesFromCut, m_NewInIntersectionFacesFromOriginal, m_NewOutsideFacesFromOriginal, m_NewOutsideFacesFromReversing);
	}
	template<class T>
	void LinkForOutside(T& arg)
	{
		m_FaceLinker.Link(arg, m_FaceNotSplitFromOriginal, m_NewOutsideFacesFromOriginal, m_NewOutsideFacesFromReversing);
	}
	template<class T>
	void LinkForInside(T& arg)
	{
		m_FaceLinker.Link(arg, m_FaceNotSplitFromCut, m_FaceNotSplitFromOriginal, m_NewInIntersectionFacesFromCut, m_NewInIntersectionFacesFromOriginal);
	}

	IntersectionFinder m_IntersectionFinder;
	FaceSplitter m_FaceSplitter;
	FaceLinker m_FaceLinker;
	FaceIterator m_FaceIterator;
	CutShapeCreator m_CutShapeCreator;

	Shape * m_CutShape = nullptr; // will need a cut shape generator
	Shape * m_OriginalShape = nullptr;

	std::vector<Face*> m_NewInIntersectionFacesFromCut;
	std::vector<Face*> m_NewInIntersectionFacesFromOriginal;

	std::vector<Face*> m_NewOutsideFacesFromCut;
	std::vector<Face*> m_NewOutsideFacesFromOriginal;

	std::vector<Face*> m_FacesDetachedFromCut;
	std::vector<Face*> m_FacesDetachedFromOriginal;

	std::vector<Face*> m_FaceNotSplitFromOriginal;
	std::vector<Face*> m_FaceNotSplitFromCut;

	std::vector<Face*> m_NewOutsideFacesFromReversing;

	std::vector<PerFaceSplitData> m_PerFaceData;

	std::vector<FaceFaceIntersection<Vector3>> m_FaceFaceIntersections;

	IdGenerator m_Ids;
};
