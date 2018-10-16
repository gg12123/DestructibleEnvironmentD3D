#include "ShapeSplitter.h"
#include "Shape.h"
#include "Face.h"

static Face& CreateReversedFace(const Face& f)
{
	auto reversed = new Face(); // pool

	auto& points = f.GetCachedPoints();
	
	for (int i = points.size() - 1; i >= 0; i--)
		reversed->AddPoint(points[i]);

	return *reversed;
}

void ShapeSplitter::DetachFace(Face& face, std::vector<Face*>& detachedFrom)
{
	auto& links = face.GetLinkedFaces();

	for (auto it1 = links.begin(); it1 != links.end(); it1++)
	{
		auto& linksForEdge = *it1;
		for (auto it2 = linksForEdge.begin(); it2 != linksForEdge.end(); it2++)
		{
			auto linkedFace = *it2;

			if (linkedFace->HasRegisteredIntersections())
				continue;

			auto& data = m_PerFaceData[linkedFace->GetIdForSplitter()];
			if (!data.HasBeenAddedToDetachedFromList)
			{
				data.HasBeenAddedToDetachedFromList = true;
				detachedFrom.emplace_back(linkedFace);
			}
		}
	}

	face.DetachLinks();
}

void ShapeSplitter::AddPerFaceData(const std::vector<Face*>& faces, FaceRelationshipWithOtherShape relationship)
{
	for (auto it = faces.begin(); it != faces.end(); it++)
	{
		auto id = m_Ids.NextId();
		(*it)->SetIdForSplitter(id);
		m_PerFaceData.emplace_back(PerFaceSplitData(relationship));
	}
}

void ShapeSplitter::SplitFaces(const std::vector<Face*>& toSplit, std::vector<Face*>& inIntersection, std::vector<Face*>& outside, std::vector<Face*>& detachedFrom)
{
	m_FaceSplitter.SetOutputs(outside, inIntersection);

	for (auto it = toSplit.begin(); it != toSplit.end(); it++)
	{
		auto& f = **it;
		DetachFace(f, detachedFrom);
		m_FaceSplitter.SplitFace(f);
	}

	AddPerFaceData(outside, FaceRelationshipWithOtherShape::NotInIntersection);
	AddPerFaceData(inIntersection, FaceRelationshipWithOtherShape::InIntersection);
}

void ShapeSplitter::SplitFaces()
{
	m_NewInIntersectionFacesFromCut.clear();
	m_NewInIntersectionFacesFromOriginal.clear();

	m_NewOutsideFacesFromCut.clear();
	m_NewOutsideFacesFromOriginal.clear();

	m_FacesDetachedFromCut.clear();
	m_FacesDetachedFromOriginal.clear();

	auto& orignalsFaces = m_OriginalShape->GetFaces();
	SplitFaces(orignalsFaces, m_NewInIntersectionFacesFromOriginal, m_NewOutsideFacesFromOriginal, m_FacesDetachedFromOriginal);

	auto& cutShapesFaces = m_CutShape->GetFaces();
	SplitFaces(orignalsFaces, m_NewInIntersectionFacesFromCut, m_NewOutsideFacesFromCut, m_FacesDetachedFromCut);
}

void ShapeSplitter::SwapInNewFaces(Shape& newShape)
{
	auto& refTransform = m_OriginalShape->GetTransform();

	//newShape.SwapInNewFaces(m_NewInIntersectionFaces, refTransform);
	//m_OriginalShape->SwapInNewFaces(m_NewOutsideFaces, refTransform);
}

void ShapeSplitter::Split(const Vector3& splitPoint, const Vector3& splitNormal, Shape& orginalShape, Shape& newShape)
{
	m_OriginalShape = &orginalShape;

	// create cut shape first
	// make it so the cut shapes points are expressed in the same local space as the orignal shapes points

	PositionCutShape(splitPoint, splitNormal);

	RegisterIntersections();

	AddPerFaceData(m_OriginalShape->GetFaces(), FaceRelationshipWithOtherShape::Unkown);
	AddPerFaceData(m_CutShape->GetFaces(), FaceRelationshipWithOtherShape::Unkown);

	SplitFaces();

	// create in intersection shapes

	ReverseFacesForOutside();

	// create outside shapes

	// ensure detached from are fully linked

	SwapInNewFaces(newShape);
}

static void ClearIntersections(const std::vector<Face*>& faces)
{
	for (auto it = faces.begin(); it != faces.end(); it++)
		(*it)->ClearRegisteredIntersections();
}

void ShapeSplitter::ClearRegisteredIntersections()
{
	ClearIntersections(m_CutShape->GetFaces());
	ClearIntersections(m_OriginalShape->GetFaces());
}

void ShapeSplitter::RegisterIntersections()
{
	ClearRegisteredIntersections();

	m_FaceFaceIntersections.clear();
	m_IntersectionFinder.FindFaceFaceIntersections(*m_OriginalShape, *m_CutShape, m_FaceFaceIntersections);

	for (auto it = m_FaceFaceIntersections.begin(); it != m_FaceFaceIntersections.end(); it++)
	{
		auto& x = *it;

		x.Face1->RegisterIntersection(x);
		x.Face2->RegisterIntersection(x);
	}
}

void ShapeSplitter::ReverseFacesForOutside()
{
	m_NewOutsideFacesFromReversing.clear();

	for (auto it = m_NewInIntersectionFacesFromCut.begin(); it != m_NewInIntersectionFacesFromCut.end(); it++)
		m_NewOutsideFacesFromReversing.emplace_back(CreateReversedFace(**it));

	auto& cutShapesFaces = m_CutShape->GetFaces();
	for (auto it = cutShapesFaces.begin(); it != cutShapesFaces.end(); it++)
	{
		auto& f = **it;

		if (m_PerFaceData[f.GetIdForSplitter()].RelationshipWithOtherShape == FaceRelationshipWithOtherShape::InIntersection)
			m_NewOutsideFacesFromReversing.emplace_back(CreateReversedFace(f));
	}
}