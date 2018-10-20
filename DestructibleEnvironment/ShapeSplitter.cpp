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

void ShapeSplitter::SplitFaces(const std::vector<Face*>& toSplit, std::vector<Face*>& inIntersection, std::vector<Face*>& outside, std::vector<Face*>& detachedFrom, std::vector<Face*>& notSplit)
{
	m_FaceSplitter.SetOutputs(outside, inIntersection);

	for (auto it = toSplit.begin(); it != toSplit.end(); it++)
	{
		auto& f = **it;

		if (f.HasRegisteredIntersections())
		{
			DetachFace(f, detachedFrom);
			m_FaceSplitter.SplitFace(f);
			// return f to pool
		}
		else
		{
			notSplit.emplace_back(&f);
		}
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

	m_FaceNotSplitFromOriginal.clear();
	m_FaceNotSplitFromCut.clear();

	auto& orignalsFaces = m_OriginalShape->GetFaces();
	SplitFaces(orignalsFaces, m_NewInIntersectionFacesFromOriginal, m_NewOutsideFacesFromOriginal, m_FacesDetachedFromOriginal, m_FaceNotSplitFromOriginal);

	auto& cutShapesFaces = m_CutShape->GetFaces();
	SplitFaces(orignalsFaces, m_NewInIntersectionFacesFromCut, m_NewOutsideFacesFromCut, m_FacesDetachedFromCut, m_FaceNotSplitFromCut);
}

void ShapeSplitter::ReturnUnusedCutShapeFaces()
{
	auto& faces = m_CutShape->GetFaces();

	for (auto it = faces.begin(); it != faces.end(); it++)
	{
		auto f = *it;
		if (!f->HasRegisteredIntersections() &&
			(m_PerFaceData[f->GetIdForSplitter()].RelationshipWithOtherShape == FaceRelationshipWithOtherShape::Unkown))
		{
			// return f to pool
		}
	}
}

void ShapeSplitter::EnsureDetachedFromFacesAreFullyLinked()
{
	LinkForDetachedFrom(m_FacesDetachedFromOriginal);

	for (auto it = m_FacesDetachedFromCut.begin(); it != m_FacesDetachedFromCut.end(); it++)
	{
		auto& face = **it;
		if (m_PerFaceData[face.GetIdForSplitter()].RelationshipWithOtherShape != FaceRelationshipWithOtherShape::Unkown)
			LinkForDetachedFrom(face);
	}
}

void ShapeSplitter::InitNewShapes(const std::vector<Shape*>& newShapes)
{
	// copy the original shapes transform to use as the ref transform.
	// copy is needed becasue the original shape is re-used in the
	// new shapes.
	auto refTran = m_OriginalShape->GetTransform();

	for (auto it = newShapes.begin(); it != newShapes.end(); it++)
		(*it)->OnAllFacesAdded(refTran);
}

void ShapeSplitter::Split(const Vector3& splitPoint, const Vector3& splitNormal, Shape& orginalShape, std::vector<Shape*>& newShapes)
{
	m_OriginalShape = &orginalShape;
	m_CutShape = &m_CutShapeCreator.Create(orginalShape.GetTransform(), splitPoint, splitNormal); // may need to convert these from world space

	RegisterIntersections();

	AddPerFaceData(m_OriginalShape->GetFaces(), FaceRelationshipWithOtherShape::Unkown);
	AddPerFaceData(m_CutShape->GetFaces(), FaceRelationshipWithOtherShape::Unkown);

	SplitFaces();

	LinkForInside(m_NewInIntersectionFacesFromCut);
	LinkForInside(m_NewInIntersectionFacesFromOriginal);
	
	// This iterating must happen before reversing faces for outside.
	// This is so that the non split, in intersection, faces from cut shape are assigned.
	m_FaceIterator.SetShapeToUseNext(orginalShape);
	m_FaceIterator.CreateShapes(m_NewInIntersectionFacesFromOriginal, newShapes);

	ReverseFacesForOutside();

	LinkForOutside(m_NewOutsideFacesFromOriginal);
	LinkForOutside(m_NewOutsideFacesFromReversing);
	
	m_FaceIterator.CreateShapes(m_NewOutsideFacesFromOriginal, newShapes);

	EnsureDetachedFromFacesAreFullyLinked();
	InitNewShapes(newShapes);

	ReturnUnusedCutShapeFaces();
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

	AddPerFaceData(m_NewOutsideFacesFromReversing, FaceRelationshipWithOtherShape::NotInIntersection);
}