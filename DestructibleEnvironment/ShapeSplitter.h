#pragma once
#include <vector>
#include "Shape.h"
#include "Face.h"
#include "ShapeEdge.h"
#include "ShapePoint.h"
#include "CutShapeCreator.h"
#include "ShapeEdgesCreator.h"
#include "FaceSplitter.h"
#include "CutPathCreator.h"
#include "IntersectionFinder.h"
#include "FaceIterator.h"

class ShapeSplitter
{
private:
	void ResetHashCounters()
	{
		ObjectWithHash<Face>::ResetNextHashCounter();
		ObjectWithHash<ShapePoint>::ResetNextHashCounter();
		ObjectWithHash<ShapeEdge>::ResetNextHashCounter();
	}

	void CreateEdgesFromCPs()
	{
		auto& cps = m_CutPathCreator.GetCutPaths();

		for (auto it = cps.Begin(); it != cps.End(); it++)
			m_EdgesCreator.CreateEdges(*it);
	}

	void CreateNewInsideFaces(const Shape& cutShape)
	{
		m_NewInsideFaces.clear();
		m_NewInsideFacesFromCutShape.clear();

		auto& fcpColls = m_CutPathCreator.GetFacesCutPathCollections();

		for (auto it = fcpColls.Begin(); it != fcpColls.End(); it++)
		{
			auto& fcpColl = *it;
			auto sizeBefore = m_NewInsideFaces.size();
			m_FaceSplitter.SpitInside(fcpColl, m_NewInsideFaces);

			if (&fcpColl.GetFace().GetShape() == &cutShape)
			{
				for (auto i = sizeBefore; i < m_NewInsideFaces.size(); i++)
					m_NewInsideFacesFromCutShape.emplace_back(m_NewInsideFaces[i]);
			}
		}
	}

	void CreateNewOutsideFaces(const Shape& originalShape)
	{
		m_NewOutsideFaces.clear();

		auto& fcpColls = m_CutPathCreator.GetFacesCutPathCollections();

		for (auto it = fcpColls.Begin(); it != fcpColls.End(); it++)
		{
			auto& fcpColl = *it;
			if (&fcpColl.GetFace().GetShape() == &originalShape)
				m_FaceSplitter.SplitOutside(fcpColl, m_NewOutsideFaces);
		}
	}

	void CreateNewInsideShapes(Shape& original, std::vector<Shape*>& newShapes)
	{
		m_FaceIterator.SetShapeToUseNext(original);
		m_FaceIterator.CreateShapes(m_NewInsideFaces, newShapes, FaceRelationshipWithOtherShape::InIntersection);
	}

	void CreateNewOutsideShapes(std::vector<Shape*>& newShapes)
	{
		m_FaceIterator.CreateShapes(m_NewOutsideFaces, newShapes, FaceRelationshipWithOtherShape::NotInIntersection);
	}

	void CreateReversedFace(const Face& f)
	{

	}

	void CreateReversedFaces(const Shape& cutShape)
	{
		for (auto it = m_NewInsideFaces.begin(); it != m_NewInsideFacesFromCutShape.end(); it++)
		{
			// create reveserd
		}

		auto& faces = cutShape.GetFaces();
		for (auto it = faces.begin(); it != faces.end(); it++)
		{
			auto& f = **it;
			if (f.HashIsAssigned() && !m_CutPathCreator.FaceIsSplit(f))
			{
				// If the hash is assiged and the face is not split, it means that the hash was
				// assigned by the face iterator when it created the inside shape. Hence the face
				// is inside and needs reversing.

				// create reversed
			}
		}
	}

public:
	ShapeSplitter()
	{
	}

	void Split(const Vector3& splitPoint, const Vector3& splitNormal, Shape& originalShape, std::vector<Shape*>& newShapes)
	{
		ResetHashCounters();

		auto& cutShape = m_CutShapeCreator.Create(originalShape.GetTransform(), splitPoint, splitNormal);

		m_Intersections.clear();
		m_IntersectionFinder.FindEdgeFaceIntersections(originalShape, cutShape, m_Intersections);

		m_CutPathCreator.GeneratePaths(m_Intersections);

		CreateEdgesFromCPs();

		CreateNewInsideFaces(cutShape);
		CreateNewInsideShapes(originalShape, newShapes);

		CreateReversedFaces(cutShape);

		CreateNewOutsideFaces(originalShape);
		CreateNewOutsideShapes(newShapes);
	}

private:
	CutShapeCreator m_CutShapeCreator;
	FaceSplitter m_FaceSplitter;
	ShapeEdgesCreator m_EdgesCreator;
	CutPathCreator m_CutPathCreator;
	IntersectionFinder m_IntersectionFinder;
	FaceIterator m_FaceIterator;

	std::vector<EdgeFaceIntersection> m_Intersections;

	std::vector<Face*> m_NewInsideFacesFromCutShape;

	std::vector<Face*> m_NewInsideFaces;
	std::vector<Face*> m_NewOutsideFaces;
};
