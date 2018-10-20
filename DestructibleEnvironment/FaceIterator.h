#pragma once
#include <array>
#include <vector>
#include "Shape.h"
#include "Face.h"
#include "PerFaceSplitData.h"

class FaceIterator
{
public:

	void SetShapeToUseNext(Shape& s)
	{
		m_ShapeToUseNext = &s;
	}

	void SetPerFaceData(std::vector<PerFaceSplitData>& perFaceData)
	{
		m_PerFaceData = &perFaceData;
	}

	void CreateShapes(const std::vector<Face*>& faces, std::vector<Shape*>& newShapes)
	{
		auto root = FindNextRootFace(faces);

		while (root)
		{
			newShapes.emplace_back(&CreateShape(*root));
			root = FindNextRootFace(faces);
		}
	}

private:
	Shape & GetNextShapeToUse()
	{
		auto s = m_ShapeToUseNext;
		if (s)
		{
			m_ShapeToUseNext = nullptr;
			return *s;
		}
		// get one from pool
		return *(new Shape());
	}

	Shape& CreateShape(Face& rootFace)
	{
		auto& shape = GetNextShapeToUse();
		auto& perFaceData = *m_PerFaceData;

		auto shapesRelationship = perFaceData[rootFace.GetIdForSplitter()].RelationshipWithOtherShape;

		// iterate the faces and add to new shape
		// check the relationship with the root face is consistent
		// and assign it when it is unkown.

		return shape;
	}

	Face* FindNextRootFace(const std::vector<Face*>& faces)
	{
		auto& perFaceData = *m_PerFaceData;

		for (auto it = faces.begin(); it != faces.end(); it++)
		{
			auto face = *it;
			if (!perFaceData[face->GetIdForSplitter()].Visited)
				return face;
		}
		return nullptr;
	}

	Shape * m_ShapeToUseNext = nullptr;
	std::vector<PerFaceSplitData>* m_PerFaceData = nullptr;
};
